// Copyright 2016 Robo@FIT

#include "art_pr2_grasping/objects.h"
#include <string>
#include <vector>
#include <set>

namespace art_pr2_grasping
{
Objects::Objects(boost::shared_ptr<tf::TransformListener> tfl, std::string target_frame)
{
  tfl_ = tfl;
  object_type_srv_ = nh_.serviceClient<art_msgs::getObjectType>("/art/db/object_type/get");
  obj_sub_ = nh_.subscribe("/art/object_detector/object_filtered", 1, &Objects::detectedObjectsCallback, this);
  target_frame_ = target_frame;
  
  ce_set_pose_srv_ = nh_.serviceClient<art_msgs::PoseStampedTrigger>("/art/collision_env/detected/set_pose");

}

void Objects::setPaused(bool paused, bool clear)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  paused_ = paused;

  /*if (paused && clear)
  {
    TObjectMap::iterator it;
    for (it = objects_.begin(); it != objects_.end(); ++it)
    {
      if (isGrasped(it->first) || it->second.on_table)
        continue;
      visual_tools_->cleanupCO(it->first);
    }
  }*/
}

void Objects::clear()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  /*for (std::set<std::string>::iterator i = grasped_objects_.begin(); i != grasped_objects_.end(); ++i)
  {
    visual_tools_->cleanupACO(*i);
  }

  visual_tools_->publishRemoveAllCollisionObjects();*/
  objects_.clear();
  grasped_objects_.clear();
}

bool Objects::isKnownObject(std::string id)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  TObjectMap::iterator it = objects_.find(id);
  return it != objects_.end();
}

std::vector<std::string> Objects::getObjects()
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  std::vector<std::string> tmp;
  TObjectMap::iterator it;
  for (it = objects_.begin(); it != objects_.end(); ++it)
  {
    tmp.push_back(it->first);
  }

  return tmp;
}

TObjectInfo Objects::getObject(std::string object_id)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  if (isKnownObject(object_id))
    return objects_[object_id];
  else
    throw std::invalid_argument("unknown object_id: " + object_id);
}

bool Objects::transformPose(geometry_msgs::PoseStamped& ps)
{
  try
  {
    if (tfl_->waitForTransform(target_frame_, ps.header.frame_id, ps.header.stamp, ros::Duration(1.0)))
    {
      tfl_->transformPose(target_frame_, ps, ps);
    }
    else
    {
      ROS_ERROR_NAMED("objects", "Transform between %s and %s not available!", target_frame_.c_str(),
                      ps.header.frame_id.c_str());
      return false;
    }
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR_NAMED("objects", "%s", ex.what());
    return false;
  }

  return true;
}

void Objects::detectedObjectsCallback(const art_msgs::InstancesArrayConstPtr& msg)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  // remove outdated objects
  TObjectMap::iterator it;
  std::vector<std::string> ids_to_remove;
  for (it = objects_.begin(); it != objects_.end(); ++it)
  {
    bool found = false;

    for (int i = 0; i < msg->instances.size(); i++)
    {
      if (msg->instances[i].object_id == it->first)
      {
        found = true;
        break;
      }
    }

    if (!found && !isGrasped(it->first))
    {
      ids_to_remove.push_back(it->first);
    }
  }

  for (int i = 0; i < ids_to_remove.size(); i++)
  {
    TObjectMap::iterator it;
    it = objects_.find(ids_to_remove[i]);
    objects_.erase(it);
  }

  // add and publish currently detected objects
  for (int i = 0; i < msg->instances.size(); i++)
  {
    geometry_msgs::PoseStamped ps;

    ps.header = msg->header;
    ps.pose = msg->instances[i].pose;

    if (!transformPose(ps))
    {
      ROS_WARN_NAMED("objects", "Failed to transform object.");
      continue;
    }

    if (isGrasped(msg->instances[i].object_id)) continue;

    if (isKnownObject(msg->instances[i].object_id))
    {
      objects_[msg->instances[i].object_id].pose = ps;
    }
    else
    {
      TObjCache::iterator it = obj_cache_.find(msg->instances[i].object_type);
      if (it == obj_cache_.end())
      {
        art_msgs::getObjectType srv;
        srv.request.name = msg->instances[i].object_type;

        if (!object_type_srv_.call(srv))
        {
          ROS_ERROR_NAMED("objects", "Failed to call object_type service");
          continue;
        }

        if (!srv.response.success)
        {
          ROS_ERROR_NAMED("objects", "Call to object_type service returned failure.");
          continue;
        }

        obj_cache_[msg->instances[i].object_type] = srv.response.object_type;
      }

      TObjectInfo obj;
      obj.object_id = msg->instances[i].object_id;
      obj.pose = ps;
      obj.type = obj_cache_[msg->instances[i].object_type];
      obj.on_table = msg->instances[i].on_table;
      objects_[msg->instances[i].object_id] = obj;
    }
  }
}

void Objects::setPose(std::string object_id, geometry_msgs::PoseStamped ps)
{

  art_msgs::PoseStampedTrigger srv;
  srv.request.ps = ps;
  srv.request.str = object_id;
  ce_set_pose_srv_.call(srv);

}

void Objects::setGrasped(std::string object_id, bool grasped)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  if (grasped)
  {
    ROS_DEBUG_NAMED("objects", "Setting object_id as grasped: %s", object_id.c_str());
    grasped_objects_.insert(object_id);
  }
  else
  {
    ROS_DEBUG_NAMED("objects", "Setting object_id as not grasped: %s", object_id.c_str());
    grasped_objects_.erase(object_id);

  }
}

}  // namespace art_pr2_grasping
