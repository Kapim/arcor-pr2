cmake_minimum_required(VERSION 2.8.3)
project(art_pr2_interface)

find_package(catkin REQUIRED COMPONENTS
  art_brain
  art_msgs
  geometry_msgs
  pr2_controllers_msgs
  rospy
  std_msgs
  std_srvs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS art_brain art_msgs geometry_msgs pr2_controller_msgs rospy std_msgs std_srvs
)