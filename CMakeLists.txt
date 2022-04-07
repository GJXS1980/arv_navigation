cmake_minimum_required(VERSION 3.0.2)
project(arv_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  amcl
  gmapping
  map_server
  move_base
  roscpp
  rospy
  urdf
  xacro
)

###################################
## catkin specific configuration ##
###################################
catkin_package()


#############
## Install ##
#############

install(
DIRECTORY launch maps params scripts src params_test rviz news_v1
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)





#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/include/amcl.launch)
  roslaunch_add_file_check(launch/amcl_demo.launch)
  roslaunch_add_file_check(launch/include/gmapping.launch)
  roslaunch_add_file_check(launch/gmapping_demo.launch)
  roslaunch_add_file_check(launch/include/move_base.launch)
  roslaunch_add_file_check(launch/odom_navigation_demo.launch)
endif()
