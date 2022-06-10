cmake_minimum_required(VERSION 2.8.3)
project(gps_common CXX)

############
## Catkin ##
############

set(DEPS
  message_filters
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  rospy
)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${DEPS}
)

#catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
    GPSStatus.msg
    GPSFix.msg
)

generate_messages(DEPENDENCIES 
  nav_msgs
  sensor_msgs
  std_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    message_runtime
    ${DEPS}
)

###########
## Build ##
###########


#############
## Install ##
#############

