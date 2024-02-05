cmake_minimum_required(VERSION 3.0.2)
project(PROJECT)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  cv_bridge
  geometry_msgs
  image_transport
  message_generation
  naoqi_bridge_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Target.msg
  Target_list.msg
)

add_service_files(
    DIRECTORY srv
    FILES
    Vision.srv
    Control.srv
    ShootBow.srv
    SpeakWords.srv
    CheckHeadTouchResponse.srv
)



## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs   naoqi_bridge_msgs  sensor_msgs   std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES PROJECT
#  CATKIN_DEPENDS action_lib cv_bridge geometry_msgs image_transport message_generation message_runtime naoqi_bridge_msgs roscpp rospy sensor_msgs std_msgs std_srvs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
