cmake_minimum_required(VERSION 3.0.2)
project(workstation)

## Compile as C++17, supported in ROS Noetic and newer
add_compile_options(-std=c++17)

# This line ensures we can find RBDL with the FindRBDL.cmake file in the source directory
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  gazebo_ros
  sensor_msgs
)

# Non-ROS dependencies
find_package(Eigen3 REQUIRED)
find_package(RBDL COMPONENTS URDFReader REQUIRED)
find_package(osqp REQUIRED)

# Set all our include paths to a single variable INCLUDES for ease of access
set(INCLUDES
  include 
  ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}/eigen3  
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  pupper_command.msg
  pupper_reply.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS RBDL Eigen3
)

# Create a library for our class to be accesible to other packages
add_library(${PROJECT_NAME} 
  src/PupperCplusplusController/PupperModel.cpp
  src/PupperCplusplusController/PupperWBC.cpp
  src/PupperCplusplusController/WBCTaskMaster.cpp
)

# Link all necessary libraries to our library
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  osqp::osqp 
  ${RBDL_LIBRARY} 
  ${RBDL_URDFReader_LIBRARY}
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}/eigen3
)

# Create an executable file and link its dependencies
add_executable(PupperController 
  src/PupperCplusplusController/PupperControl.cpp
)
target_link_libraries(PupperController ${PROJECT_NAME})

# ROS node for controlling the real life pupper
add_executable(PupperNode
  src/PupperCplusplusController/pupper_ros_node.cpp
)
target_link_libraries(PupperNode ${PROJECT_NAME})
