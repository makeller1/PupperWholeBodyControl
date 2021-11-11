cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pupper_plugins)

# Find RBDL
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(RBDL COMPONENTS URDFReader REQUIRED)

# Find our package
find_package(catkin REQUIRED COMPONENTS roscpp workstation)

# So that we have a consistent quaternion definition
find_package(Eigen3 REQUIRED)

# Find Gazebo
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS} /usr/lib/x86_64-linux-gnu/)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Set include path
include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# Build our plugin
add_library(pupper_plugin SHARED pupper.cpp)
target_link_libraries(pupper_plugin 
    ${GAZEBO_LIBRARIES} 
    ${catkin_LIBRARIES} 
    ${RBDL_LIBRARY}
    ${RBDL_URDFReader_LIBRARY}
)
