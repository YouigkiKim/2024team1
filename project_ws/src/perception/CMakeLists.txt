cmake_minimum_required(VERSION 3.0.2)
project(perception)

## Compile as C++14, supported in ROS Noetic
add_compile_options(-std=c++14)

## Define macro to use deprecated proj_api.h
add_definitions(-DACCEPT_USE_OF_DEPRECATED_PROJ_API_H)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  tf
  tf2_ros
  message_generation
  carla_msgs
  custom_msgs
  # derived_object_msgs
)

## Find Python libraries and include directories
find_package(Python3 REQUIRED COMPONENTS Development NumPy)
# find_package(Python3 3.7 REQUIRED COMPONENTS Interpreter)



## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/matplotlib-cpp
  ${PROJECT_SOURCE_DIR}/src
)

## Declare message files to be built
add_message_files(
  FILES
  vehicle_state.msg
)

## Generate added messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros tf tf2_ros message_runtime carla_msgs custom_msgs # derived_object_msgs
)

## Declare C++ executables
add_executable(lidar_pre_node src/lidar_pre.cpp src/KalmanFilter.cpp)
add_executable(tf_broadcaster_node src/tf_broadcaster.cpp)
add_executable(localization_node src/localization.cpp src/KalmanFilter.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(lidar_pre_node
  ${catkin_LIBRARIES}
  ${Python3_LIBRARIES}
  proj
)

target_link_libraries(tf_broadcaster_node ${catkin_LIBRARIES})

target_link_libraries(localization_node
  ${catkin_LIBRARIES}
  ${Python3_LIBRARIES}
  proj
)

## Set RPATH to ensure Anaconda libraries are found at runtime
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${Python3_LIBRARY_DIRS}")
