cmake_minimum_required(VERSION 3.0.2)
project(controller_verification)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  carla_msgs
  tf
  
)

catkin_package(
  CATKIN_DEPENDS   roscpp std_msgs geometry_msgs nav_msgs tf carla_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /home/ailab/carla-ros-bridge/project_ws/devel/include/carla_msgs
)

# add_executable(waypoint_publisher src/waypoint_publisher.cpp)
# target_link_libraries(waypoint_publisher ${catkin_LIBRARIES})

add_executable(longitudinal_controller src/longitudinal_controller.cpp)
target_link_libraries(longitudinal_controller ${catkin_LIBRARIES})

# add_executable(stanley_controller_node src/stanley_controller.cpp src/waypoint_loader.cpp)
# target_link_libraries(stanley_controller_node ${catkin_LIBRARIES})
