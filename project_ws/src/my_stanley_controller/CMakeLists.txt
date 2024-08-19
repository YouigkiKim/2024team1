cmake_minimum_required(VERSION 3.0.2)
project(my_stanley_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  carla_msgs
  custom_msgs
  ackermann_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf carla_msgs custom_msgs  ackermann_msgs

)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   /home/ailab/carla-ros-bridge/project_ws/devel/include/carla_msgs
# )



# add_executable(stanley_controller_node src/stanley_controller.cpp src/waypoint_loader.cpp)
# target_link_libraries(stanley_controller_node ${catkin_LIBRARIES})

add_executable(Controller_node src/real_stanley_PID.cpp)
target_link_libraries(Controller_node ${catkin_LIBRARIES})
