cmake_minimum_required(VERSION 3.0.2)
project(team_project)


find_package(catkin REQUIRED COMPONENTS
  carla_msgs
  roscpp
)

catkin_package(
  LIBRARIES team_project
  CATKIN_DEPENDS carla_msgs roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /home/ailab/ws/devel/include/carla_msgs
)

add_executable(first_node src/firstnode.cpp)
add_dependencies(first_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(first_node ${catkin_LIBRARIES})
