cmake_minimum_required(VERSION 3.0.2)
project(team_project)


find_package(catkin REQUIRED COMPONENTS
  carla_msgs
  std_msgs
  geometry_msgs
  derived_object_msgs
  roscpp
  sensor_msgs
  nav_msgs
  custom_msgs
  visualization_msgs
  tf2
  rviz_visual_tools
)



catkin_package(
  LIBRARIES team_project

  CATKIN_DEPENDS 
  carla_msgs 
  roscpp 
  std_msgs 
  geometry_msgs 
  derived_object_msgs 
  sensor_msgs 
  nav_msgs
  custom_msgs
  visualization_msgs
  tf2
  rviz_visual_tools

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /home/ailab/carla-ros-bridge/project_ws/devel/include/carla_msgs
  /home/ailab/carla-ros-bridge/project_ws/msg
  /usr/include/eigen3
)

add_executable(plan_node src/main.cpp src/plan_node.cpp src/whole.cpp src/planner.cpp src/trajectorygenerator.cpp src/collisionChecker.cpp)
add_dependencies(plan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(plan_node ${catkin_LIBRARIES})


# add_executable(localization_node src/localization_main.cpp src/localization.cpp src/planner.cpp)
# add_dependencies(localization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(localization_node ${catkin_LIBRARIES})

add_executable(trajectory_node src/trajectory_main.cpp src/trajectorygenerator.cpp src/planner.cpp src/collisionChecker.cpp)
add_dependencies(trajectory_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_node ${catkin_LIBRARIES})

