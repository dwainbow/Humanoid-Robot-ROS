cmake_minimum_required(VERSION 3.0.2)
project(robot_controller)


find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  controller_state.msg
  Button_data.msg
  Axis_data.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS joy roscpp sensor_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)


add_executable(controller_node src/controller_node.cpp src/Controller.cpp)


## Link against dependencies
target_link_libraries(controller_node ${catkin_LIBRARIES})
add_dependencies(controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


