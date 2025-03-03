cmake_minimum_required(VERSION 3.0.2)
project(motor_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  dynamixel_sdk
  robot_controller
)

include_directories(
  # include/
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include

)

add_executable(motor_node src/motor_node.cpp src/Motor_Controller.cpp src/Motor_Cluster.cpp)


## Link against dependencies
target_link_libraries(motor_node ${catkin_LIBRARIES})
add_dependencies(motor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(scan_motors src/scan_motors.cpp)
target_link_libraries(scan_motors ${catkin_LIBRARIES})


add_executable(motor_test src/test_motor.cpp src/Motor_Controller.cpp src/Motor_Cluster.cpp)


## Link against dependencies
target_link_libraries(motor_test ${catkin_LIBRARIES})
add_dependencies(motor_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


