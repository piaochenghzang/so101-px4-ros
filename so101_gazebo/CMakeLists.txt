cmake_minimum_required(VERSION 3.0.2)
project(so101_gazebo)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

find_package(gazebo REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(
  so101_joint_position_controller
  SHARED
  src/joint_position_controller.cpp
)

target_link_libraries(
  so101_joint_position_controller
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_library(
  so101_joint_state_publisher
  SHARED
  src/joint_state_publisher.cpp
)

target_link_libraries(
  so101_joint_state_publisher
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  add_executable(so101_command_watchdog_test tests/test_command_watchdog.cpp)
  target_compile_options(so101_command_watchdog_test PRIVATE -UNDEBUG)
  add_test(NAME so101_command_watchdog COMMAND so101_command_watchdog_test)
endif()
