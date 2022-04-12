cmake_minimum_required(VERSION 2.8.3)
project(test_pkg)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation geometry_msgs sensor_msgs)

## Add custom message and services and their dependencies
add_service_files(
  FILES
  Position.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
 )

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build listener

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pose_reader src/robot_pose.cpp)
add_dependencies(pose_reader ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_reader ${catkin_LIBRARIES})

add_executable(wheel_state_reader src/wheel_states.cpp)
add_dependencies(wheel_state_reader ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheel_state_reader ${catkin_LIBRARIES})
