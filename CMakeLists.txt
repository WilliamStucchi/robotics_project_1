cmake_minimum_required(VERSION 2.8.3)
project(test_pkg)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
            roscpp
            std_msgs
            message_generation
            geometry_msgs
            sensor_msgs
            dynamic_reconfigure
            tf
            tf2
            tf2_ros)

## Add custom message and services and their dependencies
add_message_files(
  FILES
  wheels_rpm.msg
)

add_service_files(
  FILES
  Position.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
 )

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build listener

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(velocity_reader src/compute_rpm.cpp)
add_dependencies(velocity_reader ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_reader ${catkin_LIBRARIES})

add_executable(wheel_states src/wheel_states.cpp)
add_dependencies(wheel_states ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheel_states ${catkin_LIBRARIES})
