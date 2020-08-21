cmake_minimum_required(VERSION 2.8.3)
project(icog_face_tracker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  facebox.msg
  faces.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(tracker_node src/tracker.cpp)

add_dependencies(tracker_node ${tracker_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracker_node ${catkin_LIBRARIES})
add_dependencies(tracker_node icog_face_tracker_generate_messages_cpp)

target_link_libraries(tracker_node ${OpenCV_LIBS})

