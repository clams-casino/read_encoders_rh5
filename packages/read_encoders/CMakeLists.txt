cmake_minimum_required(VERSION 2.8.3)
project(read_encoders)

find_package(catkin REQUIRED COMPONENTS
  rospy
  duckietown_msgs
  message_generation
)

add_service_files(
  FILES
  CalibrateWheels.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()