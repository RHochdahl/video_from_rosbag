cmake_minimum_required(VERSION 3.0.2)
project(video_from_rosbag)

find_package(catkin REQUIRED
  cv_bridge
  sensor_msgs
  message_generation
)

find_package(OpenCV REQUIRED)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    sensor_msgs
  DEPENDS
    OpenCV
)
