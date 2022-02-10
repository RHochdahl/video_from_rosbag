cmake_minimum_required(VERSION 3.0.2)
project(video_from_rosbag)

find_package(catkin REQUIRED
  cv_bridge
  sensor_msgs
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    sensor_msgs
    rospy
)

install(DIRECTORY
        launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS nodes/convert_to_mp4.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})