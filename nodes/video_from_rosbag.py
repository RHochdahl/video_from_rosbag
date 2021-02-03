#!/usr/bin/env python

PACKAGE = "video_from_rosbag"

import rospy
import rospkg
import os
# import rosnode
import ffmpeg
import roslaunch
import rosbag
from PyQt4 import QtGui, QtCore
import sys


def select_bag_path():
    # GetFilePath
    files = QtGui.QFileDialog.getOpenFileNames(caption="Select bag file", directory=os.path.expanduser("~"), filter="*bag")
    while len(files) != 1:
        print("Error: Please select a bag file")
        files = QtGui.QFileDialog.getOpenFileNames(caption="Select bag file", directory=os.path.expanduser("~"), filter="*bag")

    bag_path = str(files[0])
    rospy.logdebug(type(bag_path))
    return bag_path


def get_image_topic_list(bag_path):
    bag = rosbag.Bag(bag_path)
    image_topics = {}
    topic_info = bag.get_type_and_topic_info()[1]
    for topic in topic_info:
        topic_type = topic_info[topic][0]
        if topic_type == "sensor_msgs/Image":
            frames_per_second = topic_info[topic][3]
            image_topics[topic] = frames_per_second
    bag.close()
    rospy.logdebug(image_topics)
    return image_topics


def select_image_topic(image_topic_list):
    rospy.logdebug("start")
    selected_image_topics = {}

    rospy.logdebug("select")

    win = QtGui.QWidget()
    scrollArea = QtGui.QScrollArea()
    scrollArea.setWidgetResizable(True)
    scrollAreaWidgetContents = QtGui.QWidget(scrollArea)
    scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 380, 247))
    scrollArea.setWidget(scrollAreaWidgetContents)
    layout = QtGui.QGridLayout()
    verticalLayoutScroll = QtGui.QVBoxLayout(scrollAreaWidgetContents)
    layoutIndex = 0

    checkboxes = []
    for select in image_topic_list:
        checkbox = QtGui.QCheckBox(select)
        verticalLayoutScroll.addWidget(checkbox)
        layoutIndex += 1
        checkboxes.append(checkbox)

    layout.addWidget(scrollArea)
    btn = QtGui.QPushButton("OK")
    btn.clicked.connect(app.quit)
    layout.addWidget(btn, layoutIndex, 0)
    layoutIndex += 1

    win.setLayout(layout)
    win.setWindowTitle("Select image topic")
    win.show()
    app.exec_()

    result = {}
    for (checkbox, select) in zip(checkboxes, image_topic_list):
        result[select] = checkbox.isChecked()

    for k, v in result.items():
        if v:
            selected_image_topics[k] = image_topic_list[k]

    # TODO: select from list
    # image_topic = image_topic_list[0]
    return selected_image_topics


def extract(bag_path, image_topic_name):
    rosbag_node = roslaunch.core.Node("rosbag", "play", args=bag_path)
    extract_node = roslaunch.core.Node("image_view", "extract_images", name="extract", output="screen", cwd="ROS_HOME", remap_args=[("image", image_topic_name)])

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    rosbag_process = launch.launch(rosbag_node)
    extract_process = launch.launch(extract_node)

    while rosbag_process.is_alive():
        rospy.sleep(1)

    extract_process.stop()


def convert_to_mp4(frames_per_second, output_path):
    pkg_directory = rospkg.RosPack().get_path(PACKAGE)
    temp_directory = os.path.join(pkg_directory, "temp")
    if not os.path.isdir(temp_directory):
        os.mkdir(temp_directory)

    jpg_directory = os.path.join(os.path.expanduser("~"), ".ros")

    for filename in os.listdir(jpg_directory):
        if filename.startswith("frame") and filename.endswith(".jpg"):
            os.rename(os.path.join(jpg_directory, filename), os.path.join(temp_directory, filename))

    ffmpeg.input(os.path.join(temp_directory, "frame%04d.jpg"), framerate=frames_per_second).output(output_path).run()

    for file in os.listdir(temp_directory):
        os.remove(os.path.join(temp_directory, file))
    os.rmdir(temp_directory)


if __name__ == "__main__":
    rospy.init_node("convert_to_video", log_level=rospy.DEBUG)

    app = QtGui.QApplication(sys.argv)
    
    bag_path = select_bag_path()
    bag_dir_name, bag_file_name = os.path.split(bag_path)
    image_topic_list = get_image_topic_list(bag_path)
    rospy.logdebug("don't die")
    selected_image_topics = select_image_topic(image_topic_list)
    for selected_image_topic in selected_image_topics:
        rospy.logdebug(selected_image_topics)
        extract(bag_path, selected_image_topic)
        name_extension = ""
        for char in selected_image_topic:
            if char != "/":
                name_extension += char
            else:
                name_extension += "--"
        output_file_name = bag_file_name.split(".")[0] + name_extension + ".mp4"
        output_path = os.path.join(bag_dir_name, output_file_name)
        convert_to_mp4(selected_image_topics[selected_image_topic], output_path)
