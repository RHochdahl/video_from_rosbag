#!/usr/bin/env python

import rospy
import rospkg
import os
import ffmpeg
import rosbag
from video_from_rosbag.gui import GUI
import cv2
from cv_bridge import CvBridge

PACKAGE = "video_from_rosbag"


def select_bag_path(gui):
    # GetFilePath
    bag_path = gui.select_bag()
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
    return image_topics


def select_image_topic(gui, image_topic_list):
    selected_image_topics = gui.select_topics(image_topic_list)
    return selected_image_topics


def create_temp_directory():
    pkg_directory = rospkg.RosPack().get_path(PACKAGE)
    temp_directory = os.path.join(pkg_directory, "temp")

    if not os.path.isdir(temp_directory):
        os.mkdir(temp_directory)

    return temp_directory


def extract(bag_path, image_topic_name, temp_directory):
    rospy.loginfo("Extracting images from " + image_topic_name + " in " + bag_path + "...")

    bag = rosbag.Bag(bag_path)
    bridge = CvBridge()

    num_msg = bag.get_message_count(topic_filters=image_topic_name)

    n = 0
    progress = 0
    show_progress(progress)
    for topic, msg, t in bag.read_messages(topics=[image_topic_name]):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite(os.path.join(temp_directory, "frame" + str(n) + ".jpg"), img)

        n += 1
        if (100*n//num_msg > progress):
            progress = 100*n//num_msg
            show_progress(progress)

    bag.close()


def show_progress(progress):
    message = '[' + progress*'=' + '>' + (100-progress)*'_' + ']'
    if progress < 100:
        rospy.loginfo(message + "\033[A")
    else:
        rospy.loginfo(message)


def convert_to_mp4(frames_per_second, output_path, temp_directory):
    rospy.loginfo("Converting images to mp4...")

    jpg_directory = os.path.join(os.path.expanduser("~"), ".ros")

    for filename in os.listdir(jpg_directory):
        if filename.startswith("frame") and filename.endswith(".jpg"):
            os.rename(os.path.join(jpg_directory, filename), os.path.join(temp_directory, filename))

    ffmpeg.input(os.path.join(temp_directory, "frame%d.jpg"), framerate=frames_per_second).output(output_path).run()

    for file in os.listdir(temp_directory):
        os.remove(os.path.join(temp_directory, file))
    os.rmdir(temp_directory)
    rospy.loginfo("conversion complete")


if __name__ == "__main__":
    rospy.init_node("convert_to_video", log_level=rospy.DEBUG)

    gui = GUI()
    
    bag_path = select_bag_path(gui)
    bag_dir_name, bag_file_name = os.path.split(bag_path)
    image_topic_list = get_image_topic_list(bag_path)
    selected_image_topics = select_image_topic(gui, image_topic_list)
    for selected_image_topic in selected_image_topics:
        temp_dir = create_temp_directory()
        extract(bag_path, selected_image_topic, temp_dir)
        name_extension = ""
        for char in selected_image_topic:
            if char != "/":
                name_extension += char
            else:
                name_extension += "--"
        output_file_name = bag_file_name.split(".")[0] + name_extension + ".mp4"
        output_path = os.path.join(bag_dir_name, output_file_name)
        convert_to_mp4(selected_image_topics[selected_image_topic], output_path, temp_dir)
    
    rospy.loginfo("All selected image topics have been converted. The mp4-Files are stored in " + bag_dir_name)
        