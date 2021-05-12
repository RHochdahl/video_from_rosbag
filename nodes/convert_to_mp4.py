#!/usr/bin/env python

import rospy
import os
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


def convert(selected_image_topics, bag_path):
    bag_dir_name, bag_file_name = os.path.split(bag_path)
    bag = rosbag.Bag(bag_path)
    bridge = CvBridge()
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')

    for image_topic_name in selected_image_topics:
        frames_per_second = selected_image_topics[image_topic_name]
        output_path, output_file_name = get_output_path(bag_dir_name, bag_file_name, image_topic_name)
        out = None

        rospy.loginfo("Saving images from " + image_topic_name + " in " + bag_file_name + " as " + output_file_name + " ...")

        num_msg = bag.get_message_count(topic_filters=image_topic_name)

        n = 0
        progress = 0
        show_progress(progress)
        for topic, msg, t in bag.read_messages(topics=[image_topic_name]):
            img = bridge.imgmsg_to_cv2(msg, "bgr8")

            if not out:
                (height, width, channels) = img.shape
                out = cv2.VideoWriter(output_path, fourcc, frames_per_second, (width, height))

            out.write(img)

            n += 1
            if (100*n//num_msg > progress):
                progress = 100*n//num_msg
                show_progress(progress)

        out.release()
    bag.close()
    
    rospy.loginfo("All selected image topics have been converted. The mp4-Files are stored in " + bag_dir_name)


def get_output_path(bag_dir_name, bag_file_name, image_topic_name):
    name_extension = ""
    for char in image_topic_name:
        if char != "/":
            name_extension += char
        else:
            name_extension += "--"

    output_file_name = bag_file_name.split(".")[0] + name_extension + ".mp4"
    output_path = os.path.join(bag_dir_name, output_file_name)

    return output_path, output_file_name


def show_progress(progress):
    message = '[' + progress*'=' + '>' + (100-progress)*'_' + ']'
    if progress < 100:
        rospy.loginfo(message + "\033[A")
    else:
        rospy.loginfo(message)


if __name__ == "__main__":
    rospy.init_node("convert_to_video", log_level=rospy.DEBUG)

    gui = GUI()
    
    bag_path = select_bag_path(gui)
    image_topic_list = get_image_topic_list(bag_path)
    selected_image_topics = select_image_topic(gui, image_topic_list)
    convert(selected_image_topics, bag_path)
        