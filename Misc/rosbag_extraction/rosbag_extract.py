#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""
Extract images from a rosbag.
Borrowed from: https://gist.github.com/wngreene/835cda68ddd9c5416defce876a4d7dd9
Usage: python rosbag_extract.py ./just_traffic_light.bag ./out_images/ /image_raw
"""

import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")

    args = parser.parse_args()

    print "Extract images from %s on topic %s into %s" % (args.bag_file,
                                                          args.image_topic, args.output_dir)

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        #cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        #cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        #cv2.imwrite(os.path.join(args.output_dir, "frame%06i.png" % count), cv_img)
        cv2.imwrite(os.path.join(args.output_dir, "image%04i.jpg" % count), cv_img)
        print "Wrote image %i" % count

        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()
