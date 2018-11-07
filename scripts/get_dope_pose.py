#!/usr/bin/python

import sys
import time
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import PointCloud2 as msg_PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import inspect
import ctypes
import struct


class DopePoseProcessing:

    def __init__(self, object_names):
        '''
        object_names only supports the following strings:

        cracker, gelatin, meat, mustard, soup, sugar
        '''
        self.object_pose_subscribers = {}
        base_topic = "/dope/pose_"
        for object_name in object_names:
            sub = rospy.Subscriber(
                base_topic + object_name, PoseStamped, self.pose_callback, object_name)
            self.object_pose_subscribers[object_name] = sub

        self.object_poses = {}
        for object_name in object_names:
            self.object_poses[object_name] = None

    def pose_callback(self, msg, object_name):
        print "callback"
        self.object_poses[object_name] = msg.pose


if __name__ == "__main__":
    rospy.init_node("dope_pose_processing")

    object_names = ["cracker", "mustard"]
    dope_pose_processing = DopePoseProcessing(object_names)

    while not rospy.is_shutdown():
        print dope_pose_processing.object_poses
        rospy.sleep(1.0)
        # rospy.spin()
