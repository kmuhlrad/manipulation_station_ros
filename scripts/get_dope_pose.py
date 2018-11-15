#!/usr/bin/python

import time
import rospy
import numpy as np

import tf_conversions
from geometry_msgs.msg import PoseStamped, TransformStamped



class DopePoseProcessing:

    def __init__(self, object_names):
        '''
        object_names only supports the following strings:

        cracker, gelatin, meat, mustard, soup, sugar
        '''
        self.object_poses = {}
        self.object_pose_subscribers = {}
        self.object_pose_publishers = {}
        base_topic = "/dope/pose_"
        for object_name in object_names:
            self.object_poses[object_name] = None

            sub = rospy.Subscriber(
                base_topic + object_name, PoseStamped, self.pose_callback, object_name)
            self.object_pose_subscribers[object_name] = sub

            pub = rospy.Publisher(
                "/transformed_pose_" + object_name, PoseStamped, queue_size = 10)
            self.object_pose_publishers[object_name] = pub


    def transform_dope_pose(self, dope_pose):
        dope_pose_matrix = np.eye(4)
        dope_pose_matrix[0, 3] = dope_pose.position.x
        dope_pose_matrix[1, 3] = dope_pose.position.y
        dope_pose_matrix[2, 3] = dope_pose.position.z
        dope_pose_matrix[:3, :3] = tf_conversions.transformations.quaternion_matrix(
            [dope_pose.orientation.x, dope_pose.orientation.y, dope_pose.orientation.z, dope_pose.orientation.w])[:3, :3]


        flip_x_z = tf_conversions.transformations.euler_matrix(-1.5707963267948966, -0.0, -1.5707963267948966)

        R = tf_conversions.transformations.euler_matrix(1.65259, 0.223617, 0.476623)
        R = np.dot(R, flip_x_z)
        R[:3, 3] = [-0.206765, -0.469184, 0.48013]

        return R.dot(dope_pose_matrix)

    def pose_callback(self, msg, object_name):
        dope_pose = msg.pose
        if dope_pose is not None:
            self.object_poses[object_name] = self.transform_dope_pose(dope_pose)

        p = self.transform_dope_pose(dope_pose)
        new_pose = PoseStamped()
        new_pose.header.stamp = rospy.Time.now()
        new_pose.header.frame_id = "world"

        new_pose.pose.position.x = p[0, 3]
        new_pose.pose.position.y = p[1, 3]
        new_pose.pose.position.z = p[2, 3]

        q = tf_conversions.transformations.quaternion_from_matrix(p)
        new_pose.pose.orientation.x = q[0]
        new_pose.pose.orientation.y = q[1]
        new_pose.pose.orientation.z = q[2]
        new_pose.pose.orientation.w = q[3]


        self.object_pose_publishers[object_name].publish(new_pose)

    def write_poses_to_file(self, filename):
        with open(filename, "wb") as f:
            for object_name in self.object_poses:
                f.write(object_name + "\n")
                f.write(str(self.object_poses[object_name]) + "\n")


if __name__ == "__main__":
    rospy.init_node("dope_pose_processing")

    # object_names = ["mustard", "soup", "meat", "cracker", "gelatin"]
    object_names = ["mustard", "soup"]
    dope_pose_processing = DopePoseProcessing(object_names)

    # rospy.spin()
    rospy.sleep(10.0)

    while not rospy.is_shutdown():
        all_seen = True
        for object_name in object_names:
            if dope_pose_processing.object_poses[object_name] is None:
                all_seen = False
        if all_seen:
            dope_pose_processing.write_poses_to_file("dope_poses.txt")
            break
        rospy.sleep(1.0)
        # rospy.spin()

    print "done"
