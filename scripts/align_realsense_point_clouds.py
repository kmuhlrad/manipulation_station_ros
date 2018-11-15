#!/usr/bin/python

import numpy as np

import rospy
import ros_numpy
import tf_conversions

from sensor_msgs.msg import PointCloud2

class AlignPointClouds:

    def __init__(self):
        '''
        object_names only supports the following strings:

        cracker, gelatin, meat, mustard, soup, sugar
        '''

        self.left_pc_sub = rospy.Subscriber(
            "/cam_left/depth/color/points", PointCloud2, self.left_pc_callback)
        self.middle_pc_sub = rospy.Subscriber(
            "/cam_middle/depth/color/points", PointCloud2, self.middle_pc_callback)
        self.right_pc_sub = rospy.Subscriber(
            "/cam_right/depth/color/points", PointCloud2, self.right_pc_callback)

        self.aligned_left_pc_pub = pub = rospy.Publisher(
                "/cam_left/depth/color/aligned/points", PointCloud2, queue_size = 10)
        self.aligned_middle_pc_pub = pub = rospy.Publisher(
                "/cam_middle/depth/color/aligned/points", PointCloud2, queue_size = 10)
        self.aligned_right_pc_pub = pub = rospy.Publisher(
                "/cam_right/depth/color/aligned/points", PointCloud2, queue_size = 10)

        self._flip_x_z = tf_conversions.transformations.euler_matrix(-1.5707963267948966, -0.0, -1.5707963267948966)

    def _numpy_conversion_and_alignment(self, R, cloud_msg):
        cloud_array = ros_numpy.numpify(cloud_msg)

        homogenous_points = np.ones((4, cloud_array['x'].shape[0]), dtype=np.float32)
        homogenous_points[0, :] = cloud_array['x'].T
        homogenous_points[1, :] = cloud_array['y'].T
        homogenous_points[2, :] = cloud_array['z'].T

        transformed_points = R.dot(homogenous_points)[:3, :].T

        transformed_cloud_array = np.zeros(transformed_points.shape[0],
            dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)])
        transformed_cloud_array['x'] = transformed_points[:, 0]
        transformed_cloud_array['y'] = transformed_points[:, 1]
        transformed_cloud_array['z'] = transformed_points[:, 2]
        transformed_cloud_array['rgb'] = cloud_array['rgb']

        transformed_msg = ros_numpy.msgify(PointCloud2, transformed_cloud_array)
        transformed_msg.header.stamp = rospy.Time.now()
        transformed_msg.header.frame_id = "world"

        return transformed_msg

    def left_pc_callback(self, msg):
        R = tf_conversions.transformations.euler_matrix(-1.67937, 0.209218, -0.449982)
        R = np.dot(R, self._flip_x_z)
        R[:3, 3] = [-0.226819, 0.459576, 0.482885]

        transformed_msg = self._numpy_conversion_and_alignment(R, msg)

        self.aligned_left_pc_pub.publish(transformed_msg)

    def middle_pc_callback(self, msg):
        R = tf_conversions.transformations.euler_matrix(-0.0851225, 1.20071, 3.05952)
        R = np.dot(R, self._flip_x_z)
        R[:3, 3] = [0.787323, -0.0186122, 1.01686]

        transformed_msg = self._numpy_conversion_and_alignment(R, msg)

        self.aligned_middle_pc_pub.publish(transformed_msg)

    def right_pc_callback(self, msg):
        R = tf_conversions.transformations.euler_matrix(1.65259, 0.223617, 0.476623)
        R = np.dot(R, self._flip_x_z)
        R[:3, 3] = [-0.206765, -0.469184, 0.48013]

        transformed_msg = self._numpy_conversion_and_alignment(R, msg)

        self.aligned_right_pc_pub.publish(transformed_msg)


if __name__ == "__main__":
    rospy.init_node("align_point_clouds")

    align = AlignPointClouds()

    while not rospy.is_shutdown():
        rospy.spin()