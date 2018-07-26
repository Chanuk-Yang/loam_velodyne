#!/usr/bin/env python

import rospy
import numpy as np
from ros_numpy import numpify, msgify
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2

import tf
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

pub1 = rospy.Publisher("vlp_merge/velodyne_points", PointCloud2, queue_size=1)
pub2 = rospy.Publisher("vlp_merge/merge_time", Float64, queue_size=1)

class listener():
    def __init__(self):
        rospy.init_node('listen_and_merge', anonymous=True)
        self.start_time = 0

        self.center_flag = False
        self.left_flag = False
        self.right_flag = False
        self.merhing_flag = False

        self.tf_flag_center = False
        self.tf_flag_left = False
        self.tf_flag_right = False

        self.subscriber_tf = rospy.Subscriber("/tf", TFMessage, self.callback_tf)
        self.subscriber_center = rospy.Subscriber("/vlp_center/velodyne_points", PointCloud2, self.callback_c)
        self.subscriber_left = rospy.Subscriber("/vlp_left/velodyne_points", PointCloud2, self.callback_l)
        self.subscriber_right = rospy.Subscriber("/vlp_right/velodyne_points", PointCloud2, self.callback_r)
        print('======================================================================================')

        rospy.spin()


    def callback_tf(self, data):

        self.tf_set_temp = data
        self.tf_set = self.tf_set_temp.transforms[0]

        if self.tf_set.child_frame_id == '/vlp_center':
            self.tf_center = self.tf_set
            rospy.loginfo(rospy.get_caller_id() + "  TF_center is logged")
            self.tf_flag_center = True
        elif self.tf_set.child_frame_id == '/vlp_left':
            self.tf_left = self.tf_set
            rospy.loginfo(rospy.get_caller_id() + "  TF_left is logged")
            self.tf_flag_left = True
        elif self.tf_set.child_frame_id == '/vlp_right':
            self.tf_right = self.tf_set
            rospy.loginfo(rospy.get_caller_id() + "  TF_right is logged")
            self.tf_flag_right = True

        if self.tf_flag_center and self.tf_flag_left and self.tf_flag_right:
            rospy.loginfo(rospy.get_caller_id() + "  all_TF_info is logged")
            self.subscriber_tf.unregister()

    def callback_c(self, data):
        rospy.loginfo(rospy.get_caller_id() + "  Center points is logged")
        self.center_flag = True
        self.point_center = data
        if self.center_flag and self.left_flag and self.right_flag and not self.merhing_flag:
            self.merged_point_publish(self.point_center, self.point_left, self.point_right)

    def callback_l(self, data):
        rospy.loginfo(rospy.get_caller_id() + "  Left points is logged")
        self.left_flag = True
        self.point_left = data
        if self.center_flag and self.left_flag and self.right_flag and not self.merhing_flag:
            self.merged_point_publish(self.point_center, self.point_left, self.point_right)

    def callback_r(self, data):
        rospy.loginfo(rospy.get_caller_id() + "  Right points is logged")
        self.right_flag = True
        self.point_right = data
        if self.center_flag and self.left_flag and self.right_flag and not self.merhing_flag:
            self.merged_point_publish(self.point_center, self.point_left, self.point_right)


    def merged_point_publish(self, point_center, point_left, point_right):
        print('start merging...')
        self.merhing_flag = True
        self.start_time = rospy.Time.now().to_sec()

        point_left = do_transform_cloud(point_left, self.tf_left)
        point_right = do_transform_cloud(point_right, self.tf_right)

        point_center = numpify(point_center)
        point_left = numpify(point_left)
        point_right = numpify(point_right)

        point_set = np.append(np.append(point_center, point_left), point_right)
        # point_set = np.append(point_right, point_left)

        merged_points = msgify(PointCloud2, point_set)

        pub1.publish(merged_points)
        self.elapsed = rospy.Time.now().to_sec() - self.start_time
        pub2.publish(self.elapsed)
        print('Merged PointCloud Data Set is Published')

        self.center_flag = False
        self.left_flag = False
        self.right_flag = False
        self.merhing_flag = False

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
