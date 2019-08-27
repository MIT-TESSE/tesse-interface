#!/usr/bin/env python

import numpy as np
from scipy.spatial.transform import Rotation

import rospy
import tf
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class TestTesseNode:

    def __init__(self):
        self.world_frame_id = rospy.get_param("~world_frame_id")
        self.body_frame_id = rospy.get_param("~body_frame_id")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.imu_sub = rospy.Subscriber("imu", Imu, self.imu_cb)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_cb)

        self.initialized_imu = False
        self.last_timestamp_imu = None
        self.last_R = None
        self.last_lin_accel = None

        self.initialized_odom = False
        self.last_timestamp_odom = None
        self.last_lin_vel = None
        self.last_ang_vel = None

    def imu_cb(self, msg):
        transform_stamped = None
        this_R = None
        try:
            transform_stamped = self.tfBuffer.lookup_transform(
                self.world_frame_id, self.body_frame_id, msg.header.stamp)
            this_R = tf.transformations.quaternion_matrix(
                np.array([transform_stamped.transform.rotation.x,
                          transform_stamped.transform.rotation.y,
                          transform_stamped.transform.rotation.z,
                          transform_stamped.transform.rotation.w]))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass

        if transform_stamped is not None:
            if self.initialized_imu:
                    dt = (msg.header.stamp - self.last_timestamp_imu).to_sec()

                    expected_ang_vel = Rotation.from_quat(
                        tf.transformations.quaternion_from_matrix(
                            np.transpose(self.last_R).dot(this_R))).as_rotvec()

                    actual_ang_vel = np.array([msg.angular_velocity.x,
                                               msg.angular_velocity.y,
                                               msg.angular_velocity.z])

                    # assert(np.allclose(expected_ang_vel, actual_ang_vel))
                    if not np.allclose(expected_ang_vel, actual_ang_vel):
                        print "assertion failed for ang_vel..."

            else:
                self.initialized_imu = True

            self.last_timestamp_imu = msg.header.stamp
            self.last_R = this_R
            self.last_lin_accel = np.array([msg.linear_acceleration.x,
                                            msg.linear_acceleration.y,
                                            msg.linear_acceleration.z])

    def odom_cb(self, msg):
        this_lin_vel = np.array([msg.twist.twist.linear.x,
                                 msg.twist.twist.linear.y,
                                 msg.twist.twist.linear.z])

        if self.initialized_odom:
            dt = (msg.header.stamp-self.last_timestamp_odom).to_sec()

            expected_lin_accel = (this_lin_vel - self.last_lin_vel) / dt
            actual_lin_accel = self.last_lin_accel
            # assert(np.allclose(expected_lin_accel, actual_lin_accel))
            if not np.allclose(expected_lin_accel, actual_lin_accel):
                print "assertion failed for lin_accel..."

            # this_ang_vel = np.array([msg.twist.twist.angular.x,
            #                          msg.twist.twist.angular.y,
            #                          msg.twist.twist.angular.z])
            # expected_ang_accel = (this_ang_vel - self.last_ang_vel) / dt
            # actual_ang_accel = np.array([])
            # assert(np.allclose(expected_ang_accel, actual_ang_accel))

        else:
            self.initialized_odom = True

        self.last_timestamp_odom = msg.header.stamp
        self.last_lin_vel = this_lin_vel
        # self.last_ang_vel = this_ang_vel


if __name__ == "__main__":
    rospy.init_node("TestTesseNode")
    node = TestTesseNode()

    rospy.spin()
