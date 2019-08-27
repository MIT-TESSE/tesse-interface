#!/usr/bin/env python

import unittest
import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation

import tf
import tf2_ros
from sensor_msgs.msg import CameraInfo

import tesse_ros_bridge.utils

class TestUtilsOffline(unittest.TestCase):

    def test_parse_metadata_0(self):
        """Test correct metatdata parsing from xml message offline."""
        data = ET.parse("data/metadata_0.xml")
        data_str = ET.tostring(data.getroot())

        dict = tesse_ros_bridge.utils.parse_metadata(data_str)
        self.assertEqual(dict['position'], [-5.692576, 2.499105, 10.63836])
        self.assertEqual(dict['quaternion'], [0, 0.5372996, 0, 0.8433914])
        self.assertEqual(dict['velocity'], [0, -0.0004944276, 0])
        self.assertEqual(dict['ang_vel'], [0, 0, 0])
        self.assertEqual(dict['acceleration'], [0, 0.001516496, 0])
        self.assertEqual(dict['ang_accel'], [0, 0, 0])
        self.assertEqual(dict['time'], 7.935)
        self.assertEqual(dict['collision_status'], False)

    def test_parse_cam_data_0(self):
        """Test corrrect camera metadata parsing from xml message offline."""
        data = ET.parse('data/cam_data_0.xml')
        data_str = ET.tostring(data.getroot())

        dict = tesse_ros_bridge.utils.parse_cam_data(data_str)
        self.assertEqual(dict['name'], 'depth')
        self.assertEqual(dict['id'], 3)
        self.assertEqual(dict['parameters']['height'], 480)
        self.assertEqual(dict['parameters']['width'], 720)
        self.assertEqual(dict['parameters']['fov'], 37.84929)
        self.assertEqual(dict['position'], [-0.05, 0, 0])
        self.assertEqual(dict['quaternion'], [0, 0, 0, 1])
        self.assertEqual(dict['draw_distance']['far'], 50)
        self.assertEqual(dict['draw_distance']['near'], 0.3)

    def test_metadata_to_odom_0(self):
        """Test correct conversion of metadata to Odometry message offline."""
        data = ET.parse("data/metadata_0.xml")
        data_str = ET.tostring(data.getroot())

        dict = tesse_ros_bridge.utils.parse_metadata(data_str)

        odom = tesse_ros_bridge.utils.metadata_to_odom(dict, 0, "f1", "f2")
        self.assertEqual(odom.header.stamp, 0)
        self.assertEqual(odom.header.frame_id, "f1")
        self.assertEqual(odom.child_frame_id, "f2")
        self.assertEqual(odom.pose.pose.position.x, dict['position'][0])
        self.assertEqual(odom.pose.pose.position.y, dict['position'][1])
        self.assertEqual(odom.pose.pose.position.z, dict['position'][2])
        self.assertEqual(odom.pose.pose.orientation.x, dict['quaternion'][0])
        self.assertEqual(odom.pose.pose.orientation.y, dict['quaternion'][1])
        self.assertEqual(odom.pose.pose.orientation.z, dict['quaternion'][2])
        self.assertEqual(odom.pose.pose.orientation.w, dict['quaternion'][3])
        self.assertEqual(odom.twist.twist.linear.x, dict['velocity'][0])
        self.assertEqual(odom.twist.twist.linear.y, dict['velocity'][1])
        self.assertEqual(odom.twist.twist.linear.z, dict['velocity'][2])
        self.assertEqual(odom.twist.twist.angular.x, dict['ang_vel'][0])
        self.assertEqual(odom.twist.twist.angular.y, dict['ang_vel'][1])
        self.assertEqual(odom.twist.twist.angular.z, dict['ang_vel'][2])

    def test_metadata_to_imu_0(self):
        """Test correct conversion of metadata to Imu message offline."""
        data = ET.parse("data/metadata_0.xml")
        data_str = ET.tostring(data.getroot())

        dict = tesse_ros_bridge.utils.parse_metadata(data_str)

        imu = tesse_ros_bridge.utils.metadata_to_imu(dict, 0, "f", [0, 9.81, 0])
        self.assertEqual(imu.header.frame_id, "f")
        self.assertEqual(imu.header.stamp, 0)
        self.assertEqual(imu.angular_velocity.x, dict['ang_vel'][0])
        self.assertEqual(imu.angular_velocity.y, dict['ang_vel'][1])
        self.assertEqual(imu.angular_velocity.z, dict['ang_vel'][2])
        self.assertEqual(imu.linear_acceleration.x, dict['acceleration'][0])
        self.assertEqual(imu.linear_acceleration.y, dict['acceleration'][1]+9.81)
        self.assertEqual(imu.linear_acceleration.z, dict['acceleration'][2])

        # TODO(marcus): add checks on angular velocity between two frames

    def test_generate_camera_info(self):
        """Test generation of CameraInfo messages for left and right cameras."""
        data = ET.parse('data/cam_data_0.xml')
        data_str = ET.tostring(data.getroot())

        dict = tesse_ros_bridge.utils.parse_cam_data(data_str)

        (left, right) = tesse_ros_bridge.utils.generate_camera_info(dict, dict)
        self.assertEqual(left.header.frame_id, "left_cam")
        self.assertEqual(right.header.frame_id, "right_cam")
        self.assertEqual(left.width, dict['parameters']['width'])
        self.assertEqual(left.height, dict['parameters']['height'])
        self.assertEqual(right.width, dict['parameters']['width'])
        self.assertEqual(right.height, dict['parameters']['height'])

        # TODO(marcus): add more checks

    def test_make_camera_info_msg(self):
        """Test generation of CameraInfo message for one camera"""
        # TODO(marcus): complete
        pass

    def test_right_handed_frame_0(self):
        """Test rotation matrices to ensure they are right handed offline."""
        data = ET.parse("data/metadata_0.xml")
        data_str = ET.tostring(data.getroot())

        dict = tesse_ros_bridge.utils.parse_metadata(data_str)
        quat = np.array(dict['quaternion'])
        rot = tf.transformations.quaternion_matrix(quat)
        self.assertEqual(np.linalg.det(rot), 1)

    def test_process_metadata_0(self):
        """Test correct transformation of Unity metadata at rest, offline."""
        data = ET.parse("data/metadata_0.xml")
        data_str = ET.tostring(data.getroot())

        pre = tesse_ros_bridge.unity_T_enu
        post = tesse_ros_bridge.lh_T_rh

        dict = tesse_ros_bridge.utils.parse_metadata(data_str)
        proc = tesse_ros_bridge.utils.process_metadata(dict, pre, post,
            dict['time']-2, [0,0,0], [0,0,0])

        transform = proc['transform']
        transform_R = transform[:3,:3]
        transform_t = transform[:3,3]

        # First check the transformation matrix.
        # Right-handed check.
        self.assertEqual(np.linalg.det(transform_R), 1)
        # X and Z axes are switched:
        self.assertEqual(transform_t[0], dict['position'][0])
        self.assertEqual(transform_t[1], dict['position'][2])
        self.assertEqual(transform_t[2], dict['position'][1])

        truth_quat = tf.transformations.quaternion_from_matrix((pre.dot(
            post)).dot(tf.transformations.quaternion_matrix(
                dict['quaternion'])))
        self.assertTrue(np.allclose(proc['quaternion'], truth_quat))

        self.assertTrue(np.allclose(proc['velocity'],
            post[:3,:3].dot(dict['velocity'])))
        self.assertTrue(np.allclose(proc['ang_vel'],
            post[:3,:3].dot(dict['ang_vel'])))

        self.assertTrue(np.allclose(proc['acceleration'], proc['velocity']*0.5))
        self.assertTrue(np.allclose(proc['ang_accel'], proc['ang_vel']*0.5))

        self.assertEqual(proc['time'], dict['time'])
        self.assertEqual(proc['collision_status'], dict['collision_status'])

    def test_process_metadata_1(self):
        """Test correct transformation of Unity metadata in motion, offline."""
        data_1 = ET.parse("data/metadata_1.xml")
        data_2 = ET.parse("data/metadata_2.xml")
        data_1_str = ET.tostring(data_1.getroot())
        data_2_str = ET.tostring(data_2.getroot())

        pre = np.array([[1,0,0,0],
                        [0,0,1,0],
                        [0,1,0,0],
                        [0,0,0,1]])
        post = np.array([[1,0,0,0],
                         [0,-1,0,0],
                         [0,0,1,0],
                         [0,0,0,1]])

        dict_1 = tesse_ros_bridge.utils.parse_metadata(data_1_str)
        dict_2 = tesse_ros_bridge.utils.parse_metadata(data_2_str)
        proc_1 = tesse_ros_bridge.utils.process_metadata(dict_1, pre, post, 0,
            [0,0,0], [0,0,0])
        proc_2 = tesse_ros_bridge.utils.process_metadata(dict_2, pre, post,
            dict_1['time'], proc_1['velocity'], proc_1['ang_vel'])

        transform_1_R = proc_1['transform']
        transform_2_R = proc_2['transform']
        transform_1_R[:,3] = transform_2_R[:,3] = np.array([0,0,0,1])

        dt = dict_2['time'] - dict_1['time']
        axis_angle_1 = Rotation.from_quat(
            tf.transformations.quaternion_from_matrix(np.transpose(
                transform_1_R).dot(transform_2_R))).as_rotvec() / dt

        self.assertTrue(np.allclose(axis_angle_1, proc_2['ang_vel']))

        expected_accel_1 = (proc_2['velocity']-proc_1['velocity']) / \
            (proc_2['time']-proc_1['time'])
        expected_ang_accel_1 = (proc_2['ang_vel']-proc_1['ang_vel']) / \
            (proc_2['time']-proc_1['time'])
        self.assertTrue(np.allclose(proc_2['acceleration'], expected_accel_1))
        self.assertTrue(np.allclose(proc_2['ang_accel'], expected_ang_accel_1))


if __name__ == '__main__':
    unittest.main()
