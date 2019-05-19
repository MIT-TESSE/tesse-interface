#!/usr/bin/env python

import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, Imu
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Pose, PoseStamped

# from tesse_ros_bridge import *
from tesse.msgs import *
from tesse.env import *


class TesseROSWrapper:

    def __init__(self):
        self.client_ip = rospy.get_param('~client_ip', '127.0.0.1')
        self.self_ip = rospy.get_param('~self_ip', '127.0.0.1')
        self.request_port = rospy.get_param('~request_port', '9000')
        self.receive_port = rospy.get_param('~receive_port', '9001')
        self.camera_width = rospy.get_param('~camera_width', '480')
        self.camera_height = rospy.get_param('~camera_height', '320')
        self.camera_fov = rospy.get_param('~camera_fov', '60')
        self.scene_id = rospy.get_param('~scene_id', '0')

        self.use_sim = rospy.get_param('/use_sim_time', False)


        self.env = Env(self.client_ip, self.self_ip,
                        self.request_port, self.receive_port)

        result = self.env.request(SetCameraParametersRequest(self.camera_height, self.camera_width, self.camera_fov))
        print result

        result = self.env.request(SceneRequest(self.scene_id))
        print result

        self.body_frame = 'body_frame'
        self.world_frame = 'world'

        self.bridge = CvBridge()

        self.imu_rate = 100
        self.image_rate = 20
        self.speedup_factor = 20 # for simulated time

        self.cameras=[(Camera.RGB_LEFT, Compression.OFF, Channels.SINGLE),
                 (Camera.RGB_RIGHT, Compression.OFF, Channels.SINGLE),
                 (Camera.SEGMENTATION, Compression.OFF, Channels.THREE),
                 (Camera.DEPTH, Compression.OFF, Channels.THREE)]

        self.left_image_pub = rospy.Publisher("/tesse/left_image", Image, queue_size=1)
        self.right_image_pub = rospy.Publisher("/tesse/right_image", Image, queue_size=1)
        self.segmented_image_pub = rospy.Publisher("/tesse/segmented_image", Image, queue_size=1)
        self.depth_image_pub = rospy.Publisher("/tesse/depth_image", Image, queue_size=1)
        self.image_publishers = [self.left_image_pub, self.right_image_pub, self.segmented_image_pub, self.depth_image_pub]

        self.imu_pub = rospy.Publisher("/tesse/imu", Imu, queue_size=1)
        self.true_pose_pub = rospy.Publisher("/tesse/true_pose", PoseStamped, queue_size=1)

        if self.use_sim:
            self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=1)

        self.imu_counter = 0

        # self.everything_timer = rospy.Timer(rospy.Duration(1.0/1000.0), self.everything_cb)
        # self.imu_timer = rospy.Timer(rospy.Duration(1.0/self.imu_rate), self.imu_cb)
        # self.image_timer = rospy.Timer(rospy.Duration(1.0/self.image_rate), self.image_cb)
        # self.clock_timer = rospy.Timer(rospy.Duration(1.0/1000.0), self.clock_cb)

        while not rospy.is_shutdown():
            self.everything_cb(None)

    def everything_cb(self, event):
        """ For running everything in one loop """
        publish_factor = int(self.imu_rate/self.image_rate)

        if self.imu_counter >= publish_factor:
            data_request = DataRequest(metadata=True, cameras=self.cameras)
            data_response = self.env.request(data_request)

            metadata = self.parse_metadata(data_response.data)

            for i in range(len(self.cameras)):
                if self.cameras[i][2] == Channels.SINGLE:
                    img_msg = self.bridge.cv2_to_imgmsg(data_response.images[i], 'mono8')
                elif self.cameras[i][2] == Channels.THREE:
                    img_msg = self.bridge.cv2_to_imgmsg(data_response.images[i], 'bgr8')
                img_msg.header.stamp = rospy.Time(metadata['time'])
                self.image_publishers[i].publish(img_msg)

            self.imu_counter = 0

        metadata = self.get_cam_metadata()

        if self.use_sim:
            sim_time = rospy.Time.from_sec(metadata['time'] / self.speedup_factor)
            self.clock_pub.publish(sim_time)

        imu = Imu()
        imu.header.stamp = rospy.Time(metadata['time'])
        imu.header.frame_id = self.body_frame

        imu.orientation.x = metadata['quaternion'][0]
        imu.orientation.y = metadata['quaternion'][1]
        imu.orientation.z = metadata['quaternion'][2]
        imu.orientation.w = metadata['quaternion'][3]

        imu.angular_velocity.x = metadata['ang_vel'][0]
        imu.angular_velocity.y = metadata['ang_vel'][1]
        imu.angular_velocity.z = metadata['ang_vel'][2]

        imu.linear_acceleration.x = metadata['ang_vel'][0]
        imu.linear_acceleration.y = metadata['ang_vel'][1]
        imu.linear_acceleration.z = metadata['ang_vel'][2]

        self.imu_pub.publish(imu)
        self.publish_true_pose(metadata)

        self.imu_counter += 1

    def imu_cb(self, event):
        """ Publish IMU updates from simulator to ROS """
        metadata = self.get_cam_metadata()
        self.publish_true_pose(metadata)

        imu = Imu()
        imu.header.stamp = rospy.Time(metadata['time'])
        imu.header.frame_id = self.body_frame

        imu.orientation.x = metadata['quaternion'][0]
        imu.orientation.y = metadata['quaternion'][1]
        imu.orientation.z = metadata['quaternion'][2]
        imu.orientation.w = metadata['quaternion'][3]

        imu.angular_velocity.x = metadata['ang_vel'][0]
        imu.angular_velocity.y = metadata['ang_vel'][1]
        imu.angular_velocity.z = metadata['ang_vel'][2]

        imu.linear_acceleration.x = metadata['ang_vel'][0]
        imu.linear_acceleration.y = metadata['ang_vel'][1]
        imu.linear_acceleration.z = metadata['ang_vel'][2]

        self.imu_pub.publish(imu)

    def image_cb(self, event):
        """ Publish images from simulator to ROS """
        data_request = DataRequest(self.cameras)
        data_response = self.env.request(data_request)

        metadata = self.parse_metadata(data_response.data)

        for i in range(len(self.cameras)):
            img_msg = self.bridge.cv2_to_imgmsg(data_response.images[i], 'bgr8')
            img_msg.header.stamp = rospy.Time(metadata['time'])
            self.image_publishers[i].publish(img_msg)

    def clock_cb(self, event):
        """ Publish simulated clock time """
        metadata = self.get_cam_metadata()

        sim_time = rospy.Time.from_sec(metadata['time'] / self.speedup_factor)
        self.clock_pub.publish(sim_time)

    def get_cam_metadata(self):
        """ Return camera meta data object """
        metadata_request = MetadataRequest()
        metadata_response = self.env.request(metadata_request)

        return self.parse_metadata(metadata_response)

    def parse_metadata(self, data):
        """ Parse metadata into a useful dictionary """
        dict = {}
        data_string = str(data)
        split_data = data_string.split()

        position = [float(split_data[5][3:-2]), float(split_data[6][3:-1]), float(split_data[7][3:-3])]
        quaternion = [float(split_data[9][3:-2]), float(split_data[10][3:-1]), float(split_data[11][3:-1]), float(split_data[12][3:-3])]
        velocity = [float(split_data[14][7:-2]), float(split_data[15][7:-1]), float(split_data[16][7:-3])]
        ang_vel = [float(split_data[19][11:-2]), float(split_data[20][11:-1]), float(split_data[22][2:-3])]
        acceleration = [float(split_data[24][8:-2]), float(split_data[25][8:-1]), float(split_data[26][8:-3])]
        ang_acceleration = [float(split_data[29][12:-2]), float(split_data[30][12:-1]), float(split_data[31][12:-3])]
        time = float(split_data[32][6:-7])
        collision_status = True if split_data[34][8:-1] == 'true' else False

        dict['position'] = position
        dict['quaternion'] = quaternion
        dict['velocity'] = velocity
        dict['ang_vel'] = ang_vel
        dict['acceleration'] = acceleration
        dict['ang_acceleration'] = ang_acceleration
        dict['time'] = time
        dict['collision_status'] = collision_status

        return dict

    def publish_true_pose(self, metadata):
        """ Publish the pose of the agent on the map frame.
            Requires a transform from body to map.
        """
        pass

if __name__ == '__main__':
    rospy.init_node("TesseROSWrapper_node")
    node = TesseROSWrapper()
    rospy.spin()
