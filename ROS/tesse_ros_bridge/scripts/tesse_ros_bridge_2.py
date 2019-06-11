#!/usr/bin/env python

import numpy as np
import cv2

import rospy
import tf
from std_msgs.msg import Header
from sensor_msgs.msg import Image, Imu, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped
from rosgraph_msgs.msg import Clock
from cv_bridge import CvBridge, CvBridgeError

import tesse_ros_bridge
from tesse.msgs import *
from tesse.env import *

from bs4 import BeautifulSoup


class TesseROSWrapper:

    def __init__(self):
        self.client_ip = rospy.get_param('~client_ip', '127.0.0.1')
        self.self_ip = rospy.get_param('~self_ip', '127.0.0.1')
        self.position_port = rospy.get_param('~position_port', '9000')
        self.metadata_port = rospy.get_param('~metadata_port', '9001')
        self.image_port = rospy.get_param('~image_port', '9002')
        self.camera_width = rospy.get_param('~camera_width', '480')
        self.camera_height = rospy.get_param('~camera_height', '320')
        self.camera_fov = rospy.get_param('~camera_fov', '60')
        self.scene_id = rospy.get_param('~scene_id', '0')
        self.use_sim = rospy.get_param('/use_sim_time', False)
        self.speedup_factor = rospy.get_param('~speedup_factor', 20)

        self.env = Env(simulation_ip=self.client_ip, own_ip=self.self_ip,
                    position_port=self.position_port,
                    metadata_port=self.metadata_port,
                    image_port=self.image_port)

        self.body_frame = 'base_link'
        self.world_frame = 'world'

        self.bridge = CvBridge()

        self.imu_rate = 100
        self.image_rate = 20

        self.cameras=[(Camera.RGB_LEFT, Compression.OFF, Channels.SINGLE),
                 (Camera.RGB_RIGHT, Compression.OFF, Channels.SINGLE),
                 (Camera.SEGMENTATION, Compression.OFF, Channels.THREE),
                 (Camera.DEPTH, Compression.OFF, Channels.THREE)]

        self.left_image_pub = rospy.Publisher("/tesse/left_cam", Image, queue_size=1)
        self.right_image_pub = rospy.Publisher("/tesse/right_cam", Image, queue_size=1)
        self.segmented_image_pub = rospy.Publisher("/tesse/segmentation", Image, queue_size=1)
        self.depth_image_pub = rospy.Publisher("/tesse/depth", Image, queue_size=1)
        self.image_publishers = [self.left_image_pub, self.right_image_pub, self.segmented_image_pub, self.depth_image_pub]

        self.imu_pub = rospy.Publisher("/tesse/imu", Imu, queue_size=1)
        self.odom_pub = rospy.Publisher("/tesse/odom", Odometry, queue_size=1)

        self.br = tf.TransformBroadcaster()

        # Set camera parameters once for the entire simulation:
        for camera in self.cameras:
            result = self.env.request(SetCameraParametersRequest(
                                            self.camera_height,
                                            self.camera_width,
                                            self.camera_fov,
                                            camera[0]))

        # Send scene request only if we don't want the default scene:
        if self.scene_id != 0:
            result = self.env.request(SceneRequest(self.scene_id))

        # One-shot camera_info publishing for VIO
        self.publish_camera_info(rospy.Time.now())

        self.imu_counter = 0

        # self.everything_timer = rospy.Timer(rospy.Duration(1.0/1000.0), self.everything_cb)
        # self.imu_timer = rospy.Timer(rospy.Duration(1.0/self.imu_rate), self.imu_cb)
        # self.image_timer = rospy.Timer(rospy.Duration(1.0/self.image_rate), self.image_cb)

        if self.use_sim:
            self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=1)
            # self.clock_timer = rospy.Timer(rospy.Duration(1.0/1000.0), self.clock_cb)

        while not rospy.is_shutdown():
            self.everything_cb(None)

    def everything_cb(self, event):
        """ For running everything in one loop """
        publish_factor = int(self.imu_rate/self.image_rate)

        if self.use_sim:
            self.clock_cb(None)

        self.imu_cb(None)
        self.imu_counter += 1

        if self.imu_counter >= publish_factor:
            self.image_cb(None)
            self.imu_counter = 0

    def imu_cb(self, event):
        """ Publish IMU updates from simulator to ROS as well as odom info
            and agent transform
        """
        metadata = self.get_metadata()
        timestamp = rospy.Time.now()
        imu = self.metadata_to_imu(timestamp, metadata)
        self.imu_pub.publish(imu)

        odom = self.metadata_to_odom(timestamp, metadata)
        self.odom_pub.publish(odom)

        self.br.sendTransform(metadata['position'],
                         metadata['quaternion'],
                         timestamp,
                         self.body_frame, self.world_frame) # Convention TFs

    def image_cb(self, event):
        """ Publish images from simulator to ROS """
        data_request = DataRequest(False, self.cameras)
        data_response = self.env.request(data_request)

        for i in range(len(self.cameras)):
            if self.cameras[i][2] == Channels.SINGLE:
                img_msg = self.bridge.cv2_to_imgmsg(data_response.images[i], 'mono8')
            elif self.cameras[i][2] == Channels.THREE:
                img_msg = self.bridge.cv2_to_imgmsg(data_response.images[i], 'bgr8')
            img_msg.header.stamp = rospy.Time.now()
            self.image_publishers[i].publish(img_msg)

    def clock_cb(self, event):
        """ Publish simulated clock time """
        metadata = self.get_metadata()

        sim_time = rospy.Time.from_sec(metadata['time'] / self.speedup_factor)
        self.clock_pub.publish(sim_time)

    def publish_camera_info(self, timestamp):
        """ Publish CameraInfo messages one time for left_cam and right_cam
            TODO: need to get baseline from camera metadata (relative poses)
        """
        left_cam_data = self.env.request(CameraInformationRequest(Camera.RGB_LEFT))
        parsed_left_cam_data = self.parse_cam_data(left_cam_data.data)
        right_cam_data = self.env.request(CameraInformationRequest(Camera.RGB_RIGHT))
        parsed_right_cam_data = self.parse_cam_data(right_cam_data.data)

        cam_info_left = rospy.Publisher("tesse/left_cam/camera_info", CameraInfo, queue_size=10)
        cam_info_right = rospy.Publisher("tesse/right_cam/camera_info", CameraInfo, queue_size=10)

        f = (self.camera_height / 2.0) / np.tan((np.pi*(self.camera_fov / 180.0)) / 2.0)
        fx = f
        fy = f
        cx = self.camera_width / 2 # pixels
        cy = self.camera_height / 2 # pixels
        baseline = np.abs(parsed_left_cam_data['position'][0] - parsed_right_cam_data['position'][0])
        print "baseline:", baseline
        Tx = 0
        Tx_right = -f * baseline
        Ty = 0

        cam_info_msg_left = self.make_camera_msg("left_cam", self.camera_width, self.camera_height, fx, fy, cx, cy, Tx, Ty)
        cam_info_msg_right = self.make_camera_msg("right_cam", self.camera_width, self.camera_height, fx, fy, cx, cy, Tx_right, Ty)
        cam_info_msg_left.header.stamp = timestamp
        cam_info_msg_right.header.stamp = timestamp

        cam_info_left.publish(cam_info_msg_left)
        cam_info_right.publish(cam_info_msg_right)

    def get_metadata(self):
        """ Return camera meta data object """
        metadata_request = MetadataRequest()
        metadata_response = self.env.request(metadata_request)

        return self.parse_metadata(metadata_response.data)

    # utilities (should go in utils.py file in package):
    ############################################################################

    def parse_metadata(self, data):
        """ Parse metadata into a useful dictionary """

        # tree = BeautifulSoup(data, 'xml')
        # print "tree:", tree
        # TODO: fix ill-formed metadata response so that it can be parsed better

        dict = {}
        data_string = str(data)
        split_data = data_string.split()

        position = [float(split_data[5][3:-1]), float(split_data[6][3:-1]), float(split_data[7][3:-3])]
        quaternion = [float(split_data[9][3:-1]), float(split_data[10][3:-1]), float(split_data[11][3:-1]), float(split_data[12][3:-3])]
        velocity = [float(split_data[14][7:-1]), float(split_data[15][7:-1]), float(split_data[16][7:-3])]
        ang_vel = [float(split_data[19][11:-1]), float(split_data[20][11:-1]), float(split_data[22][2:-3])]
        acceleration = [float(split_data[24][8:-1]), float(split_data[25][8:-1]), float(split_data[26][8:-3])]
        ang_acceleration = [float(split_data[29][12:-1]), float(split_data[30][12:-1]), float(split_data[31][12:-3])]
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

    def parse_cam_data(self, data):
        """ Parse CameraInformationRequest data into a useful dictionary """
        tree = BeautifulSoup(data, 'xml')

        dict = {}
        dict['id'] = int(tree.camera_info.id.string)
        #TODO: solve ill-formed parameter property so you can store it
        dict['position'] = [float(tree.camera_info.position['x']),
                            float(tree.camera_info.position['y']),
                            float(tree.camera_info.position['z'])]
        dict['quaternion'] = [float(tree.camera_info.rotation['x']),
                              float(tree.camera_info.rotation['y']),
                              float(tree.camera_info.rotation['z']),
                              float(tree.camera_info.rotation['w'])]

        return dict

    def make_camera_msg(self, frame_id, width, height, fx, fy, cx, cy, Tx, Ty):
        camera_info_msg = CameraInfo()
        camera_info_msg.header.frame_id = frame_id
        camera_info_msg.width = width
        camera_info_msg.height = height
        camera_info_msg.K = [fx, 0, cx,
                            0, fy, cy,
                            0, 0, 1]
        # No rectification
        camera_info_msg.R = [1, 0, 0,
                           0, 1, 0,
                           0, 0, 1]
        # No distortion
        camera_info_msg.distortion_model = "plumb_bob"
        camera_info_msg.D = [0, 0, 0, 0]

        # Ty = 0 and Tx = -fx' * B, where B is the baseline between the cameras.
        camera_info_msg.P = [fx, 0, cx, Tx,
                            0, fy, cy, Ty,
                            0, 0, 1, 0]
        return camera_info_msg

    def metadata_to_odom(self, timestamp, metadata):
        """ Transforms a metadata message to a ROS odometry message.
        Requires a transform from body to map. (TODO remove hardcoded frame_id)
        """
        header = Header()
        header.stamp = timestamp
        odom = Odometry()
        odom.header = header
        odom.header.frame_id = self.world_frame

        odom.pose.pose.position.x =  metadata['position'][0]
        odom.pose.pose.position.y =  metadata['position'][1]
        odom.pose.pose.position.z =  metadata['position'][2]

        odom.pose.pose.orientation.x = metadata['quaternion'][0]
        odom.pose.pose.orientation.y = metadata['quaternion'][1]
        odom.pose.pose.orientation.z = metadata['quaternion'][2]
        odom.pose.pose.orientation.w = metadata['quaternion'][3]

        # Not needed for ground_truth_odometry for now.
        odom.twist.twist.linear.x = metadata['velocity'][0]
        odom.twist.twist.linear.y = metadata['velocity'][1]
        odom.twist.twist.linear.z = metadata['velocity'][2]

        odom.twist.twist.angular.x = metadata['ang_vel'][0]
        odom.twist.twist.angular.y = metadata['ang_vel'][1]
        odom.twist.twist.angular.z = metadata['ang_vel'][2]

        return odom

    def metadata_to_imu(self, timestamp, metadata):
        """ Transforms a metadata message to a ROS imu message.
        Requires a transform from body to map. (TODO is this actually body frame?)
        """
        imu = Imu()
        imu.header.stamp = timestamp
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

        return imu

    ############################################################################

if __name__ == '__main__':
    rospy.init_node("TesseROSWrapper_node")
    node = TesseROSWrapper()
    rospy.spin()
