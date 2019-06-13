#!/usr/bin/env python

import numpy as np
import copy

import cv2

import rospy
import tf
import tf2_ros
from std_msgs.msg import Header
from sensor_msgs.msg import Image, Imu, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped, TransformStamped
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
        self.stereo_baseline = rospy.get_param('~stereo_baseline', 0.2)
        self.scene_id = rospy.get_param('~scene_id', '0')
        self.use_sim = rospy.get_param('/use_sim_time', False)
        self.speedup_factor = rospy.get_param('~speedup_factor', 20)

        self.env = Env(simulation_ip=self.client_ip, own_ip=self.self_ip,
                    position_port=self.position_port,
                    metadata_port=self.metadata_port,
                    image_port=self.image_port)

        self.body_frame = 'base_link'
        self.world_frame = 'world'
        #TODO: change this to the correct vector once coordinate frame is fixed:
        self.gravity_vector = [0.0, -9.81, 0.0] # in 'world' frame

        self.bridge = CvBridge()

        self.imu_rate = 100
        self.image_rate = 20

        # TODO use a dictionary for Camera, instead of enum, in msgs.py
        # to avoid this potential issue
        # TODO we also need to know frame_id, hardcoding for now, again
        # use a more descriptive data structure.
        self.cam_frame_id = ["left_cam", "right_cam", "left_cam", "left_cam"]
        self.cameras=[(Camera.RGB_LEFT, Compression.OFF, Channels.THREE),
                 (Camera.RGB_RIGHT, Compression.OFF, Channels.THREE),
                 (Camera.SEGMENTATION, Compression.OFF, Channels.THREE),
                 (Camera.DEPTH, Compression.OFF, Channels.SINGLE)]

        self.left_image_pub = rospy.Publisher("left_cam", Image, queue_size=1)
        self.right_image_pub = rospy.Publisher("right_cam", Image, queue_size=1)
        self.segmented_image_pub = rospy.Publisher("segmentation", Image, queue_size=1)
        self.cam_info_left_pub = rospy.Publisher("left_cam/camera_info", CameraInfo, queue_size=10)
        self.cam_info_right_pub = rospy.Publisher("right_cam/camera_info", CameraInfo, queue_size=10)
        self.depth_image_pub = rospy.Publisher("depth", Image, queue_size=1)
        self.image_publishers = [self.left_image_pub, self.right_image_pub,
                                 self.segmented_image_pub, self.depth_image_pub]

        self.imu_pub = rospy.Publisher("imu", Imu, queue_size=1)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)

        self.br = tf.TransformBroadcaster()
        self.static_br = tf2_ros.StaticTransformBroadcaster()
        self.transformer = tf.TransformerROS()

        # Set camera parameters once for the entire simulation:
        for camera in self.cameras:
            result = self.env.request(SetCameraParametersRequest(
                                            self.camera_height,
                                            self.camera_width,
                                            self.camera_fov,
                                            camera[0]))
        self.cam_left_position = Point(-self.stereo_baseline / 2, 0.0, 0.0)
        self.cam_right_position = Point(self.stereo_baseline / 2, 0.0, 0.0)
        self.env.request(SetCameraPositionRequest(self.cam_left_position.x,
                                                  self.cam_left_position.y,
                                                  self.cam_left_position.z,
                                                  Camera.RGB_LEFT))
        self.env.request(SetCameraPositionRequest(self.cam_right_position.x,
                                                  self.cam_right_position.y,
                                                  self.cam_right_position.z,
                                                  Camera.RGB_RIGHT))

        # Left cam static tf
        self.static_tf_cam_left = TransformStamped()
        self.static_tf_cam_left.header.frame_id = self.body_frame
        self.static_tf_cam_left.transform.translation = self.cam_left_position
        self.static_tf_cam_left.transform.rotation.x = 0
        self.static_tf_cam_left.transform.rotation.y = 0
        self.static_tf_cam_left.transform.rotation.z = 0
        self.static_tf_cam_left.transform.rotation.w = 1.0
        self.static_tf_cam_left.child_frame_id = self.cam_frame_id[0]

        # Right cam static tf
        self.static_tf_cam_right = copy.deepcopy(self.static_tf_cam_left)
        self.static_tf_cam_right.transform.translation = self.cam_right_position
        self.static_tf_cam_right.child_frame_id = self.cam_frame_id[1]

        # Send scene request only if we don't want the default scene:
        if self.scene_id != 0:
            result = self.env.request(SceneRequest(self.scene_id))

        # Camera_info publishing for VIO
        self.cam_info_msg_left = CameraInfo()
        self.cam_info_msg_right = CameraInfo()
        self.generate_camera_info()


        self.imu_counter = 0 # only needed for everything_cb

        # self.everything_timer = rospy.Timer(rospy.Duration(1.0/1000.0),
        #                                         self.everything_cb)
        # self.imu_timer = rospy.Timer(rospy.Duration(1.0/self.imu_rate),
        #                                         self.imu_cb)
        # self.image_timer = rospy.Timer(rospy.Duration(1.0/self.image_rate),
        #                                         self.image_cb)

        if self.use_sim:
            self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=1)
            # self.clock_timer = rospy.Timer(rospy.Duration(1.0/1000.0),
            # self.clock_cb)

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

        # TODO (Marcus): this must make sense with what the simulator outputs
        # We must keep the same time spacing between these msgs and the simulator's data.
        # What happens if sim_time?
        timestamp = rospy.Time.from_sec(metadata['time']/ self.speedup_factor)

        imu = self.metadata_to_imu(timestamp, metadata)
        self.imu_pub.publish(imu)

        odom = self.metadata_to_odom(timestamp, metadata)
        self.odom_pub.publish(odom)

        self.br.sendTransform(metadata['position'],
                         metadata['quaternion'],
                         timestamp,
                         self.body_frame, self.world_frame) # Convention TFs

        self.static_tf_cam_left.header.stamp = timestamp
        self.static_tf_cam_right.header.stamp = timestamp
        self.static_br.sendTransform(self.static_tf_cam_left)
        self.static_br.sendTransform(self.static_tf_cam_right)

    def image_cb(self, event):
        """ Publish images from simulator to ROS """
        data_request = DataRequest(True, self.cameras)
        data_response = self.env.request(data_request)

        # TODO (Marcus): this must make sense with what the simulator outputs
        # We must keep the same time spacing between these msgs and the simulator's data.
        # What happens if sim_time?
        cameras_timestamp = rospy.Time.from_sec(self.parse_metadata(data_response.data)['time']/ self.speedup_factor)
        print(data_response.data)

        for i in range(len(self.cameras)):
            if self.cameras[i][2] == Channels.SINGLE:
                img_msg = self.bridge.cv2_to_imgmsg(data_response.images[i], 'mono8')
            elif self.cameras[i][2] == Channels.THREE:
                img_msg = self.bridge.cv2_to_imgmsg(data_response.images[i], 'rgb8')

            img_msg.header.frame_id = self.cam_frame_id[i]
            img_msg.header.stamp = cameras_timestamp
            self.image_publishers[i].publish(img_msg)

        self.cam_info_msg_left.header.stamp = cameras_timestamp
        self.cam_info_msg_right.header.stamp = cameras_timestamp
        self.cam_info_left_pub.publish(self.cam_info_msg_left)
        self.cam_info_right_pub.publish(self.cam_info_msg_right)

    def clock_cb(self, event):
        """ Publish simulated clock time """
        metadata = self.get_metadata()

        sim_time = rospy.Time.from_sec(metadata['time'] / self.speedup_factor)
        self.clock_pub.publish(sim_time)

    def generate_camera_info(self):
        """ Generate CameraInfo messages for left_cam and right_cam
            TODO: need to get baseline from camera metadata (relative poses)
        """
        left_cam_data = self.env.request(
                                    CameraInformationRequest(Camera.RGB_LEFT))
        right_cam_data = self.env.request(
                                    CameraInformationRequest(Camera.RGB_RIGHT))
        parsed_left_cam_data = self.parse_cam_data(left_cam_data.data)
        parsed_right_cam_data = self.parse_cam_data(right_cam_data.data)

        f = (self.camera_height / 2.0) / np.tan((np.pi*(self.camera_fov / 180.0)) / 2.0)
        fx = f
        fy = f
        cx = self.camera_width / 2 # pixels
        cy = self.camera_height / 2 # pixels
        baseline = np.abs(parsed_left_cam_data['position'][0] -
                          parsed_right_cam_data['position'][0])
        assert(baseline == self.stereo_baseline)
        Tx = 0
        Tx_right = -f * baseline
        Ty = 0

        cam_info_msg_left = self.make_camera_msg("left_cam",
                                                   self.camera_width,
                                                   self.camera_height,
                                                   fx, fy, cx, cy, Tx, Ty)
        cam_info_msg_right = self.make_camera_msg("right_cam",
                                                   self.camera_width,
                                                   self.camera_height,
                                                   fx, fy, cx, cy, Tx_right, Ty)

        self.cam_info_msg_left = cam_info_msg_left
        self.cam_info_msg_right = cam_info_msg_right

    def get_metadata(self):
        """ Return camera meta data object """
        metadata_request = MetadataRequest()
        metadata_response = self.env.request(metadata_request)

        return self.parse_metadata(metadata_response.data)

    # TODO utilities (should go in utils.py file in package):
    ############################################################################

    def parse_metadata(self, data):
        """ Parse metadata into a useful dictionary """
        # TODO: fix ill-formed metadata response so that it can be parsed better

        dict = {}
        data_string = str(data)
        split_data = data_string.split()

        position = [float(split_data[5][3:-1]), float(split_data[6][3:-1]),
                        float(split_data[7][3:-3])]
        quaternion = [float(split_data[9][3:-1]), float(split_data[10][3:-1]),
                        float(split_data[11][3:-1]),
                        float(split_data[12][3:-3])]
        velocity = [float(split_data[14][7:-1]), float(split_data[15][7:-1]),
                        float(split_data[16][7:-3])]
        ang_vel = [float(split_data[19][11:-1]), float(split_data[20][11:-1]),
                        float(split_data[22][2:-3])]
        acceleration = [float(split_data[24][8:-1]), float(split_data[25][8:-1]),
                        float(split_data[26][8:-3])]
        ang_acceleration = [float(split_data[29][12:-1]),
                            float(split_data[30][12:-1]),
                            float(split_data[31][12:-3])]
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

        imu.angular_velocity.x = metadata['ang_vel'][0]
        imu.angular_velocity.y = metadata['ang_vel'][1]
        imu.angular_velocity.z = metadata['ang_vel'][2]

        quat = np.array([metadata['quaternion'][0],
                         metadata['quaternion'][1],
                         metadata['quaternion'][2],
                         metadata['quaternion'][3]]) # x,y,z,w

        gravity_quat = np.array(self.gravity_vector + [0.0])
        gravity_quat_bf = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(quat, gravity_quat),
            tf.transformations.quaternion_conjugate(quat)
        )
        gravity_bf = gravity_quat_bf[:3]

        imu.linear_acceleration.x = metadata['acceleration'][0] + gravity_bf[0]
        imu.linear_acceleration.y = metadata['acceleration'][1] + gravity_bf[1]
        imu.linear_acceleration.z = metadata['acceleration'][2] + gravity_bf[2]

        return imu

    ############################################################################

if __name__ == '__main__':
    rospy.init_node("TesseROSWrapper_node")
    node = TesseROSWrapper()
    rospy.spin()
