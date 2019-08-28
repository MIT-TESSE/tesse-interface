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
from geometry_msgs.msg import Pose, PoseStamped, Point, \
     PointStamped, TransformStamped, Twist
from rosgraph_msgs.msg import Clock
from cv_bridge import CvBridge, CvBridgeError

import tesse_ros_bridge.utils

from tesse_ros_bridge.srv import SceneRequestService

from tesse.msgs import *
from tesse.env import *
from tesse.utils import *

class TesseROSWrapper:

    def __init__(self):
        # Interface parameters:
        self.teleop_enabled = rospy.get_param("~teleop", False)
        self.step_mode_enabled = rospy.get_param("~enable_step_mode", False)

        # Networking parameters:
        self.client_ip = rospy.get_param("~client_ip", "127.0.0.1")
        self.self_ip = rospy.get_param("~self_ip", "127.0.0.1")
        self.position_port = rospy.get_param("~position_port", 9000)
        self.metadata_port = rospy.get_param("~metadata_port", 9001)
        self.image_port = rospy.get_param("~image_port", 9002)
        self.udp_port = rospy.get_param("~udp_port", 9004)
        self.step_port = rospy.get_param("~step_port", 9005)

        # Camera parameters:
        self.camera_width = rospy.get_param("~camera_width", 720)
        self.camera_height = rospy.get_param("~camera_height", 480)
        self.camera_fov = rospy.get_param("~camera_fov", 60)
        self.stereo_baseline = rospy.get_param("~stereo_baseline", 0.2)

        # Simulator speed parameters:
        self.use_sim = rospy.get_param("/use_sim_time", False)
        self.speedup_factor = rospy.get_param("~speedup_factor", 1)
        self.frame_rate = rospy.get_param("~frame_rate", 20.0)
        self.imu_rate = rospy.get_param("~imu_rate", 200.0)

        # Output parameters:
        self.world_frame_id = rospy.get_param("~world_frame_id", "world")
        self.body_frame_id = rospy.get_param("~body_frame_id", "base_link")

        self.env = Env(simulation_ip=self.client_ip,
                        own_ip=self.self_ip,
                        position_port=self.position_port,
                        metadata_port=self.metadata_port,
                        image_port=self.image_port,
                        step_port=self.step_port)

        self.cv_bridge = CvBridge()

        # TODO we also need to know frame_id, hardcoding for now, again
        # use a more descriptive data structure.
        self.cam_frame_id = ["left_cam", "right_cam", "left_cam", "left_cam"]
        self.cameras=[(Camera.RGB_LEFT, Compression.OFF, Channels.SINGLE),
                      (Camera.RGB_RIGHT, Compression.OFF, Channels.SINGLE),
                      (Camera.SEGMENTATION, Compression.OFF, Channels.THREE),
                      (Camera.DEPTH, Compression.OFF, Channels.THREE)]

        self.img_pubs = [rospy.Publisher("left_cam", Image, queue_size=1),
                         rospy.Publisher("right_cam", Image, queue_size=1),
                         rospy.Publisher("segmentation", Image, queue_size=1),
                         rospy.Publisher("depth", Image, queue_size=1)]

        self.far_draw_dist = None

        # Camera information members.
        self.cam_info_left_pub = rospy.Publisher("left_cam/camera_info",
            CameraInfo, queue_size=10)
        self.cam_info_right_pub = rospy.Publisher("right_cam/camera_info",
            CameraInfo, queue_size=10)
        self.cam_info_msg_left = None
        self.cam_info_msg_right = None

        self.imu_pub = rospy.Publisher("imu", Imu, queue_size=1)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)

        self.scene_request_service = rospy.Service("scene_change_request",
                                                    SceneRequestService,
                                                    self.change_scene)

        self.br = tf.TransformBroadcaster()
        self.static_br = tf2_ros.StaticTransformBroadcaster()

        # Required states for finite difference calculations.
        self.prev_time = 0.0
        self.prev_vel_brh = [0.0, 0.0, 0.0]

        # Setup simulator step mode.

        # Setup camera parameters and extrinsics in the simulator per spec.
        self.setup_cameras()

        # Setup UdpListener.
        self.udp_listener = UdpListener(port=self.udp_port, rate=self.imu_rate)
        self.udp_listener.subscribe('udp_subcriber', self.udp_cb)

        # Simulated time requires that we constantly publish to '/clock'.
        self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=1)

        # Setup simulator step mode with teleop.
        if self.step_mode_enabled:
            self.env.send(SetFrameRate(self.frame_rate))

        #Setup teleop command.
        if self.teleop_enabled:
            rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb)

        print "TESSE_ROS_NODE: Initialization complete."

        self.last_cmd = Twist()

    def spin(self):
        """ Start timers and callbacks.

            Because we are publishing sim time, we
            cannot simply call `rospy.spin()` as this will wait for messages
            to go to /clock first, and will freeze the node.
        """
        rospy.Timer(rospy.Duration(1.0/self.frame_rate), self.image_cb)
        self.udp_listener.start()

        # rospy.spin()

        while not rospy.is_shutdown():
            self.clock_cb(None)

    def udp_cb(self, data):
        """ Callback for UDP metadata at high rates.

            Parses raw metadata from the simulator, processes it into the
            proper reference frame, and publishes it as odometry, imu and
            transform information to ROS.

            Args:
                data: A string or bytestring in xml format containing the
                    metadata from the simulator.
        """
        # Parse metadata and process for proper use.
        metadata = tesse_ros_bridge.utils.parse_metadata(data)
        metadata_processed = tesse_ros_bridge.utils.process_metadata(metadata,
            self.prev_time, self.prev_vel_brh)
        self.prev_time = metadata_processed['time']
        self.prev_vel_brh = metadata_processed['velocity']

        timestamp = rospy.Time.from_sec(
            metadata_processed['time'] / self.speedup_factor)

        # Publish simulated time.
        # TODO(marcus): decide who should publish timestamps
        # self.clock_pub.publish(timestamp)

        # Publish imu and odometry messages.
        imu = tesse_ros_bridge.utils.metadata_to_imu(metadata_processed,
            timestamp, self.body_frame_id)
        self.imu_pub.publish(imu)
        odom = tesse_ros_bridge.utils.metadata_to_odom(metadata_processed,
            timestamp, self.world_frame_id, self.body_frame_id)
        self.odom_pub.publish(odom)

        # Publish agent ground truth transform.
        self.br.sendTransform(metadata_processed['position'],
                              metadata_processed['quaternion'],
                              timestamp,
                              self.body_frame_id, self.world_frame_id)  # TODO: switch?

    def image_cb(self, event):
        """ Publish images from simulator to ROS.

            Left and right images are published in the mono8 encoding.
            Depth images are pre-processed s.t. pixel values directly give
            point depth, in meters.
            Segmentation images are published in the rgb8 encoding.

            Args:
                event: A rospy.Timer event object, which is not used in this
                    method. You may supply `None`.
        """
        try:
            # Get camera data.
            data_response = self.env.request(DataRequest(True, self.cameras))

            # Process metadata to publish transaform.
            metadata = tesse_ros_bridge.utils.parse_metadata(
                data_response.metadata)
            metadata_processed = tesse_ros_bridge.utils.process_metadata(
                metadata, self.prev_time, self.prev_vel_brh)
            # TODO: reenable in step mode
            # self.prev_time = metadata_processed['time']
            # self.prev_vel_brh = metadata_processed['velocity']

            timestamp = rospy.Time.from_sec(
                metadata_processed['time'] / self.speedup_factor)

            # self.clock_pub.publish(timestamp)

            # Process each image.
            for i in range(len(self.cameras)):
                if self.cameras[i][0] == Camera.DEPTH:
                    # TODO: Is this rescaling still required?
                    img_msg = self.cv_bridge.cv2_to_imgmsg(
                        data_response.images[i] * self.far_draw_dist,
                            'passthrough')
                elif self.cameras[i][2] == Channels.SINGLE:
                    img_msg = self.cv_bridge.cv2_to_imgmsg(
                        data_response.images[i], 'mono8')
                elif self.cameras[i][2] == Channels.THREE:
                    img_msg = self.cv_bridge.cv2_to_imgmsg(
                        data_response.images[i], 'rgb8')

                # Publish images to appropriate topic.
                img_msg.header.frame_id = self.cam_frame_id[i]
                img_msg.header.stamp = timestamp
                self.img_pubs[i].publish(img_msg)

            # Publish both CameraInfo messages.
            self.cam_info_msg_left.header.stamp = timestamp
            self.cam_info_msg_right.header.stamp = timestamp
            self.cam_info_left_pub.publish(self.cam_info_msg_left)
            self.cam_info_right_pub.publish(self.cam_info_msg_right)

            # TODO(marcus): re-enable in step mode
            # Publish current transform.
            # self.br.sendTransform(metadata_processed['position'],
            #                       metadata_processed['quaternion'],
            #                       timestamp,
            #                       self.body_frame_id, self.world_frame_id)
            odom = tesse_ros_bridge.utils.metadata_to_odom(metadata_processed,
                timestamp, self.world_frame_id, self.body_frame_id)
            self.odom_pub.publish(odom)

        except Exception as error:
                print "TESSE_ROS_NODE: image_cb error: ", error

    def clock_cb(self, event):
        """ Publishes simulated clock time.

            Gets current metadata from the simulator over the low-rate metadata
            port. Publishes the timestamp, optionally modified by the
            specified speedup_factor.

            Args:
                event: A rospy.Timer event object, which is not used in this
                    method. You may supply `None`.
        """
        # print "in clock_cb"
        # force_x = self.last_cmd.linear.x*10
        # force_y = self.last_cmd.linear.y*10
        # torque_z = self.last_cmd.angular.z*10
        # self.env.send(StepWithForce(force_z=force_x, torque_y=torque_z,
        #     force_x=force_y, duration=1))
        # print "send command"

        try:
            metadata = tesse_ros_bridge.utils.parse_metadata(self.env.request(
                MetadataRequest()).metadata)

            sim_time = rospy.Time.from_sec(
                metadata['time'] / self.speedup_factor)
            self.clock_pub.publish(sim_time)
        except Exception as error:
            print "TESSE_ROS_NODE: clock_cb error: ", error

    def cmd_cb(self, msg):
        """ Listens to teleop force commands and sends to simulator.

            Subscribed to the key_teleop node's output, the callback gets
            Twist messages for both linear and angular 'velocities'. These
            velocities are sent directly as forces and torques to the
            simulator. Actual values for the incoming velocities are
            determined in the launch file.

            Args:
                msg: A geometry_msgs/Twist message.
        """
        # force_x = msg.linear.x
        # force_y = msg.linear.y
        # torque_z = msg.angular.z
        # self.env.send(StepWithForce(force_z=force_x, torque_y=torque_z,
        #     force_x=force_y, duration=0))

        # self.image_cb(None)
        self.last_cmd = msg

    def setup_cameras(self):
        """ Initializes image-related members.

            Sends camera parameter, position and rotation data to the simulator
            to properly reset them as specified in the node arguments.
            Calculates and sends static transforms for the left and
            right cameras relative to the body frame.
            Also constructs the CameraInfo messages for left and right cameras,
            to be published with every frame.
        """
        # Set camera parameters once for the entire simulation.
        for camera in self.cameras:
            resp = None
            while resp is None:
                print "TESSE_ROS_NODE: Setting parameters of camera: ", \
                        camera[0], "..."
                resp = self.env.request(SetCameraParametersRequest(
                                                self.camera_height,
                                                self.camera_width,
                                                self.camera_fov,
                                                camera[0]))

        # TODO(marcus): add SetCameraOrientationRequest option.
        cam_left_position = Point(-self.stereo_baseline / 2, 0.0, 0.0)
        cam_right_position = Point(self.stereo_baseline / 2, 0.0, 0.0)

        resp = None
        while resp is None:
            print "TESSE_ROS_NODE: Setting position of left camera..."
            resp = self.env.request(SetCameraPositionRequest(
                    cam_left_position.x,
                    cam_left_position.y,
                    cam_left_position.z,
                    Camera.RGB_LEFT))

        resp = None
        while resp is None:
            print "TESSE_ROS_NODE: Setting position of right camera..."
            resp = self.env.request(SetCameraPositionRequest(
                    cam_right_position.x,
                    cam_right_position.y,
                    cam_right_position.z,
                    Camera.RGB_RIGHT))

        # Set position depth and segmentation cameras to align with left:
        resp = None
        while resp is None:
            print "TESSE_ROS_NODE: Setting position of depth camera..."
            resp = self.env.request(SetCameraPositionRequest(
                    cam_left_position.x,
                    cam_left_position.y,
                    cam_left_position.z,
                    Camera.DEPTH))
        resp = None
        while resp is None:
            print "TESSE_ROS_NODE: Setting position of segmentation camera..."
            resp = self.env.request(SetCameraPositionRequest(
                    cam_left_position.x,
                    cam_left_position.y,
                    cam_left_position.z,
                    Camera.SEGMENTATION))

        # Depth camera multiplier factor.
        depth_cam_data = None
        while depth_cam_data is None:
            depth_cam_data = self.env.request(
                CameraInformationRequest(Camera.DEPTH))
        parsed_depth_cam_data = tesse_ros_bridge.utils.parse_cam_data(
            depth_cam_data.metadata)
        self.far_draw_dist = parsed_depth_cam_data['draw_distance']['far']

        # Left cam static tf.
        static_tf_cam_left = TransformStamped()
        static_tf_cam_left.header.frame_id = self.body_frame_id
        static_tf_cam_left.transform.translation = cam_left_position
        static_tf_cam_left.transform.rotation.x = 0
        static_tf_cam_left.transform.rotation.y = 0
        static_tf_cam_left.transform.rotation.z = 0
        static_tf_cam_left.transform.rotation.w = 1.0
        static_tf_cam_left.child_frame_id = self.cam_frame_id[0]

        # Right cam static tf.
        static_tf_cam_right = copy.deepcopy(static_tf_cam_left)
        static_tf_cam_right.transform.translation = cam_right_position
        static_tf_cam_right.child_frame_id = self.cam_frame_id[1]

        self.static_br.sendTransform(static_tf_cam_left)
        self.static_br.sendTransform(static_tf_cam_right)

        # Camera_info publishing for VIO.
        left_cam_data = None
        right_cam_data = None
        while left_cam_data is None and right_cam_data is None:
            print "TESSE_ROS_NODE: Acquiring camera data..."
            left_cam_data = tesse_ros_bridge.utils.parse_cam_data(
                self.env.request(
                    CameraInformationRequest(Camera.RGB_LEFT)).metadata)
            right_cam_data = tesse_ros_bridge.utils.parse_cam_data(
                self.env.request(
                    CameraInformationRequest(Camera.RGB_RIGHT)).metadata)

        self.cam_info_msg_left, self.cam_info_msg_right = \
            tesse_ros_bridge.utils.generate_camera_info(
                left_cam_data, right_cam_data)

    def change_scene(self, req):
        """ Change scene ID of simulator as a ROS service. """
        # TODO(marcus): make this more elegant, like a None chek
        try:
            result = self.env.request(SceneRequest(req.id))
            return True
        except:
            return False


if __name__ == '__main__':
    rospy.init_node("TesseROSWrapper_node")
    node = TesseROSWrapper()
    # import cProfile
    # cProfile.run('TesseROSWrapper()',
    #              '/home/marcus/TESS/ros_bridge_profile_3.cprof')

    node.spin()
