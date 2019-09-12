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
     PointStamped, TransformStamped, Twist, Quaternion
from rosgraph_msgs.msg import Clock
from cv_bridge import CvBridge, CvBridgeError

import tesse_ros_bridge.utils

from tesse_ros_bridge.srv import SceneRequestService
from tesse_ros_bridge import brh_T_blh

from tesse.msgs import *
from tesse.env import *
from tesse.utils import *

class TesseROSWrapper:

    def __init__(self):
        # Interface parameters:
        self.teleop_enabled = rospy.get_param("~teleop", False)
        self.step_mode_enabled = rospy.get_param("~enable_step_mode", False)

        # Networking parameters:
        self.client_ip     = rospy.get_param("~client_ip", "127.0.0.1")
        self.self_ip       = rospy.get_param("~self_ip", "127.0.0.1")
        self.position_port = rospy.get_param("~position_port", 9000)
        self.metadata_port = rospy.get_param("~metadata_port", 9001)
        self.image_port    = rospy.get_param("~image_port", 9002)
        self.udp_port      = rospy.get_param("~udp_port", 9004)
        self.step_port     = rospy.get_param("~step_port", 9005)

        # Camera parameters:
        self.camera_width    = rospy.get_param("~camera_width", 720)
        assert(self.camera_width > 0)
        assert(self.camera_width % 2 == 0)
        self.camera_height   = rospy.get_param("~camera_height", 480)
        assert(self.camera_height > 0)
        assert(self.camera_height % 2 == 0)
        self.camera_fov      = rospy.get_param("~camera_vertical_fov", 60)
        assert(self.camera_fov > 0)
        self.stereo_baseline = rospy.get_param("~stereo_baseline", 0.2)
        assert(self.stereo_baseline > 0)

        # Simulator speed parameters:
        self.use_sim        = rospy.get_param("/use_sim_time", False)
        self.speedup_factor = rospy.get_param("~speedup_factor", 1.0)
        assert(self.speedup_factor > 0.0)  # We are  dividing by this so > 0
        self.frame_rate     = rospy.get_param("~frame_rate", 20.0)
        self.imu_rate       = rospy.get_param("~imu_rate", 200.0)

        # Output parameters:
        self.world_frame_id     = rospy.get_param("~world_frame_id", "world")
        self.body_frame_id      = rospy.get_param("~body_frame_id", "base_link_gt")
        self.left_cam_frame_id  = rospy.get_param("~left_cam_frame_id", "left_cam")
        self.right_cam_frame_id = rospy.get_param("~right_cam_frame_id", "right_cam")
        assert(self.left_cam_frame_id != self.right_cam_frame_id)

        self.env = Env(simulation_ip=self.client_ip,
                       own_ip=self.self_ip,
                       position_port=self.position_port,
                       metadata_port=self.metadata_port,
                       image_port=self.image_port,
                       step_port=self.step_port)

        # To send images via ROS network and convert from/to ROS
        self.cv_bridge = CvBridge()

        self.cameras=[(Camera.RGB_LEFT,     Compression.OFF, Channels.SINGLE, self.left_cam_frame_id),
                      (Camera.RGB_RIGHT,    Compression.OFF, Channels.SINGLE, self.right_cam_frame_id),
                      (Camera.SEGMENTATION, Compression.OFF, Channels.THREE,  self.left_cam_frame_id),
                      (Camera.DEPTH,        Compression.OFF, Channels.THREE,  self.left_cam_frame_id)]

        self.img_pubs = [rospy.Publisher("left_cam",     Image, queue_size=10),
                         rospy.Publisher("right_cam",    Image, queue_size=10),
                         rospy.Publisher("segmentation", Image, queue_size=10),
                         rospy.Publisher("depth",        Image, queue_size=10)]

        # TODO(Marcus): document what is this?
        self.far_draw_dist = None

        # Camera information members.
        self.cam_info_left_pub = rospy.Publisher("left_cam/camera_info",   CameraInfo, queue_size=10)
        self.cam_info_right_pub = rospy.Publisher("right_cam/camera_info", CameraInfo, queue_size=10)
        self.cam_info_msg_left = None
        self.cam_info_msg_right = None

        # Setup ROS publishers
        self.imu_pub  = rospy.Publisher("imu", Imu, queue_size=10)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)

        # Setup ROS services.
        self.setup_ros_services()

        # Transform broadcasters.
        self.tf_broadcaster = tf.TransformBroadcaster()
        # Don't call static_tf_broadcaster.sendTransform multiple times.
        # Rather call it once with multiple static tfs! Check issue #40
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Required states for finite difference calculations.
        self.prev_time      = 0.0
        self.prev_vel_brh   = [0.0, 0.0, 0.0]
        self.prev_enu_R_brh = np.identity(3)

        # Setup camera parameters and extrinsics in the simulator per spec.
        self.setup_cameras()

        # Setup collision
        enable_collision = rospy.get_param("~enable_collision", 0)
        self.setup_collision(enable_collision)

        # Change scene
        initial_scene = rospy.get_param("~initial_scene", 1)
        self.change_scene(initial_scene)

        # Setup UdpListener.
        self.udp_listener = UdpListener(port=self.udp_port, rate=self.imu_rate)
        self.udp_listener.subscribe('udp_subscriber', self.udp_cb)

        # Simulated time requires that we constantly publish to '/clock'.
        self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=10)

        # Setup simulator step mode with teleop.
        if self.step_mode_enabled:
            self.env.send(SetFrameRate(self.frame_rate))

        # Setup teleop command.
        if self.teleop_enabled:
            rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb)

        print("TESSE_ROS_NODE: Initialization complete.")

        self.last_cmd = Twist()

    def spin(self):
        """ Start timers and callbacks.

            Because we are publishing sim time, we
            cannot simply call `rospy.spin()` as this will wait for messages
            to go to /clock first, and will freeze the node.
        """
        rospy.Timer(rospy.Duration(1.0 / self.frame_rate), self.image_cb)
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
            self.prev_time, self.prev_vel_brh, self.prev_enu_R_brh)

        assert(self.prev_time < metadata_processed['time'])
        self.prev_time      = metadata_processed['time']
        self.prev_vel_brh   = metadata_processed['velocity']
        self.prev_enu_R_brh = metadata_processed['transform'][:3,:3]

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
        self.publish_tf(metadata_processed['transform'], timestamp)

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

            # Process metadata to publish transform.
            metadata = tesse_ros_bridge.utils.parse_metadata(
                data_response.metadata)

            timestamp = rospy.Time.from_sec(
                metadata['time'] / self.speedup_factor)

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
                img_msg.header.frame_id = self.cameras[i][3]
                img_msg.header.stamp = timestamp
                self.img_pubs[i].publish(img_msg)

            # Publish both CameraInfo messages.
            self.cam_info_msg_left.header.stamp = timestamp
            self.cam_info_msg_right.header.stamp = timestamp
            self.cam_info_left_pub.publish(self.cam_info_msg_left)
            self.cam_info_right_pub.publish(self.cam_info_msg_right)

            self.publish_tf(
                tesse_ros_bridge.utils.convert_coordinate_frame(metadata),
                    timestamp)

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
        # Set all cameras to have same intrinsics:
        for camera in self.cameras:
            camera_id = camera[0]
            if camera_id is not Camera.THIRD_PERSON:
                resp = None
                while resp is None:
                    print("TESSE_ROS_NODE: Setting intrinsic parameters for camera: ",
                        camera_id)
                    resp = self.env.request(SetCameraParametersRequest(
                        self.camera_height,
                        self.camera_width,
                        self.camera_fov,
                        camera_id))

        # TODO(marcus): add SetCameraOrientationRequest option.
        # TODO(Toni): this is hardcoded!! what if don't want IMU in the middle?
        # Also how is this set using x? what if it is y, z?
        left_cam_position  = Point(x = -self.stereo_baseline / 2,
                                   y = 0.0,
                                   z = 0.0)
        right_cam_position = Point(x = self.stereo_baseline / 2,
                                   y = 0.0,
                                   z = 0.0)
        cameras_orientation = Quaternion(x=0.0,
                                         y=0.0,
                                         z=0.0,
                                         w=1.0)

        resp = None
        while resp is None:
            print "TESSE_ROS_NODE: Setting position of left camera..."
            resp = self.env.request(SetCameraPositionRequest(
                    left_cam_position.x,
                    left_cam_position.y,
                    left_cam_position.z,
                    Camera.RGB_LEFT))

        resp = None
        while resp is None:
            print "TESSE_ROS_NODE: Setting position of right camera..."
            resp = self.env.request(SetCameraPositionRequest(
                    right_cam_position.x,
                    right_cam_position.y,
                    right_cam_position.z,
                    Camera.RGB_RIGHT))

        # Set position depth and segmentation cameras to align with left:
        resp = None
        while resp is None:
            print "TESSE_ROS_NODE: Setting position of depth camera..."
            resp = self.env.request(SetCameraPositionRequest(
                    left_cam_position.x,
                    left_cam_position.y,
                    left_cam_position.z,
                    Camera.DEPTH))
        resp = None
        while resp is None:
            print "TESSE_ROS_NODE: Setting position of segmentation camera..."
            resp = self.env.request(SetCameraPositionRequest(
                    left_cam_position.x,
                    left_cam_position.y,
                    left_cam_position.z,
                    Camera.SEGMENTATION))

        for camera in self.cameras:
            camera_id = camera[0]
            if camera_id is not Camera.THIRD_PERSON:
                resp = None
                while resp is None:
                    print("TESSE_ROS_NODE: Setting orientation of all cameras to identity...")
                    resp = self.env.request(SetCameraOrientationRequest(
                            cameras_orientation.x,
                            cameras_orientation.y,
                            cameras_orientation.z,
                            cameras_orientation.w,
                            camera_id))

        # Depth camera multiplier factor.
        depth_cam_data = None
        while depth_cam_data is None:
            depth_cam_data = self.env.request(
                CameraInformationRequest(Camera.DEPTH))

        parsed_depth_cam_data = tesse_ros_bridge.utils.parse_cam_data(
            depth_cam_data.metadata)
        self.far_draw_dist = parsed_depth_cam_data['draw_distance']['far']

        # Left cam static tf.
        static_tf_cam_left                       = TransformStamped()
        static_tf_cam_left.header.frame_id       = self.body_frame_id
        static_tf_cam_left.header.stamp          = rospy.Time.now()
        static_tf_cam_left.transform.translation = left_cam_position
        static_tf_cam_left.transform.rotation    = cameras_orientation
        static_tf_cam_left.child_frame_id        = self.left_cam_frame_id

        # Right cam static tf.
        static_tf_cam_right                       = TransformStamped()
        static_tf_cam_right.header.frame_id       = self.body_frame_id
        static_tf_cam_right.header.stamp          = rospy.Time.now()
        static_tf_cam_right.transform.translation = right_cam_position
        static_tf_cam_right.transform.rotation    = cameras_orientation
        static_tf_cam_right.child_frame_id        = self.right_cam_frame_id

        # Send static tfs over the ROS network
        self.static_tf_broadcaster.sendTransform([static_tf_cam_right, static_tf_cam_left])

        # Camera_info publishing for VIO.
        left_cam_data = None
        while left_cam_data is None:
            print("TESSE_ROS_NODE: Acquiring left camera data...")
            left_cam_data = tesse_ros_bridge.utils.parse_cam_data(
                self.env.request(
                    CameraInformationRequest(Camera.RGB_LEFT)).metadata)
            assert(left_cam_data['id'] == 0)
            assert(left_cam_data['parameters']['height'] > 0)
            assert(left_cam_data['parameters']['width'] > 0)

        right_cam_data = None
        while right_cam_data is None:
            print("TESSE_ROS_NODE: Acquiring right camera data...")
            right_cam_data = tesse_ros_bridge.utils.parse_cam_data(
                self.env.request(
                    CameraInformationRequest(Camera.RGB_RIGHT)).metadata)
            assert(right_cam_data['id'] == 1)
            assert(left_cam_data['parameters']['height'] > 0)
            assert(left_cam_data['parameters']['width'] > 0)

        assert(left_cam_data['parameters']['height'] == self.camera_height)
        assert(left_cam_data['parameters']['width']  == self.camera_width)
        assert(right_cam_data['parameters']['height'] == self.camera_height)
        assert(right_cam_data['parameters']['width']  == self.camera_width)

        self.cam_info_msg_left, self.cam_info_msg_right = \
            tesse_ros_bridge.utils.generate_camera_info(
                left_cam_data, right_cam_data)

        # TODO(Toni): do a check here by requesting all camera info and checking that it is
        # as the one requested!
        # Ok so let's check that the

    def setup_ros_services(self):
        """ Setup ROS services related to the simulator.

            These services include:
                scene_change_request: change the scene_id of the simulator
        """
        self.scene_request_service = rospy.Service("scene_change_request",
                                                    SceneRequestService,
                                                    self.rosservice_change_scene)

    def setup_collision(self, enable_collision):
        """ Enable/Disable collisions in Simulator. """
        print("TESSE_ROS_NODE: Setup collisions to:", enable_collision)
        if enable_collision is True:
            self.env.send(ColliderRequest(enable=1))
        else:
            self.env.send(ColliderRequest(enable=0))

    def rosservice_change_scene(self, req):
        """ Change scene ID of simulator as a ROS service. """
        # TODO(marcus): make this more elegant, like a None chek
        try:
            result = self.env.request(SceneRequest(req.id))
            return True
        except:
            return False

    def change_scene(self, scene_id):
        """ Change scene ID of simulator. """
        return self.env.request(SceneRequest(scene_id))

    def publish_tf(self, cur_tf, timestamp):
        """ Publish the ground-truth transform to the TF tree.

            Args:
                cur_tf: A 4x4 numpy matrix containing the transformation from
                    the body frame of the agent to ENU.
                timestamp: A rospy.Time instance representing the current
                    time in the simulator.
        """
        # Publish current transform to tf tree.
        trans = tesse_ros_bridge.utils.get_translation_part(cur_tf)
        quat = tesse_ros_bridge.utils.get_quaternion(cur_tf)
        self.tf_broadcaster.sendTransform(trans, quat, timestamp,
                                          self.body_frame_id,
                                          self.world_frame_id)


if __name__ == '__main__':
    rospy.init_node("TesseROSWrapper_node")
    node = TesseROSWrapper()
    # import cProfile
    # cProfile.run('TesseROSWrapper()',
    #              '/home/marcus/TESS/ros_bridge_profile_3.cprof')

    node.spin()
