#!/usr/bin/env python

import math
import numpy as np
import cv2

import rospy
import tf
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from rosgraph_msgs.msg import Clock

import tesse_ros_bridge

from tesse.msgs import *
from tesse.env import *

#### The following should be in a utilities.py file ###########
def make_camera_msg(frame_id, width, height, fx, fy, cx, cy, Tx, Ty):
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

def parse_metadata(data):
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

def metadata_to_odom(timestamp, metadata):
  """ Transforms a metadata message to a ROS odometry message.
  Requires a transform from body to map. (TODO remove hardcoded frame_id)
  """
  header = Header()
  header.stamp = timestamp
  odom = Odometry()
  odom.header = header
  odom.header.frame_id = "world"

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
###############################################################

class Params():
  def __init__(self):
    pass

  def parseParams(self):
    # For connecting to Unity
    self.client_ip    = rospy.get_param('~client_ip', '127.0.0.1')
    self.self_ip      = rospy.get_param('~self_ip', '127.0.0.1')
    self.request_port = rospy.get_param('~request_port', '9000')
    self.receive_port = rospy.get_param('~receive_port', '9001')

    # Simulation time
    self.use_sim = rospy.get_param('/use_sim_time', False)
    self.speedup_factor = rospy.get_param('~speedup_factor', 1)

class ImagePublisher:
  def __init__(self, cameras):
    # Store params
    self.cameras = cameras

    # Setup ROS interface: create a ROS image publisher for each camera
    left_cam_pub = rospy.Publisher("left_cam", Image, queue_size=10)
    right_cam_pub = rospy.Publisher("right_cam", Image, queue_size=10)
    segmentation_pub = rospy.Publisher("segmentation", Image, queue_size=10)
    depth_pub = rospy.Publisher("depth", Image, queue_size=10)

    self.publishers = [left_cam_pub, right_cam_pub, segmentation_pub, depth_pub]
    self.len_publishers = len(self.publishers) # Cache number of publishers

    assert(len(self.cameras) == self.len_publishers)

    self.bridge = CvBridge()

  def publish(self, timestamp, cv_images):
    # cv_images must be in the same order as the publishers!
    # TODO use a dictionary for Camera, instead of enum, in msgs.py
    # to avoid this potential issue
    # TODO we also need to know frame_id, hardcoding for now, again
    # use a more descriptive data structure.
    cam_frame_id = ["left_cam", "right_cam", "left_cam", "left_cam"]
    for i in range(len(cv_images)):
      try:
        # Transform cv2 image to ROS image, using appropriate encoding.
        cam_channels = self.cameras[i][2]
        if cam_channels == Channels.SINGLE:
          img_msg = self.bridge.cv2_to_imgmsg(cv_images[i], "mono8")
        elif cam_channels == Channels.THREE:
          img_msg = self.bridge.cv2_to_imgmsg(cv_images[i], "bgr8")
        else:
          rospy.logerr('Wrong number of channels for camera: %i' % i)

        img_msg.header.stamp = timestamp
        img_msg.header.frame_id = cam_frame_id[i]

        # Publish each image using the corresponding publisher.
        if i < self.len_publishers:
          self.publishers[i].publish(img_msg)
        else:
          rospy.logwarn('There are more images than ROS publishers!')

      except CvBridgeError as e:
        print(e)

def clock_cb(event):
  #sim_time = rospy.Time.from_sec(time / speedup_factor)
  #clock_pub.publish(sim_time)
  pass

def tesse_ros_bridge():
    # Init ROS node, do this before parsing params.
    rospy.init_node('tesse_ros_bridge', anonymous=True)

    # Parse ROS params.
    params = Params()
    params.parseParams()

    # Setup TESSE/Unity bridge
    env = Env(params.client_ip, params.self_ip,
              params.request_port, params.receive_port)

    # Cameras in Unity to query.
    cameras=[(Camera.RGB_LEFT, Compression.OFF, Channels.SINGLE),
             (Camera.RGB_RIGHT, Compression.OFF, Channels.SINGLE),
             (Camera.SEGMENTATION, Compression.OFF, Channels.THREE),
             (Camera.DEPTH, Compression.OFF, Channels.SINGLE)]

    # Create ROS image publishers
    image_pub = ImagePublisher(cameras)

    # Create camera info publishers
    cam_info_left = rospy.Publisher("left_cam/camera_info", CameraInfo, queue_size = 10)
    cam_info_right = rospy.Publisher("right_cam/camera_info", CameraInfo, queue_size = 10)
    generate_cam_info_done = False # Flag to compute msg only once.
    width = 640 #pixels
    height = 480 #pixels
    fov = 37.84929
    f = (height / 2.0) / math.tan((math.pi * (fov / 180.0)) / 2.0);
    fx = f #pixels
    fy = f #pixels
    cx = width / 2 #pixels
    cy = height / 2 #pixels
    baseline = 0.1
    Tx = 0
    Tx_right = -f * baseline ; # -fx' * B
    Ty = 0

    cam_info_msg_left = make_camera_msg("left_cam", width, height, fx, fy, cx, cy, Tx, Ty)
    cam_info_msg_right = make_camera_msg("right_cam", width, height, fx, fy, cx, cy, Tx_right, Ty)

    # Create Pose publisher
    gt_odom_pub = rospy.Publisher("ground_truth_odometry", Odometry, queue_size = 10)

    # Setup simulation time if requested
    if params.use_sim:
        clock_pub = rospy.Publisher("/clock", Clock, queue_size=1)
        clock_timer = rospy.Timer(rospy.Duration(1.0/1000.0), clock_cb)

    # Create Tf publisher
    br = tf.TransformBroadcaster()

    # Create data request packet
    data_request = DataRequest(True, cameras);

    # TODO use Timers instead
    rate = rospy.Rate(30) # In Hz
    while not rospy.is_shutdown():
      # GO AS FAST AS POSSIBLE HERE

      # Query images from Unity
      data_response = env.request(data_request)
      metadata = parse_metadata(data_response.data)
      #timestamp = rospy.Time(metadata['time'])
      timestamp = rospy.Time.now()

      # Simulate time in ROS
      if params.use_sim:
        sim_time = rospy.Time.from_sec(metadata['time'] / params.speedup_factor)
        clock_pub.publish(sim_time)

      # Create cam info message. Do only once.
      #print(env.request(CameraInformationRequest()))
      #if generate_cam_info_done:
        #cam_info = env.request(CameraInformationRequest()))
        #cam_info_msg = make_camera_msg(parse_cam_info_metadata(cam_info))
      cam_info_msg_left.header.stamp = timestamp
      cam_info_msg_right.header.stamp = timestamp
      cam_info_left.publish(cam_info_msg_left)
      cam_info_right.publish(cam_info_msg_right)

      # Publish images to ROS
      image_pub.publish(timestamp, data_response.images)

      # TODO publish cam info as well

      # Publish pose (Not necessary)
      # gt_odom_pub.publish(metadata_to_odom(timestamp, metadata))

      # Publish tf
      br.sendTransform(metadata['position'],
                       metadata['quaternion'],
                       timestamp,
                       "base_link", "world") # Convention TFs

      # Wait to keep desired frames per second.
      rate.sleep()

if __name__ == '__main__':
    try:
        tesse_ros_bridge()
    except rospy.ROSInterruptException:
        pass
