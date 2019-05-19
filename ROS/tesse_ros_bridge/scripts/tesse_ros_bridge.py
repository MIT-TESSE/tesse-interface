#!/usr/bin/env python

import numpy as np
import cv2

import rospy
import tf
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError

import tesse_ros_bridge

from tesse.msgs import *
from tesse.env import *

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

def metadata_to_odom(metadata):
  """ Transforms a metadata message to a ROS odometry message.
  Requires a transform from body to map. (TODO remove hardcoded frame_id)
  """
  header = Header()
  header.stamp = rospy.Time(metadata['time'])
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

class Params():
  def __init__(self):
    print("Parsing Params")

  def parseParams(self):
    # For connecting to Unity
    self.client_ip    = rospy.get_param('~client_ip', '127.0.0.1')
    self.self_ip      = rospy.get_param('~self_ip', '127.0.0.1')
    self.request_port = rospy.get_param('~request_port', '9000')
    self.receive_port = rospy.get_param('~receive_port', '9001')

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

  def publish(self, img_stamp, cv_images):
    # cv_images must be in the same order as the publishers!
    # TODO use a dictionary for Camera, instead of enum, in msgs.py
    # to avoid this potential issue
    for i in range(len(cv_images)):
      try:
        # Transform cv2 image to ROS image, using appropriate encoding.
        cam_channels = self.cameras[i][2]
        if cam_channels == Channels.SINGLE:
          img_msg = self.bridge.cv2_to_imgmsg(cv_images[i], "mono8")
        elif cam_channels == Channels.THREE:
          img_msg = self.bridge.cv2_to_imgmsg(cv_images[i], "rgb8")
        else:
          rospy.logerr('Wrong number of channels for camera: %i' % i)

        img_msg.header.stamp = rospy.Time(img_stamp)

        # Publish each image using the corresponding publisher.
        if i < self.len_publishers:
          self.publishers[i].publish(img_msg)
        else:
          rospy.logwarn('There are more images than ROS publishers!')

      except CvBridgeError as e:
        print(e)

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

    # Create Pose publisher
    gt_odom_pub = rospy.Publisher("ground_truth_odometry", Odometry, queue_size = 10)

    # Create Tf publisher
    br = tf.TransformBroadcaster()

    # Create data request packet
    data_request = DataRequest(True, cameras);

    rate = rospy.Rate(30) # In Hz
    while not rospy.is_shutdown():
      # GO AS FAST AS POSSIBLE HERE

      # Query images from Unity
      data_response = env.request(data_request)
      metadata = parse_metadata(data_response.data)

      # Publish images to ROS
      image_pub.publish(metadata['time'], data_response.images)

      # TODO publish cam info as well

      # Publish pose (Not necessary)
      # gt_odom_pub.publish(metadata_to_odom(metadata))

      # Publish tf
      br.sendTransform(metadata['position'],
                       metadata['quaternion'],
                       rospy.Time(metadata['time']),
                       "base_link", "world") # Convention TFs

      # Wait to keep desired frames per second.
      rate.sleep()

if __name__ == '__main__':
    try:
        tesse_ros_bridge()
    except rospy.ROSInterruptException:
        pass
