#!/usr/bin/env python

import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from tesse_ros_bridge import *

from tesse.msgs import *
from tesse.env import *

class Params():
  def __init__(self):
    print("Parsing Params")

  def parseParams(self):
    # For connecting to Unity
    self.client_ip = rospy.get_param('~client_ip', '127.0.0.1')
    self.self_ip = rospy.get_param('~self_ip', '127.0.0.1')
    self.request_port = rospy.get_param('~request_port', '9000')
    self.receive_port = rospy.get_param('~receive_port', '9001')

class ImagePublisher:
  def __init__(self, cameras):
    # Store params
    self.cameras = cameras

    # Setup ROS interface: create a ROS image publisher for each camera
    rgb_left_pub = rospy.Publisher("rgb_left", Image, queue_size=10)
    rgb_right_pub = rospy.Publisher("rgb_right", Image, queue_size=10)
    segmentation_pub = rospy.Publisher("segmentation", Image, queue_size=10)
    depth_pub = rospy.Publisher("depth", Image, queue_size=10)

    self.publishers = [rgb_left_pub, rgb_right_pub, segmentation_pub, depth_pub]

    assert(len(self.cameras) == len(self.publishers))

    self.bridge = CvBridge()

  def publishToROS(self, img_stamp, cv_images):
    # cv_images must be in the same order as the publishers!
    # TODO use a dictionary for Camera, instead of enum, in msgs.py
    # to avoid this potential issue
    for i in range(len(cv_images)):
      try:
        img_msg = self.bridge.cv2_to_imgmsg(cv_images[i], "bgr8")
        img_msg.header.stamp = img_stamp
        if i < len(self.publishers):
          self.publishers[i].publish(img_msg)
        else:
          print('There are more images than ROS publishers.')
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
    cameras=[(Camera.RGB_LEFT, Compression.OFF, Channels.THREE),
             (Camera.RGB_RIGHT, Compression.OFF, Channels.SINGLE),
             (Camera.SEGMENTATION, Compression.OFF, Channels.THREE),
             (Camera.DEPTH, Compression.OFF, Channels.THREE)]

    # Use common header to all images
    image_publisher = ImagePublisher(cameras)

    # Create data request packet
    data_request = DataRequest(cameras);

    rate = rospy.Rate(60) # In Hz
    while not rospy.is_shutdown():
      hello_str = "hello world %s" % rospy.get_time()
      rospy.loginfo(hello_str)

      # Query images from Unity
      data_response = env.request(data_request)
      print(data_response.data)
      imgs = data_response.images
      print len(imgs)

      # Publish images to ROS
      image_publisher.publishToROS(rospy.Time.now(), imgs)

      # Wait to keep desired frames per second.
      rate.sleep()

if __name__ == '__main__':
    try:
        tesse_ros_bridge()
    except rospy.ROSInterruptException:
        pass
