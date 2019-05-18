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
  def __init__(self, params):
    # Store params
    self.params = params

    # Setup ROS interface
    self.image_pub = rospy.Publisher("image_out", Image, queue_size=10)
    self.bridge = CvBridge()

  def publishToROS(self, img_msg_header, cv_image):
    try:
      img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
      img_msg.header = img_msg_header
      self.image_pub.publish(img_msg)
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

    # Setup image publisher to ROS
    image_publisher = ImagePublisher(params)

    # Create data request packet
    data_request = DataRequest(cameras=[(Camera.SEGMENTATION, Compression.OFF, Channels.THREE),
                                        (Camera.RGB_LEFT, Compression.OFF, Channels.SINGLE)])

    rate = rospy.Rate(60) # In Hz
    while not rospy.is_shutdown():
      hello_str = "hello world %s" % rospy.get_time()
      rospy.loginfo(hello_str)

      # Query images from Unity
      data_response = env.request(data_request)
      print(data_response.data)
      imgs = data_response.images
      for i in range(len(imgs)):
        cv2.imshow("Image with Id: %s"%i, imgs[i])

      # Publish images to ROS

      # Wait to visualize image
      cv2.waitKey(1)
      rate.sleep()

if __name__ == '__main__':
    try:
        tesse_ros_bridge()
    except rospy.ROSInterruptException:
        pass
