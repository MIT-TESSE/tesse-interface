#!/usr/bin/env python

import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from tesse_ros_bridge import gen_cityscapes as gc

class Params():
  def __init__(self):
    print("Parsing Params")

  def parseParams(self):
    # Model name
    self.model = rospy.get_param("~model", "ESPNetv2")
    # Data directory
    self.data_dir = rospy.get_param("~data_dir", './data')
    # RGB Image format
    self.img_extn = rospy.get_param("~img_extn", "png")
    # Width of RGB image
    self.inWidth = rospy.get_param("~inWidth", 1024)
    # Height of RGB image
    self.inHeight = rospy.get_param("~inHeight", 512)
    # Whether to write segmented images to save_dir or not
    self.save_imgs = rospy.get_param('~save_imgs', False)
    # Directory to save the results (ignored if save_img is False)
    self.save_dir = rospy.get_param("~save_dir", "./results")
    # Run on CPU or GPU. If TRUE, then GPU.
    self.gpu = rospy.get_param('~gpu', True)
    # Pretrained weights directory.
    self.pretrained = rospy.get_param('~pretrained', './pretrained')
    # Scale
    self.scale = rospy.get_param('~scale', 0.5)
    # If you want to convert to cityscape original label ids')
    self.cityFormat = rospy.get_param('~cityFormat', True)
    #If you want to visualize the segmentation masks in color')
    self.colored = rospy.get_param('~colored', True)
    # If you want to visualize the segmentation masks overlayed on top of RGB image
    self.overlay = rospy.get_param('~overlay', True)
    # Number of classes in the dataset. 20 for Cityscapes
    self.classes = rospy.get_param('~classes', 20)

    if self.overlay:
      self.colored = True # This has to be true if you want to overlay

class ImageConverter:
  def __init__(self, params):
    # Store params for semantic segmentation
    self.params = params

    # Create ROSMITNET for semantic segmentation
    print("Using Model: {}".format(params.model))
    self.rosmitnet = gc.ROSMITNET(params)

    # Setup ROS interface
    self.image_pub = rospy.Publisher("image_out", Image, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_in", Image, self.callback)


  def callback(self, data):
    print("Got an Image!")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    cv_image = self.semanticSegmentation(cv_image)

    # Visualize in ROS
    self.publishToROS(data.header, cv_image)


  def semanticSegmentation(self, cv_image):
    # Run semantic segmentation on a single image.
    return self.rosmitnet.segmentImage(self.params, cv_image)

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

    image_converter = ImageConverter(params)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
      hello_str = "hello world %s" % rospy.get_time()
      rospy.loginfo(hello_str)

      cv2.waitKey(1)
      rate.sleep()

if __name__ == '__main__':
    try:
        tesse_ros_bridge()
    except rospy.ROSInterruptException:
        pass
