#!/bin/bash

rosbag record \
/clock \
/tesse/gt \
/tesse/imu \
/tesse/left_cam \
/tesse/depth \
/tesse/segmentation \
/tesse/left_cam/camera_info \
/tesse/odom \
/tesse/right_cam \
/tesse/right_cam/camera_info \
/tf \
/tf_static
