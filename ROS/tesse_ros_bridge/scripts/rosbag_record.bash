#!/bin/bash

rosbag record \
/clock \
/tesse/depth \
/tesse/gt \
/tesse/imu \
/tesse/left_cam \
/tesse/left_cam/camera_info \
/tesse/odom \
/tesse/right_cam \
/tesse/right_cam/camera_info \
/tesse/segmentation \
/tf \
/tf_static
