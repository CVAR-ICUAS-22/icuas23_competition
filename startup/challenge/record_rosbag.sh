#!/bin/bash

rosbag record \
    /tf \
    /tf_static \
    /red/camera/depth/image_rect_raw \
    /red/camera/depth/camera_info \
    /red/camera/color/image_raw/compressed \
    /red/camera/color/camera_info \
    /red/odometry
