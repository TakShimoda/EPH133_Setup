#!/bin/bash

# Exit script on first error  
set -e

# Check if the script received the robot ID
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 robot_ID" >&2
    exit 1
fi

# Robot ID passed as an argument 
ROBOT_ID="$1"

# Log filenames
D435I_LOG="D435i.log"
ROSBAG_LOG="rosbag.log"

# Setup the ROS environment
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=1

# Delete old log files if they exist
rm -f ${D435I_LOG} ${ROSBAG_LOG}

# Launch D435i script and log output
ros2 launch D435i_launch.py unite_imu_method:=2 enable_sync:=true enable_color:=false enable_gyro:=true enable_accel:=true gyro_fps:=400 accel_fps:=250 &> ${D435I_LOG} &
sleep 3  # Give the launch some time to complete

# Create the base rosbag filename
DATE=$(date +%m%d)
BASE_NAME="${ROBOT_ID}-Jetson-${DATE}"

# Check for existing bag with same base name 
COUNT=1
while [[ -d ${BASE_NAME}-${COUNT} ]]; do
  let COUNT=COUNT+1
done

# Construct final unique name
ROSBAG_NAME="${BASE_NAME}-${COUNT}"

echo "save to ${ROSBAG_NAME}"

# Record rosbag and log output
#----First line: RGB-D. Second line: stereo
#ros2 bag record -o ${ROSBAG_NAME} --max-cache-size 0 /${ROBOT_ID}/D435i/color/image_raw /${ROBOT_ID}/D435i/color/camera_info /${ROBOT_ID}/D435i/depth/image_rect_raw /${ROBOT_ID}/D435i/depth/camera_info /${ROBOT_ID}/D435i/extrinsics/depth_to_color &> ${ROSBAG_LOG} &
ros2 bag record -o ${ROSBAG_NAME} --max-cache-size 0 /${ROBOT_ID}/D435i/infra1/camera_info /${ROBOT_ID}/D435i/infra2/camera_info /${ROBOT_ID}/D435i/infra1/image_rect_raw /${ROBOT_ID}/D435i/infra2/image_rect_raw /${ROBOT_ID}/D435i/imu /${ROBOT_ID}/D435i/accel/imu_info /${ROBOT_ID}/D435i/accel/metadata /${ROBOT_ID}/D435i/accel/sample /${ROBOT_ID}/D435i/gyro/imu_info /${ROBOT_ID}/D435i/gyro/metadata /${ROBOT_ID}/D435i/gyro/sample &> ${ROSBAG_LOG} &   
