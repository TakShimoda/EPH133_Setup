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
#ros2 launch D435i_launch.py enable_color:=true enable_depth:=false enable_gyro:=false enable_accel:=false  &> ${D435I_LOG}
ros2 launch D435i_apriltag_launch.py enable_color:=true enable_depth:=false enable_gyro:=false enable_accel:=false  &>  {D435I_LOG} &
sleep 3  # Give the launch some time to complete

#ros2 launch apriltag_ros tag_36h11_all.launch.py

# Create the csv filename
DATE=$(date +%m%d)
BASE_NAME="${ROBOT_ID}-Apriltags-${DATE}"

# Check for existing csv with same base name 
COUNT=1
while [[ -d ${BASE_NAME}-${COUNT} ]]; do
  let COUNT=COUNT+1
done

# Construct final unique name
CSV_NAME="${BASE_NAME}-${COUNT}"

echo "save to ${CSV_NAME}"

mkdir ${CSV_NAME} && cd ${CSV_NAME}

ros2 topic echo /tf tf2_msgs/msg/TFMessage --csv --qos-history keep_all --qos-reliability reliable > ${CSV_NAME}.csv

