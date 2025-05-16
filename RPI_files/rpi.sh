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

# If ROBOT_ID contains "W", set TURTLEBOT3_MODEL to waffle_pi. Else, set it to burger.
if [[ $ROBOT_ID == *"W"* ]]; then
    TURTLEBOT3_MODEL="waffle_pi"
else
    TURTLEBOT3_MODEL="burger"
fi

# Define log filenames
BRINGUP_LOG="bringup.log"
ROSBAG_LOG="rosbag.log"
CONTROLLER_LOG="controller.log"

# Setup the ROS environment
export TURTLEBOT3_MODEL
export ROS_DOMAIN_ID=30  
source /opt/ros/foxy/setup.bash
source /home/ubuntu/turtlebot3_ws/install/setup.bash

# Delete old log files if they exist
rm -f ${BRINGUP_LOG} ${ROSBAG_LOG} ${CONTROLLER_LOG}

# Launch bringup script and log output
ros2 launch my_tb3_launcher my_tb3_bringup.launch.py &> ${BRINGUP_LOG} &
sleep 4  # Allow launch to complete
ros2 topic pub -1 /${ROBOT_ID}/pose_relocalization geometry_msgs/Point
ros2 param set /${ROBOT_ID}/diff_drive_controller odometry.use_imu False
echo "${ROBOT_ID}"

# Create the base rosbag filename
DATE=$(date +%m%d)
BASE_NAME="${ROBOT_ID}-Rpi-${DATE}"

# Check for existing bag with same base name 
COUNT=1
while [[ -d ${BASE_NAME}-${COUNT} ]]; do
  let COUNT=COUNT+1
done

echo "${ROBOT_ID}"

# Construct final unique name
ROSBAG_NAME="${BASE_NAME}-${COUNT}"
# Construct final unique name 

# Record rosbag and log output
#ros2 bag record -o ${ROSBAG_NAME} /tf /tf_static /${ROBOT_ID}/robot_path /${ROBOT_ID}/waypoint_markers /${ROBOT_ID}/odom /${ROBOT_ID}/imu /vicon/${ROBOT_ID}/${ROBOT_ID} ${ROBOT_ID}/robot_description  &> ${ROSBAG_LOG} &
#ros2 bag record -o ${ROSBAG_NAME} /tf /tf_static /vicon/${ROBOT_ID}/${ROBOT_ID} /${ROBOT_ID}/odom /${ROBOT_ID}/imu &> ${ROSBAG_LOG} &

sleep 2  # Allow rosbag to start recording

echo "Save to: ${ROSBAG_NAME}"

ros2 run tb3_controller tb3_nav_action_client --name ${ROBOT_ID} --config "turtlebot3_ws/src/tb3_controller/config/client_config.yaml"

# Launch the controller for the provided robot ID and log output
#ros2 launch controller ${ROBOT_ID}_controller.launch.py &> ${CONTROLLER_LOG} &
#ros2 run turtlebot3_teleop teleop_keyboard --ros-args --remap __ns:=/${ROBOT_ID}
#echo "running publisher"
#ros2 topic pub -t 4 B04/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.8}}"
#ros2 topic pub -1 B04/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
#ros2 topic pub B04/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
