#!/bin/bash

# Exit script on first error  
set -e

# Check if a file path is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <file_path>"
    exit 1
fi

file_path="$1"

# Initialize variables
ROBOT_ID=""
launch_args=""
capture_profile=""
bag_topics=""

# Read ROBOT_ID and launch_args from the first two lines, and capture profile from the third
while IFS= read -r line; do
    if [ -z "$ROBOT_ID" ]; then
        # Extract ROBOT_ID from the line assuming the format 'robot: {robot_id}'
        ROBOT_ID=$(echo "$line" | sed 's/robot: //')
    elif [ -z "$launch_args" ]; then
        # Process the second line to include everything after the first ":"
        launch_args=$(echo "$line" | cut -d':' -f2-)
    elif [ -z "$capture_profile" ]; then
        # Extract the capture_profile
        capture_profile=$(echo "$line" | sed 's/capture_profile: //')
    else
        # Process subsequent lines to match the capture profile and extract topics
        if echo "$line" | grep -q "^- $capture_profile:"; then
            topics_line=$(echo "$line" | sed "s/- $capture_profile: //")
            # Split topics and prepend ROBOT_ID to each
            IFS=' ' read -ra ADDR <<< "$topics_line"
            for i in "${ADDR[@]}"; do
                bag_topics+="/$ROBOT_ID$i "
            done
            bag_topics=$(echo "$bag_topics" | sed 's/ $//') # Trim the trailing space
            break # Exit loop after finding and processing the topics for the capture profile
        fi
    fi
done < "$file_path"

echo "Capture Profile: ${capture_profile}"

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
ros2 launch D435i_launch.py $launch_args &> ${D435I_LOG} &
sleep 3  # Give the launch some time to complete

#Disable emitter
ros2 param set /${ROBOT_ID}/D435i/D435i depth_module.emitter_enabled 0
echo "emitter disabled."

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
ros2 bag record -o ${ROSBAG_NAME} --max-cache-size 0 $bag_topics &> ${ROSBAG_LOG} &   

