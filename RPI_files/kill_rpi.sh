#!/bin/bash

# Kill processes by name
pkill -f ros2
pkill -f robot_state_pub
pkill -f hlds_laser_publ
pkill -f turtlebot3_ros
pkill -f controller_node

echo "Attempted to kill specified processes."

