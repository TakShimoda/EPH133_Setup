#!/bin/bash

# Kill processes by name
pkill -f ros2
pkill -f realsense2_came
pkill -f _ros2_daemon

echo "Attempted to kill specified processes."
