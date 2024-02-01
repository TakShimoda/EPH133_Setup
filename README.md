# EPH133_Setup
This repository provides guidance on setting up and working with the turtlebots for the vicon room in EPH133, including e Jetson Nano and Realsense D435i cameras.

## Jetson Nano Setup
1. Flash a Ubuntu 20.04 image on the Jetson.
2. Install the librealsense library following the instructions [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md)
3. Install the debian package ```librealsense-ros``` from ROS servers following the instructions [here](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation):

&ensp;&ensp;&ensp;&ensp;&ensp;&ensp; ```sudo apt install ros-foxy-realsense2-*```


4. Confirm there's no mismatch between the realsense SDK and the firmware. This can cause an ```auto_exposure_limit``` error.
   -  **Note**: the version we are interested in is the one that shows when running  the realsense2 ros node. When you launch a node with for example, ```ros2 launch realsense2_camera rs_launch.py```, you should see the line:
   
      ```...Built with LibRealSense v2.51.1```

   -  This is the SDK version you should check your firmware against
      - The SDK version from step 2 may be different and can be found with e.g. ```find /usr -name *realsense*```. You should see the version with the .so file under ```/usr/lib```. At the time of writing, the version installed from debian packages should be 2.54.2 (even if the one that librealsense-ros uses may be different)
   - Find the firmware version by plugging in the camera, opening ```realsense-viewer```, and checking the firmware version under **info**. It should be something like 5.15.1
   - Check from the [firmware downloads page](https://dev.intelrealsense.com/docs/firmware-releases) if your SDK and firmware matches up. If not, change it so they do.
   - If your firmware needs to be updated, the realsense-viewer should show it to you and you can update from there. If you need to downgrade your firmware version, you should go to the firmware downloads page shown above, download the appropriate firmware, put it into a USB into the Jetson, then in the realsense-viewer go to **More**->**Update Firmware...** and select the bin file from the firmware package you downloaded.
  
5. Installing the ```foxy-future branch``` of rosbag2 from [source](https://github.com/ros2/rosbag2/tree/foxy-future):
   - First make a ros2 workspace and clone the branch:
     
      ```
      mkdir -p ros2_ws/src
      cd ros2_ws/src
      git clone -b foxy-future https://github.com/ros2/rosbag2
      ```
