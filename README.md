# EPH133_Setup
This repository provides guidance on setting up and working with the turtlebots for the vicon room in EPH133, including the Jetson Nano and Realsense D435i cameras.

## Jetson Nano Setup
1. Flash a Ubuntu 20.04 image on the Jetson and install ROS2. Also make note of the IP address with ```ifconfig```. 
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
  
5. Install the ```foxy-future branch``` of rosbag2 from [source](https://github.com/ros2/rosbag2/tree/foxy-future):
   - First make an empty ros2 workspace and source the local setup bash script:
     
      ```
      mkdir -p ros2_ws/src
      cd ros2_ws/src
      colcon build
      ```
   - Then add the following lines to ```.bashrc``` after the line sourcing the main ros2 installation:
     ```
     source ~/ros2_ws/install/local_setup.bash
     source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
     source /usr/share/colcon_cd/function/colcon_cd.sh
     export _colcon_cd_root=/opt/ros/foxy/
     ```
     and source ```.bashrc```.    
   - Then clone the repository of rosbag2:
     ```
     git clone -b foxy-future https://github.com/ros2/rosbag2
     ```
   - Then install some dependencies to properly build the packages:

     ```sudo apt install ros-foxy-test-msgs ros-foxy-pybind11-vendor```
   - Then build the following packages step by step, sourcing .bashrc after every build:
     ```
     rosbag2_test_common
     rosbag2_compression_zstd (altough this does give warnings as it can't generate safe runtime search path for target)
     rosbag2_storage
     rosbag2_storage_default_plugins
     rosbag2_cpp
     rosbag2_compression
     rosbag2_storage_evaluation
     rosbag2_py
     rosbag2_transport
     zstd_vendor
     ros2bag
     ```
   - There are some packages, like ```rosbag2_tests``` and ```rosbag2``` which have issues building, but they don't seem important for the purposes of recording and playing back bag files.
     
6. Copy the contents of jetson.sh
7. Make an environment variable for the specified robot
   
## Raspberry pi Setup

## Startup script in vicon room

