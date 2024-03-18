# EPH133_Setup
This repository provides guidance on setting up and working with the turtlebots for the vicon room in EPH133, including the Jetson Nano and Realsense D435i cameras.

## Table of Contents

[Jetson Nano Setup](https://github.com/TakShimoda/EPH133_Setup?tab=readme-ov-file#jetson-nano-setup)   
[Raspberry pi Setup](https://github.com/TakShimoda/EPH133_Setup?tab=readme-ov-file#raspberry-pi-setup)  
[Running the robots with vicon](https://github.com/TakShimoda/EPH133_Setup?tab=readme-ov-file#running-the-robots-with-vicon)  
[Save files from Jetson on USB by command line](https://github.com/TakShimoda/EPH133_Setup?tab=readme-ov-file#save-files-from-jetson-on-usb-by-command-line)  
[Savings changes from remote repositories](https://github.com/TakShimoda/EPH133_Setup?tab=readme-ov-file#saving-changes-from-remote-repositories)  
[Troubleshooting errors](https://github.com/TakShimoda/EPH133_Setup?tab=readme-ov-file#troubleshooting-errors)
## Jetson Nano Setup
### Setting up the OS
1. Download Balena Etcher https://etcher.balena.io/
2. Open Balena and Select img.tz file. Download the image from [here](https://drive.google.com/drive/folders/1qM5vqfcCoc4Gt38sy7KjRVl5En-bksCO)
3. Select the USB and flash Ubuntu on the SD card.
4. Make note of the IP address with ```ifconfig``` and checking under wlan.
### Setting up software
1. Install ROS2 foxy following instructions from the official website. 
2. Install the librealsense library following the instructions [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md)
3. Install the debian package ```librealsense-ros``` from ROS servers following the instructions [here](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation):

&ensp;&ensp;&ensp;&ensp;&ensp;&ensp; ```sudo apt install ros-foxy-realsense2-*```

4. Confirm that udev-rules are in place so that the Jetson can enable the accelerometer and gyroscope of the realsense.
   - Test if the following works:
     ```
     ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true 
     ```
   - If it doesn't work, the topics, such as ```/camera/imu``` won't publish and there will be error messages, such as the following:
     
     ```[ERROR] [1705959753.179528268] [camera.camera]: /tmp/binarydeb/ros-foxy-realsense2-camera-4.51.1/src/rs_node_setup.cpp:344:An exception has been thrown: Failed to set frequency 1. device path: /sys/devices/70090000.xusb/usb2/2-1/2-1.1/2-1.1:1.5/0003:8086:0B3A.000A/HID-SENSOR-200073.1.auto/iio:device1/in_accel_sampling_frequency Last Error: Permission denied ```
   - To fix this, re-isntall SDK by following step 2 above (first 3 steps in the link), then also install udev-rules:
     ```
     sudo apt-get install librealsense2-udev-rules
     ```
   - Reboot the Jetson after installing, and this issue should be fixed. Try running the the first command in this step again to ensure there are no error messages, and check that topics, such as ```/camera/imu``` are published.
5. Confirm there's no mismatch between the realsense SDK and the firmware. This can cause an ```auto_exposure_limit``` error.
   -  **Note**: the version we are interested in is the one that shows when running  the realsense2 ros node. When you launch a node with for example, ```ros2 launch realsense2_camera rs_launch.py```, you should see the line:
   
      ```...Built with LibRealSense v2.51.1```

   -  This is the SDK version you should check your firmware against
      - The SDK version from step 2 may be different and can be found with e.g. ```find /usr -name *realsense*```. You should see the version with the .so file under ```/usr/lib```. At the time of writing, the version installed from debian packages should be 2.54.2 (even if the one that librealsense-ros uses may be different)
   - Find the firmware version by plugging in the camera, opening ```realsense-viewer```, and checking the firmware version under **info**. It should be something like 5.15.1
   - Check from the [firmware downloads page](https://dev.intelrealsense.com/docs/firmware-releases) if your SDK and firmware matches up. If not, change it so they do.
   - If your firmware needs to be updated, the realsense-viewer should show it to you and you can update from there. If you need to downgrade your firmware version, you should go to the firmware downloads page shown above, download the appropriate firmware, put it into a USB into the Jetson, then in the realsense-viewer go to **More**->**Update Firmware...** and select the bin file from the firmware package you downloaded.
  
6. Install the ```foxy-future branch``` of rosbag2 from [source](https://github.com/ros2/rosbag2/tree/foxy-future):
   - First make an empty ros2 workspace and source the local setup bash script:
     
      ```
      mkdir -p ros2_ws/src
      cd ros2_ws
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
     cd ~/ros2_ws/src
     git clone -b foxy-future https://github.com/ros2/rosbag2
     ```
   - Then install some dependencies to properly build the packages:
     ```
     sudo apt install ros-foxy-test-msgs ros-foxy-pybind11-vendor
     ```
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
     
7. Copy the files from jetson_files to the home directory:  
   ```
   cp Jetson_files/*.* /home/jetson
   ```
8. Make an environment variable for the specified robot (optional).
   
## Raspberry pi Setup
1. Setup as shown in the turtlebot3 guide, installing ros2 foxy.
   - Also make sure to have argcomplete installed so the tab auto-complete works in the terminal:
      ```
      sudo apt install python3-colcon-common-extensions
      echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
      source ~/.bashrc
      ```
3. Copy the contents of rpi_wifi.txt to ```/etc/netplan/50-cloud-yaml```. rpi_wifi.txt can be found [here](https://drive.google.com/drive/folders/1qM5vqfcCoc4Gt38sy7KjRVl5En-bksCO). Modify so the IP address matches the output of ```ifconfig``` under wlan.
   - Also make sure to apply changes after editing the file with:
      ```
      sudo netplan apply
      ```
4. Clone the repository [my_tb3_launcher](https://github.com/h2jaafar/my_tb3_launcher) into turtlebot3/src.
5. Change any instances of "B04" to the Burger number of your choice.
6. Copy the RPI.sh and kill_rpi.sh scripts into the home folder:
      ```
      cp RPI_files/*.* /home/ubuntu
      ```
## Running the robots with vicon.
1. Start up the shell scripts to run the vicon setup.
2. ssh into the raspberry pi and jetson nano. Password for RPI is ```turtlebot``` and password for jetson is ```jetson```
   - For the Raspberry Pi (e.g. B04):
   ```ssh ubuntu@192.168.0.204```
   - For the Jetson Nano (e.g. B04:
   ```ssh jetson@192.168.0.230```
4. Run the scripts for both rpi and jetson, and then record the vicon topic for the ground-truth.
   - For the jetson, there are two options:
     1. Run the normal jetson.sh script and provide the robot number, e.g.:
        ```
        ./jetson.sh B04
        ./kill_jetsons.sh #when you're done recording.
        ```
        - This option requires you to modify parameters such as launch arguments and recorded topics in the sh file itself.
     2. Run the jetson_args.sh script and provide the ```args.txt``` file as an argument, e.g.:
        ```
        ./jetson_args.sh args.txt
        ./kill_jetsons.sh #when you're done recording.
        ```
        - This option requires you to modify parameters inside the args.txt file, which is easier and more organized. The robot namespace is also handled here,and different capture profiles (e.g. RGB, infra) can be appended to args.txt after a "- " in the lastline of the file.

## Save files from Jetson on USB by command line
1. Plug in your usb and find its mount point by typing
    ```
   lsblk
    ```
   - You will get a result, such as ```/media/jetson/LEXAR```
2. Copy the folder to your USB:
    ```
   cp -r <FOLDER> <MOUNT POINT>
   # e.g. cp -r B04-Jetson-0201-1 /media/jetson/LEXAR
    ```
3. Safely eject the USB:
    ```
   udisksctl unmount -b /dev/sda1
   udisksctl power-off -b /dev/sda
    ```
## Saving changes from remote repositories
The turtlebots have packages from remote repositories installed on them. To save changes made on the remote repositories, use the following commands:
```
cd YOUR_REPOSITORY
git fetch \origin #Fetch the changes from remote
git stash push --include-untracked #Discard local changes. If you have important changes on turtlebot, don't do this!
git merge origin/master #Merge the remote changes to your local repo.
cd YOUR_REPOSITORY
colcon build --packages-select YOUR_REPOSITORY #--symlink-install if it's a Python package
```
- sometimes in a python package, it may change the permission of your python codes to not be executable (shown in white instead of green when using the ```ls``` command). In that case, add executable permission ```chmod +x <file name>.py``` before doing the colcon build.

## Troubleshooting Errors
### RPI
#### /odom gives extremely large values
A known issue on the turtlebot is when it gives extremely large values for odometry after startup, as mentioned [here](https://github.com/ROBOTIS-GIT/turtlebot3/issues/880). This issue stems from bytestuffing from the DYNAMIXEL2Arduino library, which causes the issue, confirmed [here](https://github.com/ROBOTIS-GIT/turtlebot3/issues/926#issuecomment-1403244766)
One way to alleviate the issue is to manually create a subscriber in the turtlebot3 node to subscribe to a ```/pose_relocalization topic```, and manually send all zeros to it, making the robot start at a pose with all values at zero. 
To do so, go to the raspberry pi, and cd to the ```turtlebot3_node``` workspace:
```
cd /turtlebot3_ws/src/turtlebot3/turtlebot3_node
```
Then use the following commands to add the appropriate lines to ```src/odometry.cpp```: 
```
sed -i '107i\
\n\  pose_relocalization_state_sub_ = nh_->create_subscription<geometry_msgs::msg::Point>(\n\
    "pose_relocalization",\n\
    qos,\n\
    std::bind(&Odometry::pose_relocalization_callback, this, std::placeholders::_1));' src/odometry.cpp
echo -e "\nvoid Odometry::pose_relocalization_callback(const geometry_msgs::msg::Point::SharedPtr point)\n{\n  robot_pose_[0] = point->x;\n  robot_pose_[1] = point->y;\n  robot_pose_[2] = point->z;\n}" >> src/odometry.cpp
```
Then use the following commands to add the appropriate lines to ```include/turtlebot3_node/odometry.hpp```:
```
awk -v lineno=60 -v text="  void pose_relocalization_callback(const geometry_msgs::msg::Point::SharedPtr point);" 'NR == lineno {print "\n" text "\n"} NR != lineno' include/turtlebot3_node/odometry.hpp > tmpfile && mv tmpfile include/turtlebot3_node/odometry.hpp
awk -v lineno=70 -v text="  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pose_relocalization_state_sub_;" 'NR == lineno {print text "\n"} NR != lineno' include/turtlebot3_node/odometry.hpp > tmpfile && mv tmpfile include/turtlebot3_node/odometry.hpp
```
- Note: for the above commands, make sure to do them one by one, as the insertion is based on specific lines in the code, and those are based on the code after the previous command has already inserted lines into it.

After using the above commands, the two files should look like the ones from [this package](https://github.com/paolorugg/my_turtlebot3_node/tree/main/tbt3_node/turtlebot3_node), where [lines 108-111](https://github.com/paolorugg/my_turtlebot3_node/blob/main/tbt3_node/turtlebot3_node/src/odometry.cpp#L108-L111) and [lines 283-288](https://github.com/paolorugg/my_turtlebot3_node/blob/main/tbt3_node/turtlebot3_node/src/odometry.cpp#L283-L288) should be present in ```src/odometry.cpp```, and the lines [here](https://github.com/paolorugg/my_turtlebot3_node/blob/main/tbt3_node/turtlebot3_node/include/turtlebot3_node/odometry.hpp#L64) and [here](https://github.com/paolorugg/my_turtlebot3_node/blob/main/tbt3_node/turtlebot3_node/include/turtlebot3_node/odometry.hpp#L73) should be in ```include/turtlebot3_node/odometry.hpp```.

Finally, rebuild the package:
```
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot3_node
source ~/.bashrc
```

#### ModuleNotFoundError: No module named 'tf_transformations'
If a Python script encounters this error, use the following commands:
```
sudo apt-get update
sudo apt-get install ros-foxy-tf-transformations
sudo pip3 install transforms3d
```
- For the first command, you can ctrl+c when it fetched everything, and for the second, you can do it at the line ```Processing triggers for man-db (2.9.1-1) ...```
