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
     ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=2
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
  
6. Install the ```foxy-future branch``` of rosbag2 from [source](https://github.com/ros2/rosbag2/tree/foxy-future). This is to fix an issue of topics being dropped when recording to bag files, which will be tested in the next step. The steps to install the new rosbag2 are:
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
     export ROS_DOMAIN_ID=30
     export ROS_LOCALHOST_ONLY=1
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
   - The package ```rosbag2_storage_evaluation``` is actually named ```ros2_rosbag_evaluation```, so this latter name should be used when building. Also delete the ```COLCON_IGNORE``` file inside this package to build it with colcon 
   - There are some packages, like ```rosbag2_tests``` and ```rosbag2``` which have issues building, but they don't seem important for the purposes of recording and playing back bag files.
7. Test that the bag record function works (doesn't drop topics) by recording topics and confirming they have the same frequency as what's published live:
   - First launch the camera with imu at the maximum frequency (400Hz) with linear interpolation method (unite_imu_method=2), and as an example for images, disable RGB and enable infrared streams for the camera:
     ```
     ros2 launch realsense2_camera rs_launch.py unite_imu_method:=2 enable_sync:=true enable_color:=false enable_infra1:=true enable_infra2:=true enable_gyro:=true enable_accel:=true gyro_fps:=400 accel_fps:=250
     ```
   - Check the frequency on the topics, where the main topic of interest is ```/camera/imu```, which should be 400Hz:
     ```
     ros2 topic hz /camera/imu
     ```
     - This should return about 400Hz. You can also check the gyroscope stream to ensure it's 400Hz, the accelerometer stream to ensure it's 250Hz, and the infra1 and infra2 streams to ensure they're 30fps (default)
   - Now record these topics to a folder bag_test.
   ```
   ros2 bag record -o bag_test topics /camera/imu /camera/accel/sample /camera/gyro/sample /camera/infra1/image_rect_raw /camera/infra2/image_rect_raw
   ```
   - Now check the bag file ```bag_test_0.db3``` to ensure all the topics have the same frequencies they published at. First you can simply check the topic counts and bag duration with ```ros2 bag info bag_test_0.db3```, then divide the counts by the duration. You can also playback the bag file and check the frequencies to ensure they're consistent on playback. If the frequencies in the bag file matches the live frequency, then the bag recorder is working and topics are not dropped as they're being recorded.
   
8. Copy the files from jetson_files to the home directory:  
   ```
   cp Jetson_files/*.* /home/jetson
   ```
   - Also make sure to give executable permissions to these script files (```jetson.sh```, ```jetson_args.sh```, ```kill_jetsons.sh```)
   - Make sure to change the robot name from B04 to the appropriate name in the files ```args.txt``` and ```D435i_launch.py```.
9. Set up appropriate time and sync with Chrony NTP
   - By default, Nano will have a different timezone, so to set it to EST:
      ``` 
      sudo timedatectl set-timezone America/Toronto
      ```
      - Or set it manually in the GUI in settings.
   - Now, install chrony to setup NTP to sync the clock with a server's clock:
      ```
      sudo apt install chrony -y
      ```
   - Then start the NTP server by syncing to the first source under ```/etc/chrony/chrony.conf``` (should be ntp.ubuntu.com):
      ```
      sudo chronyd -q 'server ntp.ubuntu.com iburst'
      ```
      - If this returns an error with another chronyd already running, kill that one with ```sudo systemctl stop chronyd```
   - This should show the system clock being adjusted. Then, to start the chronyd daemon so it's available across the reboots:
      ```
      systemctl start chronyd
      systemctl enable chrony
      ```
10. Make an environment variable for the specified robot (optional).
11. For setting up the system to record tf data onto /tf_static (for Swarm-SLAM or any other algorithm that needs TF frame data), make some minor adjustments:
    - Modify D435i_launch.py so the ```camera_name``` parameter is set to the name of the robot, e.g. 'B01' instead of 'D435i'
    - Comment out [line 65](https://github.com/TakShimoda/EPH133_Setup/blob/main/Jetson_files/jetson_args.sh#L65) and uncomment [line 66](https://github.com/TakShimoda/EPH133_Setup/blob/main/Jetson_files/jetson_args.sh#L66) from jetson_args.sh to account for the different node names. Topics should remain the same as the namespacing parameter of D435i_launch.py remains the same.
    - This is only for algorithms like Swarm-SLAM mentioned above. For anything else, no need to modify. Also, this is a temporary solution in place of namespacing TF frames, however that requires uninstalling librealsense and installing at least 4.54.1 from source and modifying lines in the file mentioned [here](https://github.com/IntelRealSense/realsense-ros/issues/3119#issuecomment-2163458651)

### Setting up AprilTag detection in ROS2
1. Install the apriltag ROS library
   ```
   git clone https://github.com/AprilRobotics/apriltag
   cd apriltag && mkdir build
   cmake -B build -DCMAKE_BUILD_TYPE=Release
   cmake --build build
   cd build && sudo make install
   ```
   - This will install the apriltag libraries and headers under ```/usr/local/lib``` and ```/usr/local/include```, which should be noted when linking it to the ROS wrapper.
2. Install the apriltag_msgs and apriltag ROS2 libraries:
   ```
   cd ~/ros2_ws/src
   git clone https://github.com/christianrauch/apriltag_msgs
   git clone https://github.com/christianrauch/apriltag_ros
   ```
   - install the apriltag_msgs library first:
     ```
      cd ~/ros2_ws
      colcon build --packages-select apriltag_msgs
      source ~/.bashrc
      ```
   - install the apriltag_ros library with some modifications. In the CMakeLists.txt, before the find_package() command for apriltag, add the lines:
     ```
     set(CMAKE_PREFIX_PATH "/usr/local" $ {CMAKE_PREFIX_PATH})
     set(CMAKE_INSTALL_RPATH "/usr/local/lib" $ {CMAKE_INSTALL_RPATH})
     ```
   - then build:
     ```
      cd ~/ros2_ws
      colcon build --packages-select apriltag_ros
      source ~/.bashrc
      ```     
3. Copy the apriltags.sh and D435i_apriltag_launch.py files onto the home directory. Edit line 88 of D435i_apriltag_launch.py and change B05 to the robot being used. 
4. Run the script with:
   ```
   ./apriltags.sh <ROBOT_NAME>
   ```   
   - This will run launch the realsense script and apriltag library, while also recording the tf topic onto a csv inside a folder.
   - e.g. ./apriltags.sh B05 would put a csv into a folder called B05-Apriltags-0130-1. These are the raw outputs of the apriltag detections
   
## Raspberry pi Setup
1. Setup as shown in the turtlebot3 guide, installing ros2 foxy.
   - Also make sure to have argcomplete installed so the tab auto-complete works in the terminal:
      ```
      sudo apt install python3-colcon-common-extensions
      echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
      source ~/.bashrc
      ```
2. Copy the contents of rpi_wifi.txt to ```/etc/netplan/50-cloud-yaml```. rpi_wifi.txt can be found [here](https://drive.google.com/drive/folders/1qM5vqfcCoc4Gt38sy7KjRVl5En-bksCO). Modify so the IP address matches the output of ```ifconfig``` under wlan. Note, even after changing the WiFi network and password in the file, the IP address should remain the same.
   - Also make sure to apply changes after editing the file with:
      ```
      sudo netplan apply
      ```
3. Clone the repository [my_tb3_launcher](https://github.com/h2jaafar/my_tb3_launcher) into turtlebot3/src and build with symlink install since it's a python package.
     ```
     cd ~/turtlebot3_ws/src
     git clone https://github.com/h2jaafar/my_tb3_launcher
     cd ~/turtlebot3_ws
     colcon build --packages-select my_tb3_launcher --symlink-install
     source ~/.bashrc
     ```
   - There are multiple instances where you will have to change 'B04' with the robot you're working with (e.g. B01):
      - ```my_tb3_launcher/param/burger.yaml``` for burger or ```my_tb3_launcher/param/waffle_pi.yaml``` for waffle
      - ```my_tb3_launcher/launch/my_tb3_bringup.launch.py```, line 91 (namespace for turtlebot3_node)
      - ```my_tb3_launcher/launch/turtlebot3_state_publisher.launch.py```, line 59 (namespace for robot_state_publisher)

5. Clone the repositories [tb3_controller](https://github.com/TakShimoda/tb3_controller) and [tb3_interfaces](https://github.com/TakShimoda/tb3_interfaces) and build them using symlink install for tb3_controller since it's a python package:
   ```
   cd ~/turtlebot3_ws/src
   git clone https://github.com/TakShimoda/tb3_controller
   git clone https://github.com/TakShimoda/tb3_interfaces
   cd ~/turtlebot3_ws
   colcon build --packages-select tb3_interfaces tb3_controller --symlink-install
   source ~/.bashrc
   ```
6. Copy the RPI.sh and kill_rpi.sh scripts into the home folder:
      ```
      cp RPI_files/*.* /home/ubuntu
      ```
7. If not already done, remember to uncomment the line at the end of ~/.bashrc to export the appropriate turtlebo3 model, e.g. ```export TURTLEBOT3_MODEL=burger```
8. Set up appropriate time and sync with Chrony NTP
   - By default, RPI will be set to UTC, so to set it to EST which is UTC-04:00:
      ``` 
      sudo timedatectl set-timezone America/Toronto
      ```
   - Now, install chrony to setup NTP to sync the clock with a server's clock:
      ```
      sudo apt install chrony -y
      ```
   - Then start the NTP server by syncing to the first source under ```/etc/chrony/chrony.conf``` (should be ntp.ubuntu.com):
      ```
      sudo chronyd -q 'server ntp.ubuntu.com iburst'
      ```
   - This should show the system clock being adjusted. Then, to start the chronyd daemon so it's available across the reboots:
      ```
      systemctl start chronyd
      systemctl enable chrony
      ```
## Running the robots with vicon.
1. Startup the vicon system. Login to RCVL-temp with password: rcvl.133 and start up the vicon tracker software.
2. Start up the xubuntu computer, with the same password, rcvl.133. 
3. On the xubuntu computer, start up the shell scripts to run the vicon setup by running the vicon startup file with tmux:
   ```
   tmuxp load vicon_startup.yaml
   ```
   - The file ```vicon_startup.yaml``` is from this repository, and loads up all the necessary terminal commands to get the vicon system running.
   - There are multiple windows with multiple panes each. To toggle and control these panes, you must press ```Ctrl+b``` and quickly press a key that corresponds to a command:
      - ```z``` for full screen on a pane
      - arrow keys to navigate between panes
      - numbers for the numbered windows  
   - This script loads up the ssh commands, so the next step isn't necessary if you use this script to ssh. 
4. ssh into the raspberry pi and jetson nano. Password for RPI is ```turtlebot``` and password for jetson is ```jetson```
   - For the Raspberry Pi (e.g. B04):
   ```ssh ubuntu@192.168.0.204```
   - For the Jetson Nano (e.g. B04:
   ```ssh jetson@192.168.0.230```
5. Run the scripts for both rpi and jetson, and then record the vicon topic for the ground-truth.
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
   - Run the rpi script in the raspberry pi and provide the robot number as the argument, etc. :
     ```
     ./rpi.sh B04
     ./kill_rpi.sh #when you're done recording.
     ```

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
#If the repository is a ROS2 repo
colcon build --packages-select YOUR_REPOSITORY #--symlink-install if it's a Python package
```
- Sometimes in a python package, it may change the permission of your python codes to not be executable (shown in white instead of green when using the ```ls``` command). In that case, add executable permission ```chmod +x <file name>.py``` before doing the colcon build.
- In the above example, we merge with master, but the branch could be ```main``` instead, which you can check with ```git status``` while in the repository

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
sudo apt install python3-pip
sudo pip3 install transforms3d
```
- For the first command, you can ctrl+c when it fetched everything, and for the second and third, you can do it at the line ```Processing triggers for man-db (2.9.1-1) ...```
### Jetson Nano
#### Checking power consumption.
- The Jetson Nano, when powered on the extension pins takes 5V at 3A for each pin, taking in 15W per pin, with 2 pins providing 30W. [reference](https://www.ximea.com/support/wiki/apis/Jetson_Nano_Benchmarks)
- Appropriate power may not be provided, especially with the power demands of the realsense camera and other peripherals (e.g. WiFi dongle).
- To monitor power consumption, install [jtop](https://rnext.it/jetson_stats/).
   ```
   sudo pip3 install -U jetson-stats
   ```
   - If you don't have pip3, type ```sudo apt update``` then ```sudo apt install python3-pip```. 
- Then run jtop to monitor your system:
   ```
   jtop
   ```
   - more options for jtop can be found [here](https://rnext.it/jetson_stats/jtop/jtop.html)
