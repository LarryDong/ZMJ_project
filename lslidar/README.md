
# lslidar driver

lidar_pkg contians 2 lidar launch file and rviz test launch.



## lslidar

codes for Lidar+IMU system.

### When compiling Lidar, need pcap.h
Solution: "sudo apt install libpcap-dev"

if error: "libpcap-dev depends libpcap0.8-dev but it is not going to be installed", remove 0.8 version by:  "sudo apt-get remove libpcap0.8"
  
### When compiling, "Could not find the required component 'pcl_ros'"
Solution: "sudo apt install ros-DISTRIBUTION-pcl-ros"


## Hardware setup
1. connet wires
2. set IPV4. settings-Network-Wired-setting-IPv4, Address: 192.168.1.102, Netmask: 255.255.255.0, -Apply


## Running:
1. sudo chmod +777 /dev/ttyUSB0         // set IMU
2. roslaunch lslidar/my_launch/driver.launch
3. roslaunch lslidar/my_launch/decoder.launch
4. rosrun imu_driver imu_driver_node

Visualize:
5. launch rviz, set "laser_link" and add "pointCloud2"
Record
6. rosbag record -a     // record all data

## Attention:
1. Rotation speed: +- 490 degree/s. Larger than this value, no output so that filter failed.
