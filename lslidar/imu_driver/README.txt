
# IMU driver bag.

### Process
1. Enable USB: sudo chmod +777 /dev/ttyUSB0
2. Run: rosrun imu_driver imu_driver_node
3. Record: rosbag record -a


### Attention
Rotation speed: +- 490 degree/s. Larger than this value, no output so that filter failed.
