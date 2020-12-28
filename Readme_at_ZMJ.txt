
source devel/setup.bash: sos

launch two lidar: roslaunch lidar_pkg both.launch
launch slam: roslaunch slam all.launch
launch reconstruct: roslaunch reconstruct verLidar.launch
show reconstruct: roslaunch reconstruct rviz

launch imu: rosrun imu_driver imu_driver_node

record data: rosbag record -a

save pointcloud: rosrun reconstruct my_debug, 
then pub&save: rostopic pub /debug std_msgs/String "s"
then show icp: rosrun reconstruct my_test