
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

using namespace std;

typedef pcl::PointXYZ MyPoint;
typedef pcl::PointCloud<pcl::PointXYZ> MyPointCloud;



int main(int argc, char **argv){

    // google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "show_rviz");
    ros::NodeHandle nh;
    ROS_WARN("show_rviz.");

    // pubCarPath.publish(tool::pointCloud2RosMsg(car_path.getFullPointCloud()));
    // ros::Subscriber subCarPath = nh.subscribe<sensor_msgs::PointCloud2>("/analyze/car_path", 100, carPathHandler);

    while(ros::ok()){

    }
}