
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

using namespace std;
bool g_showCarPath = false;



void carPathHandler(const sensor_msgs::PointCloud2ConstPtr &pc){
	// pcl::fromROSMsg(*pc, g_verPointCloud); // 将传入的ros消息格式转为pcl库里的点云格式
    cout << "Car path received" << endl;
}


int main(int argc, char **argv){

    // google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "show_rviz");
    ros::NodeHandle nh;
    ROS_WARN("show_rviz.");


    // pubCarPath.publish(tool::pointCloud2RosMsg(car_path.getFullPointCloud()));
    ros::Subscriber subCarPath = nh.subscribe<sensor_msgs::PointCloud2>("/analyze/car_path", 100, carPathHandler);

    while(ros::ok()){

    }
}