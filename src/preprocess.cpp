
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>    // toROSMsg/fromROSMsg
#include <pcl/point_cloud.h>        // PointCloud2
#include <pcl/filters/voxel_grid.h> // filter

using namespace std;


ros::Publisher pubLaserCloud;

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
    // ROS_INFO("In laser pointCloud handler");

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);

    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(laserCloudIn, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "/camera_init";
    pubLaserCloud.publish(laserCloudOutMsg);
    ROS_INFO("Pub points...");
}


int main(int argc, char** argv){
	ros::init(argc, argv, "preprocess");
    ROS_WARN("SLAM node begin...");

    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud", 100, laserCloudHandler);
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/cleanPointCloud", 100);

	ros::spin();
	return 0;
}

