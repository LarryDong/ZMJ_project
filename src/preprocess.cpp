
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>    // toROSMsg/fromROSMsg
#include <pcl/point_cloud.h>        // PointCloud2
#include <pcl/filters/voxel_grid.h> // filter
#include <pcl/filters/passthrough.h>

using namespace std;


ros::Publisher pubSelectedCloud, pubFullCloud;

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
    // ROS_INFO("In laser pointCloud handler");

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    std::vector<int> indices;

    // Debugs
    int sizeIn, sizeNan, sizeRange, sizeSampling;
    // 1. remove invalid points.
    sizeIn = laserCloudIn.size();
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    sizeNan = laserCloudIn.size();

    sensor_msgs::PointCloud2 fullCloudOutMsg;
    pcl::toROSMsg(laserCloudIn, fullCloudOutMsg);
    pubFullCloud.publish(fullCloudOutMsg);     // pub full cloud (without NaN)

    // 2. remove too far/near points
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInPtr = laserCloudIn.makeShared();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud(cloudInPtr);
    pass.setFilterFieldName("x");           // x-axis [-3, 3];
    pass.setFilterLimits(-3.0, 3.0);
    pass.filter(*cloudOutPtr);
    pass.setInputCloud(cloudOutPtr);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-3.0, 3.0);
    pass.filter(*cloudOutPtr);
    sizeRange = cloudOutPtr->size();

    // 3. downsampling
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    double s = 0.1;
    grid.setLeafSize(s, s, s);
    grid.setInputCloud(cloudOutPtr);
    grid.filter(*cloudOutPtr);
    sizeSampling = cloudOutPtr->size();

    ROS_WARN_STREAM("Filter: in: " << sizeIn << ", nan: " << sizeNan << ", range: " << sizeRange << ", sampling: " << sizeSampling);

    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*cloudOutPtr, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "/laser_link";
    pubSelectedCloud.publish(laserCloudOutMsg);

    // check time stamp
    if (fullCloudOutMsg.header.stamp != laserCloudOutMsg.header.stamp)
        ROS_ERROR("Time stamp not same!");
}


int main(int argc, char** argv){
	ros::init(argc, argv, "preprocess");
    ROS_WARN("SLAM node begin...");

    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud", 100, laserCloudHandler);
    pubSelectedCloud = nh.advertise<sensor_msgs::PointCloud2>("/cleanPointCloud", 100);
    pubFullCloud = nh.advertise<sensor_msgs::PointCloud2>("/fullPointCloud", 100);

	ros::spin();
	return 0;
}

