
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Dense>

#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
// #include <string>
// #include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pubLaserCloudMap, pubDebug;
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZ>()), fullPointCloud(new pcl::PointCloud<pcl::PointXYZ>());
Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0);
Eigen::Vector3d t_wodom_curr(0, 0, 0);

// std::queue<sensor_msgs::PointCloud2ConstPtr> 
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::mutex mBuf;

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry){
	mBuf.lock();
	odometryBuf.push(laserOdometry);
	mBuf.unlock();
    // ROS_INFO("IIn laserOdometry handler...");
}

void fullPointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloud){
    mBuf.lock();
    fullResBuf.push(pointCloud);
    mBuf.unlock();
}

void process(void){
    ROS_WARN("Mapping process begin...");
    fullPointCloud->header.frame_id="/laser_link";

    while(1){

        mBuf.lock();

        // STEP I. Sync. the clock.
        if (odometryBuf.empty() || fullResBuf.empty()){
            mBuf.unlock();
            continue;
        }
        // sync the fullPointsCloud and odom
        // fullResBuf should be earlier than odom. Since odom data passed odomtry_node
        while(!fullResBuf.empty() && fullResBuf.front()->header.stamp.toSec() < odometryBuf.front()->header.stamp.toSec()){
            fullResBuf.pop();
        }
        if (fullResBuf.empty()){
            mBuf.unlock();
            continue;
        }
        // but sometimes odom is faster.
        while(!odometryBuf.empty() && fullResBuf.front()->header.stamp.toSec() > odometryBuf.front()->header.stamp.toSec()){
            odometryBuf.pop();
        }
        if(odometryBuf.empty()){
            mBuf.unlock();
            continue;
        }
        // TODO: maybe sometims both buf are popped. Unreasonable.
        double odomTime = odometryBuf.front()->header.stamp.toSec();
        double fullResTime = fullResBuf.front()->header.stamp.toSec();
        if(odomTime != fullResTime){
            ROS_ERROR("Time un-sync. odom: %.10f, %.10f", odomTime, fullResTime);
            mBuf.unlock();
            continue;
        }

        // STEP II. Extract data from odometry_node
        // to Ros message
        laserCloudFullRes->clear();
        pcl::fromROSMsg(*fullResBuf.front(), *laserCloudFullRes);
        fullResBuf.pop();
        // record pose from odometry_node
        auto op = odometryBuf.front()->pose.pose;
        q_wodom_curr.x() = op.orientation.x;
        q_wodom_curr.y() = op.orientation.y;
        q_wodom_curr.z() = op.orientation.z;
        q_wodom_curr.w() = op.orientation.w;
        t_wodom_curr.x() = op.position.x;
        t_wodom_curr.y() = op.position.y;
        t_wodom_curr.z() = op.position.z;
        odometryBuf.pop();

        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.topLeftCorner<3, 3>() = q_wodom_curr.toRotationMatrix();
        transform.topRightCorner<3, 1>() = t_wodom_curr;

        ROS_INFO_STREAM("Transform: " << transform);
        pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudFullResOut(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*laserCloudFullRes, *laserCloudFullResOut, transform);

        // debug
        sensor_msgs::PointCloud2 pc1;
        pcl::toROSMsg(*laserCloudFullResOut, pc1);
        pubDebug.publish(pc1);

        *fullPointCloud = *fullPointCloud + *laserCloudFullResOut;
        // grid filter
        pcl::VoxelGrid<pcl::PointXYZ> grid;
        double s = 0.2;
        grid.setLeafSize(s, s, s);
        grid.setInputCloud(fullPointCloud);
        grid.filter(*fullPointCloud);

        // pass
        pcl::PassThrough<pcl::PointXYZ> pass;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInPtr = fullPointCloud->makeShared();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutPtr(new pcl::PointCloud<pcl::PointXYZ>);
        pass.setInputCloud(cloudInPtr);
        pass.setFilterFieldName("x"); // x-axis [-3, 3];
        pass.setFilterLimits(-5.0, 5.0);
        pass.filter(*cloudOutPtr);
        pass.setInputCloud(cloudOutPtr);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-5.0, 5.0);
        pass.filter(*cloudOutPtr);

        // back to ROS
        sensor_msgs::PointCloud2 pc2;
        pcl::toROSMsg(*fullPointCloud, pc2);
        pubLaserCloudMap.publish(pc2);


        mBuf.unlock();

        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "mapping");
    ros::NodeHandle nh;

	float lineRes = 0;
	float planeRes = 0;

	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, laserOdometryHandler);
    ros::Subscriber subLaserFullPoints = nh.subscribe<sensor_msgs::PointCloud2>("/fullPointCloud", 100, fullPointCloudHandler);
	// ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100, laserCloudFullResHandler);
    pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_mapping", 100);
    pubDebug = nh.advertise<sensor_msgs::PointCloud2>("/transformed_laser", 100);
    std::thread mapping_process{process};

    ros::spin();
    return 0;
}

