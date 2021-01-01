
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

#include "lidarFactor.hpp"
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"

// debug parameters
double gt_prepareMap = 0, gt_dataAssociate = 0, gt_mapSolver = 0, gt_optimize = 0, gt_addPoint = 0, gt_ds = 0, gt_pub = 0, gt_whole = 0;

int frameCount = 0;

double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;
double timeLaserOdometry = 0;

// map子图的大小，cube的数量，即x,y,z方向各有21,21,11个cube
int laserCloudCenWidth = 10;
int laserCloudCenHeight = 10;
int laserCloudCenDepth = 5;
const int laserCloudWidth = 21;
const int laserCloudHeight = 21;
const int laserCloudDepth = 11;
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; //4851
int laserCloudValidInd[125]; // 表示5*5*3cube范围内的点云index
int laserCloudSurroundInd[125];

// input: from odom
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());

// surround points in map to build tree
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());

// points in every cube
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];

//kd-tree
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4);

// wmap_T_odom * odom_T_curr = wmap_T_curr;
// transformation between odom's world and map's world frame
Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0);
Eigen::Vector3d t_wmap_wodom(0, 0, 0);

Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0);
Eigen::Vector3d t_wodom_curr(0, 0, 0);

std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::mutex mBuf;

pcl::VoxelGrid<PointType> downSizeFilterCorner;
pcl::VoxelGrid<PointType> downSizeFilterSurf;

std::vector<int> pointSearchInd;
std::vector<float> pointSearchSqDis;

PointType pointOri, pointSel;

ros::Publisher pubLaserCloudMap, pubLaserCloudFullRes, pubOdomAftMapped, pubOdomAftMappedHighFrec, pubLaserAfterMappedPath;
nav_msgs::Path laserAfterMappedPath;

// set initial guess
void transformAssociateToMap(){
	q_w_curr = q_wmap_wodom * q_wodom_curr;
	t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
}
void transformUpdate(){
	q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
	t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
}


void pointAssociateToMap(PointType const *const pi, PointType *const po){ // 转换到地图坐标系下
	Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
	po->x = point_w.x();
	po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
	//po->intensity = 1.0;
}
void pointAssociateTobeMapped(PointType const *const pi, PointType *const po){
	Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
	po->x = point_curr.x();
	po->y = point_curr.y();
	po->z = point_curr.z();
	po->intensity = pi->intensity;
}

void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2){
	mBuf.lock();
	cornerLastBuf.push(laserCloudCornerLast2);
	mBuf.unlock();
}
void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudSurfLast2){
	mBuf.lock();
	surfLastBuf.push(laserCloudSurfLast2);
	mBuf.unlock();
}
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2){
	mBuf.lock();
	fullResBuf.push(laserCloudFullRes2);
	mBuf.unlock();
}

//receive odomtry
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry){
	mBuf.lock();
	odometryBuf.push(laserOdometry);
	mBuf.unlock();

	// high frequence publish
	Eigen::Quaterniond q_wodom_curr;
	Eigen::Vector3d t_wodom_curr;
	q_wodom_curr.x() = laserOdometry->pose.pose.orientation.x;
	q_wodom_curr.y() = laserOdometry->pose.pose.orientation.y;
	q_wodom_curr.z() = laserOdometry->pose.pose.orientation.z;
	q_wodom_curr.w() = laserOdometry->pose.pose.orientation.w;
	t_wodom_curr.x() = laserOdometry->pose.pose.position.x;
	t_wodom_curr.y() = laserOdometry->pose.pose.position.y;
	t_wodom_curr.z() = laserOdometry->pose.pose.position.z;

	Eigen::Quaterniond q_w_curr = q_wmap_wodom * q_wodom_curr;
	Eigen::Vector3d t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom; 

	nav_msgs::Odometry odomAftMapped;
	odomAftMapped.header.frame_id = "/laser_link";
	odomAftMapped.child_frame_id = "/aft_mapped";
	odomAftMapped.header.stamp = laserOdometry->header.stamp;
	odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
	odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
	odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
	odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
	odomAftMapped.pose.pose.position.x = t_w_curr.x();
	odomAftMapped.pose.pose.position.y = t_w_curr.y();
	odomAftMapped.pose.pose.position.z = t_w_curr.z();
	pubOdomAftMappedHighFrec.publish(odomAftMapped);
}



// 主线程
void process() {

	ROS_WARN("--> Mapping node begin...");
	while(1){

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);

        mBuf.lock();            // Bug fixed. Lock before.
        if (cornerLastBuf.empty() || surfLastBuf.empty() || fullResBuf.empty() ||odometryBuf.empty()){
            mBuf.unlock();
            continue;
        }
        else
            mBuf.unlock();

        mBuf.lock();
        while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
            odometryBuf.pop();
        if (odometryBuf.empty()){
            mBuf.unlock();
            ROS_ERROR("Odom break;");
            continue;           // Bug fixed. continue not break;
        }

        while (!surfLastBuf.empty() && surfLastBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
            surfLastBuf.pop();
        if (surfLastBuf.empty()){
            mBuf.unlock();
            ROS_ERROR("Surf break;");
            continue;
        }

        while (!fullResBuf.empty() && fullResBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
            fullResBuf.pop();
        if (fullResBuf.empty()){
            mBuf.unlock();
            ROS_ERROR("Full break;");
            continue;
        }

        timeLaserCloudCornerLast = cornerLastBuf.front()->header.stamp.toSec();
        timeLaserCloudSurfLast = surfLastBuf.front()->header.stamp.toSec();
        timeLaserCloudFullRes = fullResBuf.front()->header.stamp.toSec();
        timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();
        
        if (timeLaserCloudCornerLast != timeLaserOdometry || timeLaserCloudSurfLast != timeLaserOdometry ||timeLaserCloudFullRes != timeLaserOdometry){
            printf("odom time: %f\n", timeLaserOdometry);
            printf("time corner %f surf %f full %f odom %f \n", timeLaserCloudCornerLast, timeLaserCloudSurfLast, timeLaserCloudFullRes, timeLaserOdometry);
            printf("unsync messeage!!! --mapping\n");
            ROS_ERROR("unsync messeage --mapping!!!");
            cornerLastBuf.pop();
            mBuf.unlock();
            // break;
            continue;
        }

        // ros消息转换成pcl消息
        laserCloudCornerLast->clear();
        pcl::fromROSMsg(*cornerLastBuf.front(), *laserCloudCornerLast);
        cornerLastBuf.pop();

        laserCloudSurfLast->clear();
        pcl::fromROSMsg(*surfLastBuf.front(), *laserCloudSurfLast);
        surfLastBuf.pop();

        laserCloudFullRes->clear();
        pcl::fromROSMsg(*fullResBuf.front(), *laserCloudFullRes);
        fullResBuf.pop();

        // 记录odometry前端线程传过来的位姿
        q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;
        q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;
        q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;
        q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;
        t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;
        t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;
        t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;
        odometryBuf.pop();

        while(!cornerLastBuf.empty()){
            cornerLastBuf.pop();
            printf("drop lidar frame in mapping for real time performance \n");
        }

        mBuf.unlock();

        TicToc t_whole;

        transformAssociateToMap(); // 把前端传过来的odom位姿转换为地图坐标系下的位姿
        // 构造子图，方便接下来特征和子图进行匹配，子图的中点为当前位姿
        TicToc t_shift;
        int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth; // 子图中间的cube，I,J,K相当于是x,y,z
        int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;
        int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;

        if (t_w_curr.x() + 25.0 < 0)
            centerCubeI--;
        if (t_w_curr.y() + 25.0 < 0)
            centerCubeJ--;
        if (t_w_curr.z() + 25.0 < 0)
            centerCubeK--;


{       // cube process
        // 如果centerCube在整个子图的边缘（接近边缘3个cube），则将整个laserCloudCornerArray的内容平移
        while (centerCubeI < 3){
            for (int j = 0; j < laserCloudHeight; j++){
                for (int k = 0; k < laserCloudDepth; k++){ 
					int i = laserCloudWidth - 1;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]; 
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    for (; i >= 1; i--){
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    }
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudCubeCornerPointer;
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            centerCubeI++;
            laserCloudCenWidth++;
        }

        while (centerCubeI >= laserCloudWidth - 3){ 
            for (int j = 0; j < laserCloudHeight; j++){
                for (int k = 0; k < laserCloudDepth; k++){
                    int i = 0;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    for (; i < laserCloudWidth - 1; i++){
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
					}
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudCubeCornerPointer;
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            centerCubeI--;
            laserCloudCenWidth--;
        }

        while (centerCubeJ < 3){
            for (int i = 0; i < laserCloudWidth; i++){
                for (int k = 0; k < laserCloudDepth; k++){
                    int j = laserCloudHeight - 1;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    for (; j >= 1; j--){
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
                    }
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudCubeCornerPointer;
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            centerCubeJ++;
            laserCloudCenHeight++;
        }

        while (centerCubeJ >= laserCloudHeight - 3){
            for (int i = 0; i < laserCloudWidth; i++){
                for (int k = 0; k < laserCloudDepth; k++){
                    int j = 0;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    for (; j < laserCloudHeight - 1; j++){
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
                    }
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudCubeCornerPointer;
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            centerCubeJ--;
            laserCloudCenHeight--;
        }

        while (centerCubeK < 3){
            for (int i = 0; i < laserCloudWidth; i++){
                for (int j = 0; j < laserCloudHeight; j++){
                    int k = laserCloudDepth - 1;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    for (; k >= 1; k--){
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
                    }
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudCubeCornerPointer;
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }

            centerCubeK++;
            laserCloudCenDepth++;
        }

        while (centerCubeK >= laserCloudDepth - 3){
            for (int i = 0; i < laserCloudWidth; i++){
                for (int j = 0; j < laserCloudHeight; j++){
                    int k = 0;
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                    for (; k < laserCloudDepth - 1; k++){
                        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
                    }
                    laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudCubeCornerPointer;
                    laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =laserCloudCubeSurfPointer;
                    laserCloudCubeCornerPointer->clear();
                    laserCloudCubeSurfPointer->clear();
                }
            }
            centerCubeK--;
            laserCloudCenDepth--;
        }
}       // end cube process

        int laserCloudValidNum = 0;
        int laserCloudSurroundNum = 0;
        for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++){
            for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++){
                for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++){
                    if (i >= 0 && i < laserCloudWidth &&j >= 0 && j < laserCloudHeight &&k >= 0 && k < laserCloudDepth){ 
                        // 地图中心附近点的index
                        laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                        laserCloudValidNum++;
                        laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                        laserCloudSurroundNum++;
                    }
                }
            }
        }

        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        for (int i = 0; i < laserCloudValidNum; i++){
            *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]]; // 中心cude附近的激光特征
            *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
        }
        int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
        int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

        // 进行下采样，下采样的分辨率可以在launch文件中设置
        pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerStack);
        int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

        pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfStack);
        int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

        // 到此整个子图构建结束，接下来计算特征和子图的匹配
		// ROS_INFO_STREAM("Map prepare time: " << t_shift.toc() << " ms, corner/surf: " << laserCloudCornerFromMapNum << "/" << laserCloudSurfFromMapNum);
		gt_prepareMap = t_shift.toc();
		
        if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50){ // 中心cude附近的激光特征点大于一定数量
            TicToc t_opt;
            TicToc t_tree;
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap); // 设置kd树
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
            // printf("build tree time %f ms \n", t_tree.toc());

            for (int iterCount = 0; iterCount < 2; iterCount++){ // 跟odometry前端线程类似，进行2次大循环迭代优化
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
                ceres::Problem::Options problem_options;

                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(parameters, 4, q_parameterization);
                problem.AddParameterBlock(parameters + 4, 3);

                TicToc t_data;
                int corner_num = 0;

                for (int i = 0; i < laserCloudCornerStackNum; i++){ // 对Edgepoints构建误差方程
                    pointOri = laserCloudCornerStack->points[i]; // 附近的所有Edgepoints
                    pointAssociateToMap(&pointOri, &pointSel); // 将当前点从激光雷达坐标系转到世界（map）坐标系下
                    kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis); // 找到最近的5个相应特征点

                    if (pointSearchSqDis[4] < 1.0){ // 最近的五个点均小于1m
                        std::vector<Eigen::Vector3d> nearCorners;
                        Eigen::Vector3d center(0, 0, 0);
                        for (int j = 0; j < 5; j++){
							auto pt = laserCloudCornerFromMap->points[pointSearchInd[j]];
                            Eigen::Vector3d tmp(pt.x, pt.y, pt.z);
                            center = center + tmp;
                            nearCorners.push_back(tmp);
                        }
                        center = center / 5.0; // 5个点的质心

                        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                        for (int j = 0; j < 5; j++){
                            Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center; // 坐标去质心
                            covMat = covMat + tmpZeroMean * tmpZeroMean.transpose(); // 这5个点的协方差
                        }

                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

                        // if is indeed line feature
                        // note Eigen library sort eigenvalues in increasing order
                        Eigen::Vector3d unit_direction = saes.eigenvectors().col(2); // 这些edge points的主方向向量
                        Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]){ // 这里相当于判断这些点的分布足够成线状分布
                            Eigen::Vector3d point_on_line = center;
                            Eigen::Vector3d point_a, point_b;
                            point_a = 0.1 * unit_direction + point_on_line; // 相当于拟合出了这条线的两个端点
                            point_b = -0.1 * unit_direction + point_on_line;
                            // 计算点到直线距离作为误差函数，这里没有运动畸变，不需要插值
                            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
                            problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                            corner_num++;	
                        }							
                    }
                }

                int surf_num = 0;
                for (int i = 0; i < laserCloudSurfStackNum; i++){ // 对Planar Points构建误差方程
                    pointOri = laserCloudSurfStack->points[i];
                    // 同样先将特征点转换到世界坐标系下，然后找5个最近点
                    pointAssociateToMap(&pointOri, &pointSel);
                    kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                    Eigen::Matrix<double, 5, 3> matA0;
                    Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
                    if (pointSearchSqDis[4] < 1.0){
                        // 设平面方程为ax+by+cz+d=0，取d=1，则（a,b,c）为平面法向量
                        for (int j = 0; j < 5; j++){
                            matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
                            matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
                            matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
                            //printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
                        }
                        // find the norm of plane 得到这个拟合平面的法向量
                        Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                        double negative_OA_dot_norm = 1 / norm.norm();
                        norm.normalize();

                        // Here n(pa, pb, pc) is unit norm of plane
                        bool planeValid = true;
                        for (int j = 0; j < 5; j++){
                            // if OX * n > 0.2, then plane is not fit well 判断法向量的拟合效果，即拟合原方程ax+by+cz+1=0的效果
							auto pt = laserCloudSurfFromMap->points[pointSearchInd[j]];
							if (fabs(norm(0) * pt.x + norm(1) * pt.y + norm(2) * pt.z + negative_OA_dot_norm) > 0.2){
								planeValid = false;
                                break;
							}
						}
                        Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
                        if (planeValid){
                            ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
                            problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
                            surf_num++;
                        }
                    }
                }
				gt_dataAssociate = t_data.toc();

                TicToc t_solver;
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
                options.minimizer_progress_to_stdout = false;
                options.check_gradients = false;
                options.gradient_check_relative_precision = 1e-4;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
                gt_mapSolver = t_solver.toc();
            }
			gt_optimize = t_opt.toc();
		}
        else
            ROS_WARN("time Map corner and surf num are not enough");
        transformUpdate(); // 计算odom和map之间的转换关系

        TicToc t_add;
        for (int i = 0; i < laserCloudCornerStackNum; i++){ // 将这一帧的特征点加入到地图当中
            pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);
            int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
            int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
            int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;
            if (pointSel.x + 25.0 < 0)
                cubeI--;
            if (pointSel.y + 25.0 < 0)
                cubeJ--;
            if (pointSel.z + 25.0 < 0)
                cubeK--;

            if (cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 && cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth){
                int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                laserCloudCornerArray[cubeInd]->push_back(pointSel);
            }
        }

        for (int i = 0; i < laserCloudSurfStackNum; i++){
            pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);
            int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
            int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
            int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;
            if (pointSel.x + 25.0 < 0)
                cubeI--;
            if (pointSel.y + 25.0 < 0)
                cubeJ--;
            if (pointSel.z + 25.0 < 0)
                cubeK--;
            if (cubeI >= 0 && cubeI < laserCloudWidth &&cubeJ >= 0 && cubeJ < laserCloudHeight &&cubeK >= 0 && cubeK < laserCloudDepth){
                int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                laserCloudSurfArray[cubeInd]->push_back(pointSel);
            }
        }
        // printf("add points time %f ms\n", t_add.toc());
		gt_addPoint = t_add.toc();

		TicToc t_filter; // 加完新特征点后对地图里的特征点进行下采样
        for (int i = 0; i < laserCloudValidNum; i++){
            int ind = laserCloudValidInd[i];

            pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
            downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
            downSizeFilterCorner.filter(*tmpCorner);
            laserCloudCornerArray[ind] = tmpCorner;

            pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
            downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
            downSizeFilterSurf.filter(*tmpSurf);
            laserCloudSurfArray[ind] = tmpSurf;
        }
        // printf("filter time %f ms \n", t_filter.toc());
        gt_ds = t_filter.toc();

        TicToc t_pub;
        if (frameCount % 2 == 0){ // 发布地图消息，大地图
            pcl::PointCloud<PointType> laserCloudMap;
            for (int i = 0; i < laserCloudNum; i++){
                laserCloudMap += *laserCloudCornerArray[i];
                laserCloudMap += *laserCloudSurfArray[i];
            }
            sensor_msgs::PointCloud2 laserCloudMsg;
            pcl::toROSMsg(laserCloudMap, laserCloudMsg);
            laserCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            laserCloudMsg.header.frame_id = "/laser_link";
            pubLaserCloudMap.publish(laserCloudMsg);
        }

        // using all points for Map.
        // 将激光点转换到世界坐标系下并发布，可视化用
        int laserCloudFullResNum = laserCloudFullRes->points.size();
        for (int i = 0; i < laserCloudFullResNum; i++){
            pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudFullRes3;
        pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        laserCloudFullRes3.header.frame_id = "/laser_link";
        pubLaserCloudFullRes.publish(laserCloudFullRes3);

        nav_msgs::Odometry odomAftMapped;
        odomAftMapped.header.frame_id = "/laser_link";
        odomAftMapped.child_frame_id = "/aft_mapped";
        odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
        odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
        odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
        odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
        odomAftMapped.pose.pose.position.x = t_w_curr.x();
        odomAftMapped.pose.pose.position.y = t_w_curr.y();
        odomAftMapped.pose.pose.position.z = t_w_curr.z();
        pubOdomAftMapped.publish(odomAftMapped); // 发布地图坐标系下的位姿

        geometry_msgs::PoseStamped laserAfterMappedPose;
        laserAfterMappedPose.header = odomAftMapped.header;
        laserAfterMappedPose.pose = odomAftMapped.pose.pose;
        laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
        laserAfterMappedPath.header.frame_id = "/laser_link";
        laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
        pubLaserAfterMappedPath.publish(laserAfterMappedPath); // 发布地图坐标系下的位姿组成的路径

        static tf::TransformBroadcaster br; // 发布tf
        tf::Transform transform;
        tf::Quaternion q;
        transform.setOrigin(tf::Vector3(t_w_curr(0),t_w_curr(1),t_w_curr(2)));
        q.setW(q_w_curr.w());
        q.setX(q_w_curr.x());
        q.setY(q_w_curr.y());
        q.setZ(q_w_curr.z());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "/laser_link", "/aft_mapped"));

        frameCount++;
		gt_pub = t_pub.toc();
		gt_whole = t_whole.toc();
		// gt_prepareMap = 0, gt_dataAssociate = 0, gt_mapSolver = 0, gt_optimize = 0, gt_addPoint = 0, gt_ds = 0, gt_pub = 0, gt_whole = 0;
		// ROS_INFO_STREAM("Total: " << gt_whole << ", prepare: " << gt_prepareMap << ", DA: " << gt_dataAssociate << ", opti: " << gt_optimize);
        std::cout << "[Map ] " << gt_whole << "ms, prepare: " << gt_prepareMap << ", DA: " << gt_dataAssociate << ", opti: " << gt_optimize << std::endl;
    }
}

int main(int argc, char **argv){

	ros::init(argc, argv, "laserMapping");
	ros::NodeHandle nh;

	float lineRes = 0;
	float planeRes = 0;
	nh.param<float>("mapping_line_resolution", lineRes, 0.4); // 对地图特征点云下采样的分辨率
	nh.param<float>("mapping_plane_resolution", planeRes, 0.8);
	printf("line resolution %f plane resolution %f \n", lineRes, planeRes);
	downSizeFilterCorner.setLeafSize(lineRes, lineRes,lineRes);
	downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);
	// 订阅从odometry前端发来的4个topic消息，分别放在各自的buffer里
	ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100, laserCloudCornerLastHandler);
    ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100, laserCloudSurfLastHandler);
	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, laserOdometryHandler);
	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud_3", 100, laserCloudFullResHandler);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);
	pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);
	pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/lslidar_point_cloud_registered", 100);
	pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);
	pubOdomAftMappedHighFrec = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);
	pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);
	for (int i = 0; i < laserCloudNum; i++){
		laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
		laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
	}
	std::thread mapping_process{process}; // 主要处理线程
	ros::spin();

	return 0;
}