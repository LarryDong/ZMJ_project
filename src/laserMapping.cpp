
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>

#include <ros/ros.h>
// #include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
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


double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;
double timeLaserOdometry = 0;



// pointCloud ptr. From last input.
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
// pointCloud ptr. for the whole map
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());


// downsampling for corner/planer
pcl::VoxelGrid<PointType> downSizeFilterCorner, downSizeFilterSurf;



pcl::PointXYZI pointOri, pointSel;
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());
std::vector<int> pointSearchInd;
std::vector<float> pointSearchSqDis;



double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4);

// wmap_T_odom * odom_T_curr = wmap_T_curr;
// transformation between odom's world and map's world frame. wodom is drifted.
Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0);
Eigen::Vector3d t_wmap_wodom(0, 0, 0);

Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0);
Eigen::Vector3d t_wodom_curr(0, 0, 0);



std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::mutex mBuf;


// my pub
ros::Publisher pubCornerFromMap, pubSurfFromMap, pubLaserCloudFullRes;


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
void pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
	po->x = point_curr.x();
	po->y = point_curr.y();
	po->z = point_curr.z();
	po->intensity = pi->intensity;
}




// subscribe messages from corner/surf/full points and odom
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
	// pubOdomAftMappedHighFrec.publish(odomAftMapped); // commented by dongy
}




void process(void){
    ROS_INFO("Main thread of mapping begin.....");

    while(ros::ok()){
        if(cornerLastBuf.empty() || surfLastBuf.empty() || fullResBuf.empty() || odometryBuf.empty()){
            // ROS_INFO_STREAM("Corner size: "<<cornerLastBuf.size());
            // ROS_INFO_STREAM("surfLastBuf size: "<<surfLastBuf.size());
            // ROS_INFO_STREAM("fullResBuf size: "<<fullResBuf.size());
            // ROS_INFO_STREAM("odometryBuf size: "<<odometryBuf.size());
            continue;
        }
            
        mBuf.lock();
        // printf("corner. Time: %f \n", cornerLastBuf.front()->header.stamp.toSec());
        // STEP 1. sync. with cornerLast;
        while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec()){
            // printf("odometryBuf. pop(). Time: %f \n", odometryBuf.front()->header.stamp.toSec());
            odometryBuf.pop();
        }
        if (odometryBuf.empty()){
            mBuf.unlock();
            continue;
        }
        while (!surfLastBuf.empty() && surfLastBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec()){
            // printf("surfLastBuf. pop(). Time: %f \n", surfLastBuf.front()->header.stamp.toSec());
            surfLastBuf.pop();
        }
            
        if (surfLastBuf.empty()){
            mBuf.unlock();
            continue;
        }
        while (!fullResBuf.empty() && fullResBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec()){
            // printf("fullResBuf. pop(). Time: %f \n", fullResBuf.front()->header.stamp.toSec());
            fullResBuf.pop();
        }
        if (fullResBuf.empty()){
            mBuf.unlock();
            continue;
        }

        double timeLaserCloudSurfLast = surfLastBuf.front()->header.stamp.toSec();
        double timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();
        double timeLaserCloudCornerLast = cornerLastBuf.front()->header.stamp.toSec();
        double timeLaserCloudFullRes = fullResBuf.front()->header.stamp.toSec();
        if (timeLaserCloudCornerLast != timeLaserOdometry || timeLaserCloudSurfLast != timeLaserOdometry || timeLaserCloudFullRes != timeLaserOdometry){
            printf("--> time corner %f surf %f full %f odom %f \n", timeLaserCloudCornerLast, timeLaserCloudSurfLast, timeLaserCloudFullRes, timeLaserOdometry);
            printf("unsync messeage!");
            cornerLastBuf.pop();        // added by dongy.
            mBuf.unlock();
            continue; 
        }

        // STEP 2. ROS to PCL, and odom data.
        laserCloudCornerLast->clear();
        pcl::fromROSMsg(*cornerLastBuf.front(), *laserCloudCornerLast);
        cornerLastBuf.pop();

        laserCloudSurfLast->clear();
        pcl::fromROSMsg(*surfLastBuf.front(), *laserCloudSurfLast);
        surfLastBuf.pop();

        laserCloudFullRes->clear();
        pcl::fromROSMsg(*fullResBuf.front(), *laserCloudFullRes);
        fullResBuf.pop();

        q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;
        q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;
        q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;
        q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;
        t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;
        t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;
        t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;
        odometryBuf.pop();

        while(!cornerLastBuf.empty()){      // make sure the real-time
            cornerLastBuf.pop();
            ROS_INFO("drop lidar frame in mapping for real time performance");
        }

        mBuf.unlock();

        // STEP 3. Mapping.
        // STEP 3.1. Extract all features.
        pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());

        // begin my show-time
        *laserCloudCornerFromMap += *laserCloudCornerLast;
        *laserCloudSurfFromMap += *laserCloudSurfLast;

        // ds the full-map
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMap);
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMap);


        // STEP 4. Optimiza.
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerStack);
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfStack);

        if (laserCloudCornerFromMap->size() < 10 || laserCloudSurfFromMap->size() < 50)
            ROS_ERROR("Too little corner/surf points....");

        kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

        for(int iterCount = 0; iterCount<1; ++iterCount){
            
            ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);
            problem.AddParameterBlock(parameters, 4, q_parameterization);
            problem.AddParameterBlock(parameters + 4, 3);

            // STEP 4.1. Add corner points error
            int corner_num = 0, surf_num = 0;
            for(int i=0; i<laserCloudCornerStack->size(); ++i){
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
            
            // STEP 4.2 Add planer points error
            for (int i = 0; i < laserCloudSurfStack->size(); i++){    // 对Planar Points构建误差方程

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
                    }
                    // find the norm of plane 得到这个拟合平面的法向量
                    Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                    double negative_OA_dot_norm = 1 / norm.norm();
                    norm.normalize();

                    // Here n(pa, pb, pc) is unit norm of plane
                    bool planeValid = true;
                    for (int j = 0; j < 5; j++){
                        // if OX * n > 0.2, then plane is not fit well 判断法向量的拟合效果，即拟合原方程ax+by+cz+1=0的效果
                        if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                                    norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                                    norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2){
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
            // STEP 4.3. Optimize all.
            TicToc t_optimize;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            options.check_gradients = false;
            options.gradient_check_relative_precision = 1e-4;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            ROS_INFO_STREAM("Optimize time: " << t_optimize.toc() << ", size: " << corner_num << " / " << surf_num);
        }

        transformUpdate();      // calc. transform between odom and map.

        // STEP 5. Back to ROS
        sensor_msgs::PointCloud2 laserCornerMsg;
        pcl::toROSMsg(*laserCloudCornerFromMap, laserCornerMsg);
        laserCornerMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        laserCornerMsg.header.frame_id = "/laser_link";
        pubCornerFromMap.publish(laserCornerMsg);


        // pub the full map. TODO: corner + surf points.


        // one sweep in map world.
        int laserCloudFullResNum = laserCloudFullRes->points.size();
        for (int i = 0; i < laserCloudFullResNum; i++){
            pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudFullRes3;
        pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        laserCloudFullRes3.header.frame_id = "/laser_link";
        pubLaserCloudFullRes.publish(laserCloudFullRes3);

        // 

    }
}

int main(int argc, char **argv){

	ros::init(argc, argv, "laserMapping");
	ros::NodeHandle nh;

    ROS_WARN("Laser mapping node begin...");

	float lineRes = 0;
	float planeRes = 0;
	nh.param<float>("mapping_line_resolution", lineRes, 0.4); // 对地图特征点云下采样的分辨率
	nh.param<float>("mapping_plane_resolution", planeRes, 0.8);
	// printf("line resolution %f plane resolution %f \n", lineRes, planeRes);
	downSizeFilterCorner.setLeafSize(lineRes, lineRes,lineRes);
	downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);

	// 订阅从odometry前端发来的4个topic消息，分别放在各自的buffer里
	ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100, laserCloudCornerLastHandler);
	ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100, laserCloudSurfLastHandler);
	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, laserOdometryHandler);
	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud_3", 100, laserCloudFullResHandler);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    pubCornerFromMap = nh.advertise<sensor_msgs::PointCloud2>("/corner_from_map", 100);
    pubSurfFromMap = nh.advertise<sensor_msgs::PointCloud2>("/surf_from_map", 100);
    pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_register", 100);

	// pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);
	// pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);
	
	// pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);
	// pubOdomAftMappedHighFrec = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);
	// pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);

	// for (int i = 0; i < laserCloudNum; i++)
	// {
	// 	laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
	// 	laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
	// }

	std::thread mapping_process{process}; // 主要处理线程

	ros::spin();

	return 0;
}