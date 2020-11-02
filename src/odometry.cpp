
#include <iostream>
#include <mutex>
#include <queue>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <pcl_conversions/pcl_conversions.h>    // toROSMsg/fromROSMsg
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>        // PointCloud2
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h> // filter
// #include <pcl/features/normal_3d.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
// #include <pcl/registration/icp_nl.h>
// #include <pcl/registration/transforms.h>

#include <Eigen/Dense>

#include "defination.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> myPC;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

class MyPointRepresentation : public pcl::PointRepresentation<PointNormalT> {
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    MyPointRepresentation(){nr_dimensions_=4;}
    virtual void copyToFloatArray(const PointNormalT &p, float *out) const{
        out[0]=p.x;
        out[1]=p.y;
        out[2]=p.z;
        out[3]=p.curvature;
    }
};

std::queue<sensor_msgs::PointCloud2ConstPtr> allPointsBuf;
std::mutex mBuf;


// double para_q[4] = {0, 0, 0, 1};
// double para_t[3] = {0, 0, 0};

void cleanPointHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloud){
    mBuf.lock();
    allPointsBuf.push(pointCloud);
    mBuf.unlock();
    // ROS_INFO_STREAM("Odom get points. Size: "<<allPointsBuf.size());
}

void pairAlign(const myPC::Ptr pcSrc, const myPC::Ptr pcTar, myPC output, Eigen::Matrix4d &final);

int main(int argc, char **argv){
    ROS_INFO("In odometry node...");
    ros::init(argc, argv, "odometry");
    
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/cleanPointCloud", 100, cleanPointHandler);
    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);
    nav_msgs::Path laserPath;

    // ros::Rate rate(1000);

    bool is_inited = false;

    Eigen::Quaterniond q_w_curr(1, 0, 0, 0), q_last_curr;
    Eigen::Vector3d t_w_curr(0, 0, 0), t_last_curr;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr ptNew(new pcl::PointCloud<pcl::PointXYZ>()), ptOld(new pcl::PointCloud<pcl::PointXYZ>());
    myPC::Ptr ptNew(new myPC()), ptOld(new myPC());

    while(ros::ok()){
        ros::spinOnce();

        if(allPointsBuf.empty()){
            // ROS_WARN("Empty buf...");
            continue;
        }

        if(!is_inited){         // initialize
            q_last_curr = Eigen::Quaterniond(1, 0, 0, 0);
            t_w_curr = Eigen::Vector3d(0, 0, 0);
            mBuf.lock();
            ptNew->clear();
            pcl::fromROSMsg(*allPointsBuf.front(), *ptNew);
            allPointsBuf.pop();
            mBuf.unlock();
            is_inited = true;
            continue;
        }


        // ptOld = ptNew;      // update point cloud;
        ptOld.swap(ptNew);
        // ROS_WARN_STREAM("Target: " << ptOld->size());
        mBuf.lock();
        ptNew->clear();
        pcl::fromROSMsg(*allPointsBuf.front(), *ptNew);
        allPointsBuf.pop();
        mBuf.unlock();

        // calculate ICP;
        myPC pcOutput;
        Eigen::Matrix4d T;
        ROS_INFO("Begin NDT...");
        pairAlign(ptNew, ptOld, pcOutput, T);
        ROS_INFO("End NDT...");
        Eigen::Matrix3d tmpT = T.topLeftCorner(3,3);
        q_last_curr = tmpT;
        t_last_curr = T.topRightCorner(3,1);
        t_w_curr = t_w_curr + q_w_curr * t_last_curr;
        q_w_curr = q_w_curr * q_last_curr;
        ROS_INFO_STREAM("trace: \n"<<t_w_curr);
    }

}


void pairAlign(const myPC::Ptr pcSrc, const myPC::Ptr pcTgt, myPC output, Eigen::Matrix4d &T){
    
    
    // down sampling
    myPC::Ptr src(new myPC), tgt(new myPC);
    pcl::VoxelGrid<PointT> grid;
    double s = 0.2;
    grid.setLeafSize(s, s, s);
    grid.setInputCloud(pcSrc);
    grid.filter(*src);
    grid.setInputCloud(pcTgt);
    grid.filter(*tgt);
    ROS_INFO_STREAM("Filter. src: " << pcSrc->size() << "/" << src->size() << ", tgt: " << pcTgt->size() << "/" << tgt->size());

    // NDT
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(0.1); // termination condition
    ndt.setStepSize(1);               // maximum size for More-Thuente line search
    ndt.setResolution(0.5);             // resolution for NDT grid 

    ndt.setMaximumIterations(10);
    ndt.setInputSource(src);
    ndt.setInputTarget(tgt);
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ROS_WARN("Align begin...");
    ndt.align(*output_cloud, init_guess);
    ROS_WARN("Align end...");
    T = ndt.getFinalTransformation().cast<double>();

#if 0
    // normal and curve
    PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);
    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(30);
    norm_est.setInputCloud(src);
    norm_est.compute(*points_with_normals_src);
    pcl::copyPointCloud(*src, *points_with_normals_src);
    norm_est.setInputCloud(tgt);
    norm_est.compute(*points_with_normals_tgt);
    pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

    MyPointRepresentation point_representation;
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};      // weights
    point_representation.setRescaleValues(alpha);

    // registration
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon(0.01);
    reg.setMaxCorrespondenceDistance(0.1);
    reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));
    reg.setInputCloud(points_with_normals_src);
    reg.setInputTarget(points_with_normals_tgt);

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity(), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations(10);
    // for(int i=0; i<30; ++i){
    //     ROS_INFO_STREAM("Iter: " << i);
    //     points_with_normals_src = reg_result;
    //     reg.setInputCloud(points_with_normals_src);
    //     reg.align(*reg_result);
    //     T = reg.getFinalTransformation() * T;
    //     if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
    //         reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);
    //     prev = reg.getLastIncrementalTransformation();
    // }
    reg.align(*reg_result);
    T = reg.getFinalTransformation() * T;
    targetToSource = T.inverse();
    Transform = targetToSource.cast<double>();
    // // ICP
    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setInputSource(src);
    // icp.setInputTarget(tgt);
    // // ROS_INFO_STREAM("ICP size: "<<tgt->size());
    // pcl::PointCloud<pcl::PointXYZ> ptFinal;
    // icp.align(ptFinal);
    // // ROS_INFO_STREAM("ICP converged: " << icp.hasConverged() << ", score: " << icp.getFitnessScore());
    // Eigen::Matrix4f tmp = icp.getFinalTransformation();
    // T = tmp.cast<double>();
    // ROS_INFO_STREAM("Results: --------\n" << T);
#endif

    return ;
}

