
#pragma once

#ifndef CAR_PATH_H
#define CAR_PATH_H


#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>


using namespace std;

class CarPath{

public:
    CarPath() { cout << "[Error]. Not allowed empty input."; }
    CarPath(ros::NodeHandle &nh, string filename);
    void pub(void);
    void pubOld(void);
    double getClosestPointInPath(const pcl::PointXYZ& in, pcl::PointXYZ& out);

public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_, pc_ori_;

// private:
    void smooth(void);
    ros::Publisher pubCloud_, pubCloud_ori_;
    ros::NodeHandle nh_;
};

#endif
