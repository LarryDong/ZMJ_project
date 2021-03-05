
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

#include "process/tool.h"

using namespace std;

class CarPath{

public:
    CarPath() { cout << "[Error]. Not allowed empty input."; }
    CarPath(ros::NodeHandle &nh, string filename);
    void pub(void);
    void pubOld(void);
    double getClosestPointInPath(const MyPoint& in, MyPoint& out);
    inline MyPoint getBeginPoint(void) const { assert(!pc_->empty()); return (*pc_)[0];}
    inline MyPoint getEndPoint(void) const { assert(!pc_->empty()); return (*pc_)[pc_->size()-1];}
    inline MyPoint getPoint(const int &idx) const { assert(idx < pc_->size() && idx >= 0); return (*pc_)[idx];}

    void digitalize(void);

public:
    MyPointCloud::Ptr pc_, pc_ori_;

private:
    
    void preProcess(void);
    ros::Publisher pubCloud_, pubCloud_ori_;
    ros::NodeHandle nh_;
    int moving_direction_;      // 1: y+; -1: y-
    double step_;
};

#endif
