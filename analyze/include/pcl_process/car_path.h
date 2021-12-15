
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

#include "tool.h"

using namespace std;

class CarPath{

public:
    CarPath() { cout << "[Error]. Not allowed empty input."; }
    CarPath(string filename);

    MyPointCloud getFullPointCloud(void) const { return *pc_; }
    MyPointCloud resetFullPointCloud(const MyPointCloud& input_pc) {*pc_ = input_pc;}

    double getClosestPointInPath(const MyPoint& in, MyPoint& out);
    inline MyPoint getBeginPoint(void) const { assert(!pc_->empty()); return (*pc_)[0];}
    inline MyPoint getEndPoint(void) const { assert(!pc_->empty()); return (*pc_)[pc_->size()-1];}
    inline MyPoint getAnyPoint(const int &idx) const { assert(idx < pc_->size() && idx >= 0); return (*pc_)[idx];}

    void digitalize(void);

private:
    MyPointCloud::Ptr pc_, pc_ori_;
    double step_;

    void preProcess(void);
};

#endif
