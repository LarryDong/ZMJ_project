

#pragma once

#ifndef TOOL_H
#define TOOL_H

#include <iostream>
#include <pcl/point_types.h>
// #include <pcl/features/normal_3d.h>
#include <eigen3/Eigen/Core>
#include "defination.h"


namespace tool
{
    inline Eigen::Vector3d xyz2vector(const MyPoint& p){
        Eigen::Vector3d v;
        v(0) = p.x;
        v(1) = p.y;
        v(2) = p.z;
        return v;
    }
    inline MyPoint vector2xyz(const Eigen::Vector3d& v){
        MyPoint p;
        p.x = v(0);
        p.y = v(1);
        p.z = v(2);
        return p;
    }

    inline double calDistance(const MyPoint& p1, const MyPoint& p2){
        double d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
        return sqrtf(fabs(d));
    }
    inline double calDistance(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2){
        return calDistance(vector2xyz(v1), vector2xyz(v2));
    }

    inline sensor_msgs::PointCloud2 pointCloud2RosMsg(const MyPointCloud& cloud_in){
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud_in, msg);
        msg.header.frame_id = "laser_link";
        return msg;
    }
}


#endif
