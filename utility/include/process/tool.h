

#pragma once

#ifndef TOOL_H
#define TOOL_H

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <sensor_msgs/PointCloud2.h>

#include <eigen3/Eigen/Core>

namespace tool{
    inline double calDistance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2){
        double d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
        return sqrtf(fabs(d));
    }

    inline sensor_msgs::PointCloud2 pointCloud2RosMsg(const pcl::PointCloud<pcl::PointXYZ>& cloud_in){
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud_in, msg);
        msg.header.frame_id = "laser_link";
        return msg;
    }

    inline Eigen::Vector3d xyz2vector(const pcl::PointXYZ& p){
        Eigen::Vector3d v;
        v(0) = p.x;
        v(1) = p.y;
        v(2) = p.z;
        return v;
    }

    // inline double dotProduct(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2){
    //     return p1.dot()
    // }

    // // inline double dotProduct(const pcl::_Normal& n1, const pcl::_Normal& n2){
    // //     return (n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2]);
    // // }
}


#endif
