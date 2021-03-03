

#pragma once

#ifndef TOOL_H
#define TOOL_H

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <sensor_msgs/PointCloud2.h>

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

    // // inline double dotProduct(const pcl::_Normal& n1, const pcl::_Normal& n2){
    // //     return (n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2]);
    // // }
}


#endif
