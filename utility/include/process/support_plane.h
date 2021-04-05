
#pragma once

#ifndef SUPPORT_PLANE_H
#define SUPPORT_PLANE_H

#include <iostream>
#include <string> 

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// #include <pcl/io/pcd_io.h>
// #include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/registration/icp.h>

#include <eigen3/Eigen/Core>

#include "process/defination.h"

using namespace std;


class PlaneParameters{
public:
    PlaneParameters(double xmin, double xmax, double zmin, double zmax):
        x_min_(xmin), x_max_(xmax), z_min_(zmin), z_max_(zmax)
    {}

    double x_min_, x_max_;
    double z_min_, z_max_;
};


class SupportPlane{
public:
    SupportPlane(const MyPointCloud &cloud_in) {
        pcl::copyPointCloud(cloud_in, raw_cloud_); 
    }
    void resetPointCloud(const MyPointCloud &cloud_in) { pcl::copyPointCloud(cloud_in, raw_cloud_); }
    void detectPlane(const PlaneParameters &pp);
    visualization_msgs::Marker createMarker(Eigen::Vector4f color = Eigen::Vector4f(0.5, 0.5, 1.0, 1));


    MyPointCloud raw_cloud_, plane_cloud_;

private:

    static int marker_id_;
    
    Eigen::Vector3f plane_center_;
    Eigen::Vector3f plane_normal_;
    Eigen::Quaternionf q_;
    double plane_width_, plane_height_;     // directly from Settings....
};


#endif
