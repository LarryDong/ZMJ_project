
#pragma once

#ifndef SUPPORT_CYLINDER_H
#define SUPPORT_CYLINDER_H

#include <iostream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <eigen3/Eigen/Core>

#include "process/defination.h"

using namespace std;


class CylinderParameters{
public:
    CylinderParameters(double search_r, double weight, double distance_th, int iter, double min_r, double max_r):
        search_radius_(search_r),
        normal_distance_weight_(weight),
        distance_threshold_(distance_th),
        max_iteration_(iter),
        min_radius_(min_r),
        max_radius_(max_r){}

    double search_radius_;
    double normal_distance_weight_;
    double distance_threshold_;
    int max_iteration_;
    double min_radius_, max_radius_;
};


class Cylinder{
public:
    Cylinder(const MyPointCloud &cloud_in) { pcl::copyPointCloud(cloud_in, raw_cloud_); }
    void resetPointCloud(const MyPointCloud &cloud_in) { pcl::copyPointCloud(cloud_in, raw_cloud_); }
    bool detectCylinder(const CylinderParameters& cp);
    MyPointCloud getCylinderPointCloud(void) {return cylinder_cloud_;}
    MyPointCloud getNoneCylinderPointCloud(void) {return non_cylinder_cloud_;}
    visualization_msgs::Marker calMarker(double b = 0, double g = 1.0, double r = 0, double a = 0.5);

private:
    void calCylinderCoeff(pcl::ModelCoefficients::Ptr raw_coeff);
    void cylinder_filter(MyPointCloud& pc);

    static int marker_id_;
    MyPointCloud raw_cloud_, cylinder_cloud_, non_cylinder_cloud_;
    Eigen::Vector3d cylinder_center_;
    Eigen::Quaterniond q_;          // cylinder axis's quat.
    double cylinder_length_;
    double cylinder_radius_;
};

#endif
