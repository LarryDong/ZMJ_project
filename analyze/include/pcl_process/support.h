
// This include all components of a support

#pragma once

#ifndef SUPPORT_ALL_H
#define SUPPORT_ALL_H

#include <iostream>
#include <string> 

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <pcl/features/normal_3d.h>                     // pcl::NormalEstimation
#include <pcl/filters/extract_indices.h>                // ExtractIndices
#include <pcl/segmentation/sac_segmentation.h>          // pcl::SACSegmentationFromNormals
#include <pcl/filters/statistical_outlier_removal.h>    // pcl:StatisticalOutlierRemoval

// #include <pcl/io/pcd_io.h>
// #include <pcl/io/ply_io.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/registration/icp.h>

#include <eigen3/Eigen/Core>

#include "defination.h"

using namespace std;


class PlaneParameters{
public:
    PlaneParameters(double xmin, double xmax, double zmin, double zmax):
        x_min_(xmin), x_max_(xmax), z_min_(zmin), z_max_(zmax){}

    double x_min_, x_max_;
    double z_min_, z_max_;
};

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


// class SupportConfigs{
// public:
//     SupportConfigs(){};
//     inline void setRoof(double xmin, double xmax, double zmin, double zmax, double angle){
//         roof_norm_angle = angle;
//         roof_x_min = xmin;
//         roof_x_max = xmax;
//         roof_z_min = zmin;
//         roof_z_max = zmax;
//     }
//     inline void setSegment(double xmin, double xmax, double y_range, double zmin, double zmax){
//         segment_x_min = xmin;
//         segment_x_max = xmax;
//         segment_y = y_range;
//         segment_z_min =zmin;
//         segment_z_max=zmax;
//     }

// public: 
//     // roof parameters
//     double roof_norm_angle;
//     double roof_x_min, roof_x_max;
//     double roof_z_min, roof_z_max;
//     // segment parameters
//     double segment_y;
//     double segment_z_min, segment_z_max;
//     double segment_x_min, segment_x_max;
// };


// class Support{
// public:
//     // Support(void) {}
//     Support(const MyPointCloud& input_cloud, const MyPoint& position)
//         : position_(position)
//     {
//         pcl::copyPointCloud(input_cloud, pc_);
        
//         // id_++;
//     }
//     // void detectCylinder(const CylinderParameters& cp){;}// TODO:
//     void detectBase() { ; };
//     void detectPlane() { ; }
    
//     // Not allow operator=;
//     Support &operator=(const Support &rhs) { cout << "Warning. Operator= is not implemented" << endl; }

//     void setSupportConfigs()
    
//     SupportConfigs config_;
// // private:
//     static int id_;
//     MyPointCloud pc_;
//     MyPoint position_;      // position @ car path. Used for some segmentation.

//     vector<double> base_coeff_;     // px, py, pz, rx, ry, rz
//     vector<double> cylinder_coeff_; // top, down, xyz; radius
//     vector<double> plane_coeff_;    //

// };


// bool detectCylinder(const MyPointCloud& input_cloud, MyPointCloud& output_cylinder, const CylinderParameters& cp);

#endif
