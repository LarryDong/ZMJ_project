
#pragma once

#ifndef SUPPORT_BASE_H
#define SUPPORT_BASE_H

#include <iostream>
#include <string> 

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>

#include <eigen3/Eigen/Core>

#include "process/defination.h"

using namespace std;

class SupportBase{

public:
    SupportBase(string model_name="/home/larrydong/base.pcd", string mesh_name = "/home/larrydong/base_surf.dae") {
        base_mesh_file_ = mesh_name;
        if(pcl::io::loadPCDFile(model_name, model_cloud_) !=0 ){
            cout << "[Error] SupportBase: cannot load file: " << model_name << endl;
            while(1);
        }
    }
    
    visualization_msgs::Marker createBaseMarker(Eigen::Vector4f color = Eigen::Vector4f(0.8, 0.8, 0, 1));
    bool detectBase(const MyPointCloud& scene_cloud, const Eigen::Matrix4f& init_guess);

    static int marker_id_;
    MyPointCloud raw_cloud_, model_cloud_;
    string base_mesh_file_;
    Eigen::Matrix4f transformation_;

private:
    int tmp__;

};

#endif
