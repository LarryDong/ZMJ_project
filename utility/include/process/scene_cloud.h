
#pragma once

#ifndef SCENE_CLOUD_H
#define SCENE_CLOUD_H


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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>



using namespace std;


class RandColor{
public:
    RandColor(){
        r = rand() % 255;
        g = rand() % 255;
        b = rand() % 255;
    }
    double r,g,b;
};


class ClusterParameter{
public:
    ClusterParameter(double angle, double dist, int mi, int ma) 
        : delta_angle(angle), delta_distance(dist), min_num(mi), max_num(ma) {}

    double delta_angle, delta_distance;
    int min_num, max_num;
};


class SceneCloud{

public:
    SceneCloud() { cout << "[Error]. Not allowed empty input."; }
    SceneCloud(ros::NodeHandle &nh, string filename);
    void pub();

    void detectPlanes(void);
    void filter(void);

public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_, plane_centers_;
    vector<pcl::PointCloud<pcl::PointXYZ>> v_planes_;

private:
    void preProcess(void);
    void extractClusters(
        const pcl::PointCloud<pcl::PointXYZ> &cloud_in, 
        const pcl::PointCloud<pcl::Normal> &normals_in,
        const pcl::search::Search<pcl::PointXYZ>::Ptr &tree_in,
        const ClusterParameter& param,
        std::vector<pcl::PointIndices> &out_cluster);

    ros::Publisher pubCloud_;
    ros::NodeHandle nh_;
};

#endif
