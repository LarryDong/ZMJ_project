
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

#include "process/defination.h"
#include "car_path.h"


using namespace std;



class SceneCloud{
public:
    SceneCloud() { cout << "[Error]. Not allowed empty input."; }
    SceneCloud(string filename);

    void filter(double ds, double xmin, double xmax);
    int mergeAllPlanes(const ClusterParameter& cp, const PlaneParameter& pp);
    int extractAllRoofs(const ClusterParameter& cp, const PlaneParameter& pp);
    int selectRoofs(const CarPath& cp, const SupportParameter& sp);

    MyPointCloud getFullPointCloud(void) const { return *pc_; }
    MyPointCloud resetFullPointCloud(const MyPointCloud& input_pc) {*pc_ = input_pc;}
    MyPointCloud getMergedPlanes(void) const { return merged_all_planes_; }
    MyPointCloud getMergedRoofs(void) const { return merged_all_roofs_; }
    MyPointCloud getMergedValidRoofs(void) const { return merged_all_roofs_valid_; }
    MyPointCloud getAllPlaneCenters(void) const { return *plane_centers_; }
    vector<MyPointCloud> getAllRoofs(void) const { return v_roofs_; }
    vector<MyPointCloud> getAllSupports(void) const { return v_supports_; }



private:
    MyPointCloud::Ptr pc_, plane_centers_;
    MyPointCloud merged_all_planes_, merged_all_roofs_, merged_all_roofs_valid_;
    vector<MyPointCloud> v_roofs_, v_valid_roofs_, v_supports_;

    void preProcess(void);
    void extractClusters(const MyPointCloud &cloud_in, const ClusterParameter &param, std::vector<pcl::PointIndices> &out_cluster);
    bool checkIsPlane(const MyPointCloud &cloud_in, const PlaneParameter &pp);
};

#endif
