
#pragma once

#ifndef SCENE_CLOUD_H_NEW
#define SCENE_CLOUD_H_NEW


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
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>

#include "defination.h"
#include "config.h"

#include "pcl_process/car_path.h"


using namespace std;

class SceneCloud{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SceneCloud() { cout << "[Error]. Not allowed empty input."; }
    SceneCloud(string filename);
    
    MyPointCloud getFullPointCloud(void) const { return *pc_; }
    MyPointCloud resetFullPointCloud(const MyPointCloud& input_pc) { pcl::copyPointCloud(input_pc, *pc_); }     // TODO: this function is bugs.

    void filter(double ds, double xmin, double xmax);
    int filerByClustering(const ClusterParameter& cp, const PlaneParameter& pp);    // get plane pointscloud and save into `cluster_filtered_pc_`
    int extractPlanes(const ClusterParameter& cp, const PlaneParameter& pp);
    int selectRoofsAndSegment(const CarPath& cp, const SupportParameter& sp);


    MyPointCloud getClusterFilteredPC(void) const { return cluster_filtered_pc_; }
    MyPointCloud getPlanesWholePC(void) const { return merged_plane_pc_; }
    MyPointCloud getRoofsWholePC(void) const { return merged_roof_pc_; }

    MyPointCloud getPlaneCentersWholePC(void) const { return *plane_centers_; }
    vector<MyPointCloud> getRoofsVectorPC(void) const { return v_roofs_; }
    vector<MyPointCloud> getSupportsVectorPC(void) const { return v_supports_; }
    vector<MyPoint> getSupportDistance(void) const { return v_car_path_points_; }

    // private:
    MyPointCloud::Ptr pc_, plane_centers_;
    MyPointCloud cluster_filtered_pc_;      // first filter the pc by clustering small planes
    vector<MyPointCloud> v_roofs_, v_valid_roofs_, v_supports_;
    vector<MyPoint> v_car_path_points_;
    // for debug
    MyPointCloud merged_plane_pc_;         // all planePC from `cluster_filtered_pc`
    MyPointCloud merged_roof_pc_;          // all roof PC from `cluster_filtered_pc`
    

    void preProcess(void);
    void extractClusters(const MyPointCloud &cloud_in, const ClusterParameter &param, std::vector<pcl::PointIndices> &out_cluster);
    bool checkIsPlane(const MyPointCloud &cloud_in, const PlaneParameter &pp);
};

#endif
