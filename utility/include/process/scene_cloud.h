
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>


using namespace std;

class SceneCloud{

public:
    SceneCloud() { cout << "[Error]. Not allowed empty input."; }

    SceneCloud(ros::NodeHandle &nh, string filename) : 
        nh_(nh),
        pc_(new pcl::PointCloud<pcl::PointXYZ>())
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *pc_) == -1){
            cout << "[Error]. Cannot open '" << filename << "'. " << endl;
            return ;
        }
        pubCloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/scene_cloud", 5);
    }


    void pub(void){     // pub pointcloud
        sensor_msgs::PointCloud2 scene_cloud_msg;
        pcl::toROSMsg(*pc_, scene_cloud_msg);
        scene_cloud_msg.header.frame_id = "/laser_link";
        pubCloud_.publish(scene_cloud_msg);
    }

public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_;


private:
    ros::Publisher pubCloud_;
    ros::NodeHandle nh_;
};

#endif
