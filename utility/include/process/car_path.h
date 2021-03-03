
#pragma once

#ifndef CAR_PATH_H
#define CAR_PATH_H


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

class CarPath{

public:
    CarPath() { cout << "[Error]. Not allowed empty input."; }

    CarPath(ros::NodeHandle &nh, string filename) : 
        nh_(nh),
        pc_(new pcl::PointCloud<pcl::PointXYZ>())
    {
        ifstream in(filename);
        if (!in.is_open()){
            cout << "[Error]. Cannot load nav_msgs" << endl;
            return ;
        }
        while (!in.eof()){
            double tmp; // orientation information is useless;
            pcl::PointXYZ p;
            in >> p.x >> p.y >> p.z >> tmp >> tmp >> tmp >> tmp;
            pc_->push_back(p);
        }
        pubCloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/car_path_cloud", 5);
    }

    void pub(void){     // pub pointcloud
        sensor_msgs::PointCloud2 car_msg;
        pcl::toROSMsg(*pc_, car_msg);
        car_msg.header.frame_id = "/laser_link";
        pubCloud_.publish(car_msg);
    }

public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_;

private:
    ros::Publisher pubCloud_;
    ros::NodeHandle nh_;
};

#endif
