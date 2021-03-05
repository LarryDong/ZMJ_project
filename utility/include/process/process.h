
#pragma once

#ifndef PROCESS_H
#define PROCESS_H

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

#include <eigen3/Eigen/Core>

#include <gflags/gflags.h>

#include "process/car_path.h"
#include "process/scene_cloud.h"

Eigen::Matrix4d calcGlobalT(const CarPath& cp);
void updateCoordinate(CarPath& cp, SceneCloud& sc, const Eigen::Matrix4d& T);


#endif
