
// 1. load pc
// 2. show pc in rviz
// 3. ..


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

#include "process/scene_cloud.h"
#include "process/car_path.h"

using namespace std;


int main(int argc, char **argv){

    ros::init(argc, argv, "process");
    ros::NodeHandle nh;

    string filename = "ver.pcd";
    SceneCloud scene_cloud(nh, filename);
    CarPath car_cloud(nh, "nav_msgs.txt");
    

    ros::Rate r(10);
    while(ros::ok()){
        scene_cloud.pub();
        car_cloud.pub();
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}