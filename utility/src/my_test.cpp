
// 1. load pc
// 2. show pc in rviz
// 3. ..


#include <iostream>
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
    CarPath(string filename){
        ifstream in("nav_msgs.txt");
        if (!in.is_open()){
            cout << "Cannot load nav_msgs" << endl;
            return ;
        }
        while (!in.eof()){
            double tmp;     // orientation information is useless;
            pcl::PointXYZ p;
            in >> p.x >> p.y >> p.z >> tmp >> tmp >> tmp >> tmp;
            pc.push_back(p);
        }

        pcl::toROSMsg(pc, msg);
        msg.header.frame_id = "/laser_link";
    }

    pcl::PointCloud<pcl::PointXYZ> pc;
    sensor_msgs::PointCloud2 msg;

};




int main(int argc, char **argv){

    ros::init(argc, argv, "my_test");
    ros::NodeHandle nh;

    // ros::Subscriber subString = nh.subscribe<std_msgs::String>("/debug", 1, handler);
    // ros::Subscriber subHorMap = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_map", 100, horPointCloudHandler);
    // ros::Subscriber subVerMap = nh.subscribe<sensor_msgs::PointCloud2>("/ver_map", 100, verPointCloudHandler);
    // ros::Subscriber subPath = nh.subscribe<nav_msgs::Path>("/car_path", 100, pathHandler);

    
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/scene_cloud", 100);
    ros::Publisher pubPathPC = nh.advertise<sensor_msgs::PointCloud2>("/path_pc", 5);

    // load pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    string filename = "ver.pcd";
    if(argc == 2)
        filename = argv[1];
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *scene_cloud) == -1){
        cout << "Cannot open '" << filename << "'. " << endl;
    }

    // to RosMsgs
    sensor_msgs::PointCloud2 scene_cloud_msg;
    pcl::toROSMsg(*scene_cloud, scene_cloud_msg);
    scene_cloud_msg.header.frame_id = "/laser_link";
    

    // load path
    CarPath car_path("nav_msgs.txt");


    ros::Rate r(10);
    while(ros::ok()){
        pubCloud.publish(scene_cloud_msg);
        pubPathPC.publish(car_path.msg);
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}