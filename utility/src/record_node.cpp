// record all necessary data.
// record horizontal pointcloud and vertical pointcloud when receiving "s"
// pub command (in terminal): rostopic pub /debug std_msgs/String "s" -1
//                          -- create by dongy. Modified: 20210101

#include <iostream>

#include <mutex>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

using namespace std;

std::mutex mMap;
pcl::PointCloud<pcl::PointXYZ> g_horPointCloud, g_verPointCloud;
nav_msgs::Path g_carPath;

void handler(const std_msgs::String s){
    ROS_INFO_STREAM("Input message: " << s);
    if(s.data == "s"){
        mMap.lock();
        pcl::io::savePCDFileASCII("ver.pcd", g_verPointCloud);
        pcl::io::savePCDFileASCII("hor.pcd", g_horPointCloud);

        // save path files;
        ofstream out("nav_msgs.txt", ios::out);
        for(auto pose : g_carPath.poses){
            geometry_msgs::Pose p = pose.pose;

            out << p.position.x << " " << p.position.y << " " << p.position.z << " "
                << p.orientation.w << " " << p.orientation.x << " " << p.orientation.y << " " << p.orientation.z << endl;
        }


        mMap.unlock();
        std::cout << "Saved ver.pcd (" << g_verPointCloud.size() << "), hor.pcd (" << g_horPointCloud.size() << "). " << std::endl;
        std::cout << "Saved nav_msgs to nav_msgs.txt (" << g_carPath.poses.size() << std::endl;
    }
    else{
        ROS_WARN("Incorrect command.");
    }
}

void horPointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &pc){
    mMap.lock();
    pcl::fromROSMsg(*pc, g_horPointCloud); // 将传入的ros消息格式转为pcl库里的点云格式
    mMap.unlock();
}

void verPointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &pc){
    mMap.lock();
	pcl::fromROSMsg(*pc, g_verPointCloud); // 将传入的ros消息格式转为pcl库里的点云格式
    mMap.unlock();
}

void pathHandler(const nav_msgs::PathConstPtr &path){
    mMap.lock();
    g_carPath = *path;
    mMap.unlock();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "my_debug");
    ros::NodeHandle nh;
    ROS_INFO("Debug node begin......");
    std::cout << "type: 'rostopic pub /debug std_msgs/String \"s\" -1' in terminal to save pointcloud." << std::endl;

    ros::Subscriber subString = nh.subscribe<std_msgs::String>("/debug", 1, handler);
    ros::Subscriber subHorMap = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_map", 100, horPointCloudHandler);
    ros::Subscriber subVerMap = nh.subscribe<sensor_msgs::PointCloud2>("/ver_map", 100, verPointCloudHandler);
    ros::Subscriber subPath = nh.subscribe<nav_msgs::Path>("/car_path", 100, pathHandler);

    ros::Rate r(10);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}