
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

#include <gflags/gflags.h>


DEFINE_string(ver_saving, "/home/larrydong/lidar_ws/output/raw/ver.pcd", "vertical point cloud saved");
DEFINE_string(hor_saving, "/home/larrydong/lidar_ws/output/raw/hor.pcd", "horizontal point cloud saved");
DEFINE_string(car_path_saving, "/home/larrydong/lidar_ws/output/raw/car_path.txt", "car path pointcloud.");


using namespace std;



std::mutex mMap;
pcl::PointCloud<pcl::PointXYZ> g_horPointCloud, g_verPointCloud;
nav_msgs::Path g_carPath;

bool g_horDataGet = false, g_verDataGet = false, g_carPathGet = false;


void saveData(void){
    mMap.lock();
    pcl::io::savePCDFileASCII(FLAGS_ver_saving, g_verPointCloud);
    pcl::io::savePCDFileASCII(FLAGS_hor_saving, g_horPointCloud);  // two clouds are used for extrinsics calibration

    // save path files;
    ofstream out(FLAGS_car_path_saving, ios::out);
    for(auto pose : g_carPath.poses){
        geometry_msgs::Pose p = pose.pose;

        out << p.position.x << " " << p.position.y << " " << p.position.z << " "
            << p.orientation.w << " " << p.orientation.x << " " << p.orientation.y << " " << p.orientation.z << endl;
    }
    mMap.unlock();
    cout << "Saved (" << g_verPointCloud.size() << ") pts into ver: " << FLAGS_ver_saving << endl;
    cout << "Saved (" << g_horPointCloud.size() << ") pts into hor: " << FLAGS_hor_saving << endl;
    cout << "Saved (" << g_carPath.poses.size() << ") car path into: " << FLAGS_car_path_saving << endl;
}

void horPointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &pc){
    mMap.lock();
    pcl::fromROSMsg(*pc, g_horPointCloud); // 将传入的ros消息格式转为pcl库里的点云格式
    mMap.unlock();
    g_horDataGet = true;
}

void verPointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &pc){
    mMap.lock();
	pcl::fromROSMsg(*pc, g_verPointCloud); // 将传入的ros消息格式转为pcl库里的点云格式
    mMap.unlock();
    g_verDataGet = true;
}

void pathHandler(const nav_msgs::PathConstPtr &path){
    mMap.lock();
    g_carPath = *path;
    mMap.unlock();
    g_carPathGet = true;
}


int main(int argc, char **argv){

    ros::init(argc, argv, "record_data");
    ros::NodeHandle nh;
    google::ParseCommandLineFlags(&argc, &argv, true);

    cout << "This node save (hor/ver) pcd and car path when received data for the first time." << endl;

    ros::Subscriber subHorMap = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_map", 100, horPointCloudHandler);
    ros::Subscriber subVerMap = nh.subscribe<sensor_msgs::PointCloud2>("/ver_map", 100, verPointCloudHandler);
    ros::Subscriber subPath = nh.subscribe<nav_msgs::Path>("/car_path", 100, pathHandler);

    ros::Rate r(10);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
        if(g_carPathGet && g_horDataGet && g_verDataGet){   // save data when all are got.
            saveData();
            ROS_WARN("Data saved.");
            return 0;
        }
    }
    return 0;
}