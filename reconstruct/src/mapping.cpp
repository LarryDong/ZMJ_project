
// #include <cmath>
#include <vector>
#include <queue>
#include <mutex>
// #include <string>

#include <pcl_conversions/pcl_conversions.h>    //~ `toROSMsg
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>



std::queue<sensor_msgs::PointCloud2> pointCloudBuf;
std::queue<nav_msgs::Odometry> odomBuf;
std::mutex mBuf;

const int BuffSize = 100;

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudPtr){
    mBuf.lock();
    if(pointCloudBuf.size()>BuffSize){
        pointCloudBuf.pop();
        // ROS_WARN("PointCloud buff pop().");
    }
    sensor_msgs::PointCloud2 pc(*laserCloudPtr);            // TODO: correct or not???
    pc.header.stamp = ros::Time().now();
    pointCloudBuf.push(pc);
    mBuf.unlock();
    // ROS_INFO_STREAM("Vertical point cloud size: " << pointCloudBuf.size());
}

void odomHandler(const nav_msgs::OdometryConstPtr &odomPtr){
    mBuf.lock();
    if(odomBuf.size()>BuffSize){
        odomBuf.pop();
        // ROS_WARN("Odom buff pop().");
    }
    odomBuf.push(*odomPtr);
    mBuf.unlock();
    // ROS_INFO_STREAM("Odom size: " << odomBuf.size());
}

int main(int argc, char **argv){

    ros::init(argc, argv, "reconstruct");
    ros::NodeHandle nh;
    ROS_INFO("Vertical Mapping node begin...");

    std::string input_topic_name;
    nh.param<std::string>("topic_name", input_topic_name, "lslidar_point_cloud");
    std::cout<<"--------"<<input_topic_name<<" --------"<<std::endl;
    // sub pointcloud from lslidar
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(input_topic_name, 100, laserCloudHandler);
    ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100, odomHandler);
    
    ros::Publisher pubRegisteredPointCloud = nh.advertise<sensor_msgs::PointCloud2>("/ver_point_registered", 100);

    ros::Rate rate(100);
    int counter=0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr fullPointCloud(new pcl::PointCloud<pcl::PointXYZ>());

    size_t system_delay_counter = 0;
    while(ros::ok()){

        rate.sleep();
        ros::spinOnce();

        if (system_delay_counter++ < 100)               // system delay
            continue;

        if(pointCloudBuf.empty() || odomBuf.empty()){
            continue;
        }
        double odomTime = odomBuf.front().header.stamp.toSec();
        double pointCloudTime = pointCloudBuf.front().header.stamp.toSec();
        // printf("odom/point time: %f, %f \n", odomTime, pointCloudTime);

        // sync time. Find pointCloud at odom time. (odom should between two pointCloud msgs.)
        nav_msgs::Odometry currOdom;
        sensor_msgs::PointCloud2 pcMsg;
        bool isSynced = false;
        while (!odomBuf.empty() && !pointCloudBuf.empty() && !isSynced){
            double currOdomTime, firstPointTime, secondPointTime;
            currOdom = odomBuf.front();
            odomBuf.pop();
            currOdomTime = currOdom.header.stamp.toSec();
            firstPointTime = pointCloudBuf.front().header.stamp.toSec();
            // printf("odom / point: %f,  %f \n", currOdomTime, firstPointTime);

            if(currOdomTime <= firstPointTime)      // if odom is earlier, skip.
                continue;

            while (pointCloudBuf.size() >= 2 && !isSynced){
                firstPointTime = pointCloudBuf.front().header.stamp.toSec();
                pcMsg = pointCloudBuf.front();
                pointCloudBuf.pop();
                secondPointTime = pointCloudBuf.front().header.stamp.toSec();
                if(currOdomTime >= firstPointTime && currOdomTime <= secondPointTime){
                    // printf("[Catch]: odom / first / second: %f,  %f,  %f \n", currOdomTime, firstPointTime, secondPointTime);
                    isSynced = true;
                }
            }
        }

        // printf("[Synced]. Odom / pointCloud: %f,  %f \n", currOdom.header.stamp.toSec(), pcMsg.header.stamp.toSec());


        // // reconstruct
        Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
        auto q = currOdom.pose.pose.orientation;
        auto t = currOdom.pose.pose.position;
        Eigen::Quaterniond quart(q.w, q.x, q.y, q.z);
        T.topLeftCorner(3,3) = quart.toRotationMatrix();
        T.topRightCorner(4, 1) = Eigen::Vector4d(t.x, t.y, t.z, 1);
        // to pcl data
        pcl::PointCloud<pcl::PointXYZ> pc;
        pcl::fromROSMsg(pcMsg, pc);
        // pcl::transformPointCloud(pc, pc, T.cast<float>());

        // sensor_msgs::PointCloud2 registeredPointCloud;
        // pcl::toROSMsg(pc, registeredPointCloud);
        // registeredPointCloud.header = pcMsg.header;
        // ROS_INFO("Before pub...");
        // pubRegisteredPointCloud.publish(registeredPointCloud);
    }
    
    return 0;
}

