
// #include <cmath>
#include <vector>
#include <queue>
#include <mutex>
// #include <string>

#include <pcl_conversions/pcl_conversions.h>    //~ `toROSMsg
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


double g_min_range = 0.5, g_max_range = 10.0;

std::queue<sensor_msgs::PointCloud2> pointCloudBuf;
std::queue<nav_msgs::Odometry> odomBuf;
std::mutex mBuf;

const int BuffSize = 100, SystemDelayCnt = 100;



template <typename PointT>
void removeCloseAndFarPoints(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, float thres, float maxTh){
    if (&cloud_in != &cloud_out){
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }
    size_t j = 0;
    for (size_t i = 0; i < cloud_in.points.size(); ++i){
        double d = cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z;
        if (d < thres * thres || d > maxTh * maxTh)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
        cloud_out.points.resize(j);

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}


void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudPtr){
    mBuf.lock();
    if(pointCloudBuf.size()>BuffSize){
        pointCloudBuf.pop();
        // ROS_WARN("PointCloud buff pop().");
    }
    sensor_msgs::PointCloud2 pc(*laserCloudPtr);
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

    nh.param<double>("min_range", g_min_range, 0.5);
    nh.param<double>("max_range", g_max_range, 10.0);
    ROS_WARN_STREAM("range: [" << g_min_range << ", " << g_max_range << "].");

    Eigen::Matrix4d T_hor_ver = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R_hor_ver, R_delta;
    R_hor_ver << 1, 0, 0, 0, 0, -1, 0, 1, 0;        // installed with axis aligned.
    R_delta << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    // R_delta << 0.9996, -0.0207, 0.0187, 0.02054, 0.9997, 0.00928, -0.01891, -0.008897, 0.99978;
    R_hor_ver = R_delta * R_hor_ver;
    Eigen::Vector3d t_hor_ver(0, -0.25, -0.18);     // ver_lidar position in hor_lidar coordinate.
    T_hor_ver.topLeftCorner(3, 3) = R_hor_ver;
    T_hor_ver.topRightCorner(3, 1) = t_hor_ver;
    ROS_INFO_STREAM("T_hor_ver: \n" << T_hor_ver);

    std::string input_topic_name;
    nh.param<std::string>("topic_name", input_topic_name, "lslidar_point_cloud");
    // sub pointcloud from lslidar
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(input_topic_name, 100, laserCloudHandler);
    ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100, odomHandler);
    
    ros::Publisher pubRegisteredPointCloud = nh.advertise<sensor_msgs::PointCloud2>("/ver_point_registered", 100);
    ros::Publisher pubMap = nh.advertise<sensor_msgs::PointCloud2>("/ver_map", 100);
    // ros::Publisher pubCleanMap = nh.advertise<sensor_msgs::PointCloud2>("/ver_clean_map", 100);
    
    // -----------------------------------------------------------------------------------------------

    ros::Rate rate(100);
    pcl::PointCloud<pcl::PointXYZ>::Ptr fullPointCloud(new pcl::PointCloud<pcl::PointXYZ>());
    size_t system_delay_counter = 0;

    int output_ctrl_counter = 0;

    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();
        if (system_delay_counter++ < SystemDelayCnt)        // system delay
            continue;
        if(pointCloudBuf.empty() || odomBuf.empty())    
            continue;

        double odomTime = odomBuf.front().header.stamp.toSec();
        double pointCloudTime = pointCloudBuf.front().header.stamp.toSec();

        // sync time. Find pointCloud at odom time. (odom should between two pointCloud msgs.)
        nav_msgs::Odometry currOdom;
        sensor_msgs::PointCloud2 pcMsg;

        // extract sync. data from odom and pointcloud.
        mBuf.lock();
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
        if(!isSynced){
            mBuf.unlock();
            continue;
        }
        mBuf.unlock();
        // printf("[Synced]. Odom / pointCloud: %f,  %f \n", currOdom.header.stamp.toSec(), pcMsg.header.stamp.toSec());

        // reconstruct
        Eigen::Matrix4d T_w_hor = Eigen::Matrix4d::Zero();      // T_w_hor is `odom
        auto q = currOdom.pose.pose.orientation;
        auto t = currOdom.pose.pose.position;
        Eigen::Quaterniond quart(q.w, q.x, q.y, q.z);
        T_w_hor.topLeftCorner(3,3) = quart.toRotationMatrix();
        T_w_hor.topRightCorner(4, 1) = Eigen::Vector4d(t.x, t.y, t.z, 1);

        Eigen::Matrix4d T_w_ver = T_w_hor * T_hor_ver;          // calculate T_w_ver
        // to pcl data
        pcl::PointCloud<pcl::PointXYZ> pc;
        pcl::fromROSMsg(pcMsg, pc);
        
        // clean the data.
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(pc, pc, indices);
        removeCloseAndFarPoints(pc, pc, g_min_range, g_max_range);
        pcl::transformPointCloud(pc, pc, T_w_ver.cast<float>());

        // pub registered PointCloud (for debug)
        sensor_msgs::PointCloud2 registeredPointCloudMsg;
        pcl::toROSMsg(pc, registeredPointCloudMsg);
        registeredPointCloudMsg.header = pcMsg.header;
        pubRegisteredPointCloud.publish(registeredPointCloudMsg);

        // pub full map
        *fullPointCloud += pc;
        pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.setInputCloud(fullPointCloud);
        downSizeFilter.filter(*fullPointCloud);
        sensor_msgs::PointCloud2 fullPointCloudMsg;
        pcl::toROSMsg(*fullPointCloud, fullPointCloudMsg);
        fullPointCloudMsg.header = pcMsg.header;
        pubMap.publish(fullPointCloudMsg);


        if (output_ctrl_counter++ >= 10){
            output_ctrl_counter = 0;
            std::cout << "." << std::flush;
        }
    }
    
    return 0;
}

