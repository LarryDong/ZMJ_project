
#include <cmath>
#include <vector>
#include <string>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"

#include <pcl_conversions/pcl_conversions.h>    //~ `toROSMsg
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


using std::atan2;
using std::cos;
using std::sin;

const double scanPeriod = 0.05;      //~ 1/frequency.

// const int systemDelay = 0; 
// int systemInitCount = 0;
// bool systemInited = false;
int N_SCANS = 32;
float cloudCurvature[400000];       //~ each point's curvature
int cloudSortInd[400000];           //~ 
int cloudNeighborPicked[400000];    //~ whether a point's neighbour is picked. 1 for picked.
int cloudLabel[400000];             //~ note a point's label. Sharp(2, 1) or Flat(-1).

bool comp(int i, int j) { return (cloudCurvature[i] < cloudCurvature[j]); }

ros::Publisher pubLaserCloud;               //~ origin pointCloud from Lidar
ros::Publisher pubCornerPointsSharp;        
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
// ros::Publisher pubRemovePoints;
std::vector<ros::Publisher> pubEachScan;

bool PUB_EACH_LINE = false;
double MINIMUM_RANGE = 0.1;

template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, float thres){
    if (&cloud_in != &cloud_out){
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }
    size_t j = 0;
    for (size_t i = 0; i < cloud_in.points.size(); ++i){
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;           //~ avoid too near points. 
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())        //~ using 'resize' to delete tails.
        cloud_out.points.resize(j);

    //~ parameters for pcl.
    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){

    // // STEP 0. System init.
    // if (!systemInited){ 
    //     systemInitCount++;
    //     if (systemInitCount >= systemDelay){            //~ use delay to avoid laser without imu.
    //         systemInited = true;
    //     }
    //     else
    //         return;
    // }

    TicToc t_whole;
    TicToc t_prepare;
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    std::vector<int> indices;

    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);

    
    int cloudSize = laserCloudIn.points.size();
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);        //~ TODO: why a minus-sign ?
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y, laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

    if (endOri - startOri > 3 * M_PI)
        endOri -= 2 * M_PI;
    else if (endOri - startOri < M_PI)
        endOri += 2 * M_PI;
    //~ lslidar: [-1.43, 4.76];
    // ROS_WARN_STREAM("Angle range: [" << startOri << ", " << endOri << "] .");
    // return;

    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    for (int i = 0; i < cloudSize; i++){
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;     //~ layer angle. Used to determine scanID
        int scanID = int(angle + 17);
        // ROS_INFO_STREAM("ID: " << scanID << ", Angle: " << angle);
        if (scanID < 0 || scanID >= N_SCANS - 1){
            // ROS_ERROR("invalid scan...");
            count--;
            continue;
        }

        float ori = -atan2(point.y, point.x);       //~ TODO: why a minus-sign?
        if (!halfPassed){
            if (ori < startOri - M_PI / 2)
                ori += 2 * M_PI;
            else if (ori > startOri + M_PI * 3 / 2)
                ori -= 2 * M_PI;
            if (ori - startOri > M_PI)
                halfPassed = true;
        }
        else{
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
                ori += 2 * M_PI;
            else if (ori > endOri + M_PI / 2)
                ori -= 2 * M_PI;
        }

        float relTime = (ori - startOri) / (endOri - startOri);     //~ use angle to calculate time interp
        point.intensity = scanID + scanPeriod * relTime;            //~ `intensity parameter is used for "ID.time"
        laserCloudScans[scanID].push_back(point);
    }
    cloudSize = count; // 去除一些非法点之后的点云数量
    // printf("points size %d \n", cloudSize);


    // STEP 2. Calculate corner/flat points
    // calculate carvature.
    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    // 记录每一层起始点和终止点的位置，需要根据这个起始/终止来操作点云曲率，在求曲率的过程中已经去除了前5个点和后5个点
    for (int i = 0; i < N_SCANS; i++){
        scanStartInd[i] = laserCloud->size() + 5;
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;
    }
    for (int i = 5; i < cloudSize - 5; i++){ 
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;
        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ; // 曲率
        cloudSortInd[i] = i; // 每个点云的index，后面根据曲率进行排序的时候使用
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0; // 默认为0，则为surfPointsLessFlatScan点
    }

    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> cornerPointsLessSharp;
    pcl::PointCloud<PointType> surfPointsFlat;
    pcl::PointCloud<PointType> surfPointsLessFlat;
    // select corners/surf in each scan
    for (int i = 0; i < N_SCANS; ++i){
        if (scanEndInd[i] - scanStartInd[i] < 6)    // skip less point scans
            continue;
        for (int j = 0; j < 6; ++j){
            // start/end point of each scan
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
        
            int largestPickedNum = 0;
            for(int k=ep; k>=sp; --k){
                int ind = cloudSortInd[k];
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1){
                    largestPickedNum++;
                    if(largestPickedNum<=2){    // record 2 largest corner
                        cloudLabel[ind]=2;
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                    }
                    else if(largestPickedNum<=20){
                        cloudLabel[ind]=1;
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else 
                        break;
                    cloudNeighborPicked[ind]=1;

                    // mark around points;
                    for (int l = 1; l <= 5; l++){
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--){
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            break;
                        cloudNeighborPicked[ind + l] = 1;
                    }

                    // TODO: deal with planar points
                }
            }
        }
    }


    // Pub 1: full clouds
    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "/laser_link";
    pubLaserCloud.publish(laserCloudOutMsg); // 去除掉一些无效点之后的原始点云数据

    // Pub2: corners
    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "/laser_link";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);

    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "/laser_link";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    // Pub2: pub each linese
    if (PUB_EACH_LINE){
        for (int i = 0; i < N_SCANS; i++){
            sensor_msgs::PointCloud2 scanMsg;
            pcl::toROSMsg(laserCloudScans[i], scanMsg);
            scanMsg.header.stamp = laserCloudMsg->header.stamp;
            scanMsg.header.frame_id = "/laser_link";
            pubEachScan[i].publish(scanMsg);
        }
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "scanRegistration");
    ros::NodeHandle nh;
    ROS_INFO("Scan registration node begin...");

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud", 100, laserCloudHandler);

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/lslidar_point_cloud_2", 100);
    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);
    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);
    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);
    // pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);
    // pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);      //~ not subscribed by anyone.

    if(PUB_EACH_LINE){              //~ publish every scan lines. From 1-16/32/64
        for(int i = 0; i < N_SCANS; i++){
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
            pubEachScan.push_back(tmp);
        }
    }
    ros::spin();
    return 0;
}

