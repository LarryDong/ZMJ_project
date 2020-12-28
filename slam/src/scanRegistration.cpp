
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

// global settings.
const int N_SCANS = 32;
int g_skip_counter = 0;
const bool PUB_EACH_LINE = false;   // pub each-line, for debug.

// global parameters from launch.file
double g_min_range = 1.0, g_max_range = 10.0;
int g_scan_skip = 0;                // skip every `scan_skip scans.
int g_used_scans = 32;               // scan numbers = total-scan-number / (scan_skip+1);
int g_sector_num = 6;
int g_sharp_num = 2, g_sharpless_num = 20;
int g_flat_num = 4;
double g_flatless_ds = 0.4;

// const int systemDelay = 0;
// int systemInitCount = 0;
// bool systemInited = false;

float cloudCurvature[400000];       // 每个点云的曲率
int cloudSortInd[400000];           // 每个点云的index，根据曲率进行排序的时候使用
int cloudNeighborPicked[400000];    // 表示某点已经被打上过标签的标记
int cloudLabel[400000];             // 点云的类别，分为4种，-1,0,1,2，对应着surfPointsFlatScan，surfPointsLessFlatScan，cornerPointsLessSharp，cornerPointsSharp

bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }  // 根据曲率进行排序的比较函数

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
// ros::Publisher pubRemovePoints;
std::vector<ros::Publisher> pubEachScan;

ros::Time g_cloud_input_time = ros::Time();

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


// main handler
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
    // printf("Cloud msg time: %f \n", laserCloudMsg->header.stamp.toSec());
    g_cloud_input_time = ros::Time().now();

    g_skip_counter++;
    if (g_skip_counter % (g_scan_skip+1) != 0)
        return; 

    // if (!systemInited){ 
    //     systemInitCount++;
    //     if (systemInitCount >= systemDelay){
    //         systemInited = true;
    //     }
    //     else
    //         return;
    // }

    TicToc t_whole;
    std::vector<int> scanStartInd(g_used_scans, 0);
    std::vector<int> scanEndInd(g_used_scans, 0);

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn; // 输入点云，pcl点云格式
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn); // 将传入的ros消息格式转为pcl库里的点云格式
    std::vector<int> indices;

    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices); // 去除无效点
    removeCloseAndFarPoints(laserCloudIn, laserCloudIn, g_min_range, g_max_range); // // 去除一些距离激光雷达过近的点，通常这些点被认为是不可靠的

    int cloudSize = laserCloudIn.points.size();
    // 激光雷达旋转的起始角度，用第一个点来计算
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    // 激光雷达旋转的终止角度，用最后一个点来计算
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y, laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

    // 定义激光点的角度范围在M_PI到3*M_PI之间(即2*M_PI左右，正好一圈)
    if (endOri - startOri > 3 * M_PI)
        endOri -= 2 * M_PI;
    else if (endOri - startOri < M_PI)
        endOri += 2 * M_PI;
    //printf("end Ori %f\n", endOri);

    bool halfPassed = false;
    int count = 0; // 去除掉一些非法点之后的点云数量
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);


    // 将所有点云按不同的线束分别放在laserCloudScans的数组里，并计算每个点在该线束中根据起始和终点计算的位置（时间）
    int id_counter = 0;
    for (int i = 0; i < cloudSize; i++){
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        // 计算每个3d点有关于激光雷达原点连线的俯仰角，以此来判断该点云是哪层线束的激光
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = int(angle + 17);
        if (scanID % (g_scan_skip+1) != 0)                // skip half scans.
            continue;

        int selectedID = scanID / (g_scan_skip + 1);
        float ori = -atan2(point.y, point.x); // 计算每个点云和激光雷达原点连线的航向角
        if (!halfPassed){ // 根据扫描线是否旋转过半选择与起始位置还是终止位置进行差值计算
            // 确保-3*pi/2 < ori - endOri < pi/2
            if (ori < startOri - M_PI / 2)  
                ori += 2 * M_PI;
            else if (ori > startOri + M_PI * 3 / 2)
                ori -= 2 * M_PI;
            if (ori - startOri > M_PI)
                halfPassed = true;
        }
        else{       // 确保-3*pi/2 < ori - endOri < pi/2
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
                ori += 2 * M_PI;
            else if (ori > endOri + M_PI / 2)
                ori -= 2 * M_PI;
        }

        // float relTime = (ori - startOri) / (endOri - startOri);
        // point.intensity = scanID + scanPeriod * relTime;
        point.intensity = selectedID;           // intensity is necessary ti check ID.
        laserCloudScans[selectedID].push_back(point);
        count++;
    }
    
    cloudSize = count; // 去除一些非法点之后的点云数量
    // printf("points size %d \n", cloudSize);

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    // 记录每一层起始点和终止点的位置，需要根据这个起始/终止来操作点云曲率，在求曲率的过程中已经去除了前5个点和后5个点
    for (int i = 0; i < g_used_scans; i++){ 
        scanStartInd[i] = laserCloud->size() + 5;
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;
    }

    for (int i = 5; i < cloudSize - 5; i++){ // 计算每个点云的曲率cloudCurvature[i]
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

    float t_q_sort = 0;
    for (int i = 0; i < g_used_scans; i++){
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        for (int j = 0; j < g_sector_num; j++){ // 将点云均分成6块区域，每块区域选取一定数量的Edge Points，和Planar Points
            // 每块区域的开始点和结束点
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / g_sector_num; 
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / g_sector_num - 1;

            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--) {// 曲率从大到小遍历点云
                int ind = cloudSortInd[k]; // 曲率从大到小顺序下点云的index

                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1){
                    largestPickedNum++;
                    if (largestPickedNum <= g_sharp_num){
                        cloudLabel[ind] = 2; // 标签为2表示cornerPointsSharp
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else if (largestPickedNum <= g_sharpless_num){                        
                        cloudLabel[ind] = 1; // 标签为1表示cornerPointsLessSharp
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else
                        break;

                    cloudNeighborPicked[ind] = 1; // 表示这点被选取过了
                    // 之后这个点和它周围一些距离比较的点也都打上标签，之后不会被选取
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
                }
            }

            int smallestPickedNum = 0;
            // 相似的方法选取Planar Points
            for (int k = sp; k <= ep; k++) { // 曲率从小到大遍历点云
                if (g_flat_num == 0)         // skip if setting 0 planar points.
                    break;
                int ind = cloudSortInd[k];
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1){
                    cloudLabel[ind] = -1; 
                    surfPointsFlat.push_back(laserCloud->points[ind]);
                    smallestPickedNum++;
                    if (smallestPickedNum >= g_flat_num) 
                        break;
                    cloudNeighborPicked[ind] = 1;
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
                }
            }

            // remaining are `less-flat points
            for (int k = sp; k <= ep; k++){
                if (cloudLabel[k] <= 0)
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
            }
        }

        // downsampling in each scan for less points.
        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(g_flatless_ds, g_flatless_ds, g_flatless_ds);
        downSizeFilter.filter(surfPointsLessFlatScanDS);
        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }

    // Transfer pcl 2 ros msgs.
    // Pub 1: full full cloud;
    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    // laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.stamp = g_cloud_input_time;
    laserCloudOutMsg.header.frame_id = "/laser_link";
    
    pubLaserCloud.publish(laserCloudOutMsg);

    // pub 2: corners/less-sharp corners
    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = g_cloud_input_time;
    cornerPointsSharpMsg.header.frame_id = "/laser_link";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);
    // less sharp corners
    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = g_cloud_input_time;
    cornerPointsLessSharpMsg.header.frame_id = "/laser_link";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    // pub 3: planers
    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = g_cloud_input_time;
    surfPointsFlat2.header.frame_id = "/laser_link";
    pubSurfPointsFlat.publish(surfPointsFlat2);
    // less flat points.
    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = g_cloud_input_time;
    surfPointsLessFlat2.header.frame_id = "/laser_link";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

    if(PUB_EACH_LINE) {
        for(int i = 0; i< g_used_scans; i++){
            sensor_msgs::PointCloud2 scanMsg;
            pcl::toROSMsg(laserCloudScans[i], scanMsg);
            scanMsg.header.stamp = g_cloud_input_time;
            scanMsg.header.frame_id = "/laser_link";
            pubEachScan[i].publish(scanMsg);
        }
    }

    std::cout << "[Regi] " << t_whole.toc() << "ms, corner : " << cornerPointsSharp.size() << "/" << cornerPointsLessSharp.size()
              << ", planer: " << surfPointsFlat.size() << "/" << surfPointsLessFlat.size() << std::endl;

    if(t_whole.toc() > 50)
        ROS_WARN("scan registration process over 100 ms");
}


int main(int argc, char **argv){

    ros::init(argc, argv, "scanRegistration");
    ros::NodeHandle nh;
    ROS_INFO("Scan registration node begin...");

    // load settings.
    nh.param<int>("scan_skip", g_scan_skip, 0); // 激光雷达的线束
    if (32 % (g_scan_skip + 1) != 0){
        ROS_ERROR_STREAM("Invalid scan_skip: "<<g_scan_skip);
    }
    g_used_scans = 32 / (g_scan_skip + 1);
    
    nh.param<int>("sector_num", g_sector_num, 6);
    nh.param<int>("sharp_num", g_sharp_num, 2);
    nh.param<int>("sharpless_num", g_sharpless_num, 20);
    nh.param<int>("flat_num", g_flat_num, 2);
    nh.param<double>("flatless_ds", g_flatless_ds, 0.4);
    nh.param<double>("min_range", g_min_range, 0.8);
    nh.param<double>("max_range", g_max_range, 10.0);
    
    ROS_WARN_STREAM("Scan Number: " << g_used_scans);
    ROS_WARN_STREAM("Sector Num: " << g_sector_num);
    ROS_WARN_STREAM("sharp: " << g_sharp_num);
    ROS_WARN_STREAM("less-sharp: " << g_sharpless_num);
    ROS_WARN_STREAM("flat: " << g_flat_num);
    ROS_WARN_STREAM("less-flat downsampling: " << g_flatless_ds);
    ROS_WARN_STREAM("range: [" << g_min_range << ", " << g_max_range << "].");

    std::string input_topic_name;
    nh.param<std::string>("topic_name", input_topic_name, "lslidar_point_cloud");
    std::cout<<"--------"<<input_topic_name<<" --------"<<std::endl;
    // sub pointcloud from lslidar
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(input_topic_name, 100, laserCloudHandler);

    // pub 5 topics.
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/lslidar_point_cloud_2", 100);
    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);
    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);
    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);
    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);
    // pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);      //~ not subscribed by anyone.

    if(PUB_EACH_LINE){
        for(int i = 0; i < g_used_scans; i++){
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
            pubEachScan.push_back(tmp);
        }
    }
    ros::spin();
    return 0;
}

