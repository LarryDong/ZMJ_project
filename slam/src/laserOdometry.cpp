
#include <cmath>
#include <mutex>
#include <queue>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <eigen3/Eigen/Dense>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "lidarFactor.hpp"


// Unsed parameters.
// #define DISTORTION 0 // 表示激光点云是否已经被去过畸变
// constexpr double SCAN_PERIOD = 0.1;         // 激光雷达的频率，0.1s
// int skipFrameNum = 5; // 通过launch文件进行设置，控制输出的频率


// global settings.
constexpr double DISTANCE_SQ_THRESHOLD = 25; // 找最近点的距离平方的阈值
constexpr double NEARBY_SCAN = 2.5; // 找点时最远激光层的阈值
int g_SKIP_FRAME = 1;

// global variables
int corner_correspondence = 0, plane_correspondence = 0;
int laserCloudCornerLastNum = 0, laserCloudSurfLastNum = 0;
int g_skip_counter = 0;

bool systemInited = false;
double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());

pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());


// Transformation from current frame to world frame
Eigen::Quaterniond q_w_curr(1, 0, 0, 0); // 激光雷达在世界坐标系中的位姿，用四元数来表示方向
Eigen::Vector3d t_w_curr(0, 0, 0);

// q_curr_last(x, y, z, w), t_curr_last
double para_q[4] = {0, 0, 0, 1}; // ceres用来优化时的数组，四元数
double para_t[3] = {0, 0, 0}; // 平移

Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);     // Eigen::Map: shared.
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;
std::mutex mBuf;



// debug parameters;
double gt_getData=0, gt_associate = 0, gt_solve = 0, gt_pub = 0, gt_total = 0;

// undistort lidar point 将激光点转换到这一帧起始时刻的坐标系下（也相当于是上一帧的末尾）
void TransformToStart(PointType const *const pi, PointType *const po){
    // //interpolation ratio
    double s = 1.0;
    // if (DISTORTION)
    //     s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD; // 根据在这一帧中的时间（intensity的小数部分）进行位姿转换（去畸变）
    // else
    //     s = 1.0;

    // 通过线性插值获得这一时刻激光雷达相对上一帧的位姿
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    Eigen::Vector3d t_point_last = s * t_last_curr;
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last; // 将激光点通过这一时刻的位姿转换到这一帧的起始坐标系下，去畸变

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2){
    mBuf.lock();
    cornerSharpBuf.push(cornerPointsSharp2);
    mBuf.unlock();
}
void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2){
    mBuf.lock();
    cornerLessSharpBuf.push(cornerPointsLessSharp2);
    mBuf.unlock();
}
void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2){
    mBuf.lock();
    surfFlatBuf.push(surfPointsFlat2);
    mBuf.unlock();
}
void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2){
    mBuf.lock();
    surfLessFlatBuf.push(surfPointsLessFlat2);
    mBuf.unlock();
}
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2){
    mBuf.lock();
    fullPointsBuf.push(laserCloudFullRes2);
    mBuf.unlock();
}


int main(int argc, char **argv){

    ros::init(argc, argv, "laserOdometry");
    ros::NodeHandle nh;

    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud_2", 100, laserCloudFullResHandler);
    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, laserCloudSharpHandler);
    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, laserCloudLessSharpHandler);
    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, laserCloudFlatHandler);
    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, laserCloudLessFlatHandler);
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);
    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/lslidar_point_cloud_3", 100);
    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);

    nav_msgs::Path laserPath;

    ros::Rate rate(100);

    while (ros::ok()){
        rate.sleep();
        ros::spinOnce();

        // STEP 1. Check sync.
        if (cornerSharpBuf.empty() || cornerLessSharpBuf.empty() || surfFlatBuf.empty() || surfLessFlatBuf.empty() || fullPointsBuf.empty())
            continue;
        timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
        timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
        timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
        timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
        timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();
        if (timeCornerPointsSharp != timeLaserCloudFullRes || timeCornerPointsLessSharp != timeLaserCloudFullRes ||
            timeSurfPointsFlat != timeLaserCloudFullRes || timeSurfPointsLessFlat != timeLaserCloudFullRes) {
            printf("unsync messeage!");
            ROS_BREAK();
        }

        TicToc t_total;

        // STEP 2. Get data.
        TicToc t_getData;
        mBuf.lock();
        cornerPointsSharp->clear();
        pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
        cornerSharpBuf.pop();

        cornerPointsLessSharp->clear();
        pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
        cornerLessSharpBuf.pop();

        surfPointsFlat->clear();
        pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
        surfFlatBuf.pop();

        surfPointsLessFlat->clear();
        pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
        surfLessFlatBuf.pop();

        laserCloudFullRes->clear();
        pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
        fullPointsBuf.pop();
        mBuf.unlock();
        gt_getData = t_getData.toc();

        // STEP 3. Odom running.
        TicToc t_whole;
        if (!systemInited){ // 第一帧信息存下来作为lastframe，这样之后的信息才可以和上一帧匹配得到位姿
            systemInited = true;
            std::cout << "Initialization finished \n";
        }
        else{
            TicToc t_associate;

            int cornerPointsSharpNum = cornerPointsSharp->points.size();
            int surfPointsFlatNum = surfPointsFlat->points.size();

            for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter){   // optimize 2 times
            
                corner_correspondence = 0;
                plane_correspondence = 0;

                // STEP 3.1. ceres settings.
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization(); 
                ceres::Problem::Options problem_options;
                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(para_q, 4, q_parameterization); 
                problem.AddParameterBlock(para_t, 3);

                pcl::PointXYZI pointSel;
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;

                // find correspondence for corner features
                for (int i = 0; i < cornerPointsSharpNum; ++i) // 所有的Edge Points
                {
                    TransformToStart(&(cornerPointsSharp->points[i]), &pointSel); // 将激光点转换到这一帧起始时刻的坐标系下（根据时间线性插值粗略的去畸变）
                    kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                    int closestPointInd = -1, minPointInd2 = -1;
                    if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD) {
                        closestPointInd = pointSearchInd[0];
                        int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity); // 最近点在scan第几层（intensity的整数部分）
                        double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                        // search in the direction of increasing scan line
                        for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j){
                            // if in the same scan line, continue
                            if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                                continue;
                            // if not in nearby scans, end the loop 
                            if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                break;
                            auto pt = laserCloudCornerLast->points[j];
                            double pointSqDis = (pt.x - pointSel.x) *(pt.x - pointSel.x) + (pt.y - pointSel.y)*(pt.y - pointSel.y) + (pt.z - pointSel.z)*(pt.z - pointSel.z);
                            if (pointSqDis < minPointSqDis2){
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }

                        // search in the direction of decreasing scan line
                        for (int j = closestPointInd - 1; j >= 0; --j){ // 跟上面一样，只不过这回从反方向遍历
                            // if in the same scan line, continue
                            if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                                continue;
                            // if not in nearby scans, end the loop
                            if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                break;
                            auto pt = laserCloudCornerLast->points[j];
                            double pointSqDis = (pt.x - pointSel.x) *(pt.x - pointSel.x) + (pt.y - pointSel.y)*(pt.y - pointSel.y) + (pt.z - pointSel.z)*(pt.z - pointSel.z);
                            if (pointSqDis < minPointSqDis2){
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }
                    }
                    // 找到两个不在同一层上的Edge Points之后，计算点到这两个Edge Points拟合的直线上的距离，作为误差函数
                    if (minPointInd2 >= 0) {                // both closestPointInd and minPointInd2 is valid
                        Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x, cornerPointsSharp->points[i].y, cornerPointsSharp->points[i].z);
                        Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x, laserCloudCornerLast->points[closestPointInd].y, laserCloudCornerLast->points[closestPointInd].z);
                        Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x, laserCloudCornerLast->points[minPointInd2].y, laserCloudCornerLast->points[minPointInd2].z);
                        ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, 1.0);
                        problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                        corner_correspondence++;
                    }
                }

                // find correspondence for plane features
                for (int i = 0; i < surfPointsFlatNum; ++i) {
                    // 跟Edge Points处理类似，先根据这一帧的位姿增量进行粗略去畸变
                    TransformToStart(&(surfPointsFlat->points[i]), &pointSel);      // ~ useless.
                    kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                    int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                    if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD){
                        closestPointInd = pointSearchInd[0];
                        // get closest point's scan ID
                        int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                        double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;
                        // search in the direction of increasing scan line
                        for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j){
                            // if not in nearby scans, end the loop
                            if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                break;
                            auto pt = laserCloudCornerLast->points[j];
                            double pointSqDis = (pt.x - pointSel.x) * (pt.x - pointSel.x) + (pt.y - pointSel.y) * (pt.y - pointSel.y) + (pt.z - pointSel.z) * (pt.z - pointSel.z);

                            // if in the same or lower scan line 其中一个次最近点需要在同一层或更底层
                            if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2){
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                            // if in the higher scan line 另一个次最近点需要在更高层，这样选出来的3个点不容易共线，可以更好的拟合出一个平面
                            else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3){
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }

                        // search in the direction of decreasing scan line
                        for (int j = closestPointInd - 1; j >= 0; --j){
                            // if not in nearby scans, end the loop
                            if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                break;
                            auto pt = laserCloudCornerLast->points[j];
                            double pointSqDis = (pt.x - pointSel.x) * (pt.x - pointSel.x) + (pt.y - pointSel.y) * (pt.y - pointSel.y) + (pt.z - pointSel.z) * (pt.z - pointSel.z);
                            // if in the same or higher scan line
                            if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2){
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                            else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3){
                                minPointSqDis3 = pointSqDis;
                                minPointInd3 = j;
                            }
                        }

                        if (minPointInd2 >= 0 && minPointInd3 >= 0){ // 找到了3个点，构造点到平面距离的误差函数
                            Eigen::Vector3d curr_point(surfPointsFlat->points[i].x, surfPointsFlat->points[i].y, surfPointsFlat->points[i].z);
                            Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x, laserCloudSurfLast->points[closestPointInd].y, laserCloudSurfLast->points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x, laserCloudSurfLast->points[minPointInd2].y, laserCloudSurfLast->points[minPointInd2].z);
                            Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x, laserCloudSurfLast->points[minPointInd3].y, laserCloudSurfLast->points[minPointInd3].z);
                            ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, 1.0);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            plane_correspondence++;
                        }
                    }
                }

                gt_associate = t_associate.toc();

                if ((corner_correspondence + plane_correspondence) < 10) // 提示找到的匹配点数量过少
                    ROS_WARN_STREAM("less correspondence! corner/plan: " << corner_correspondence << " / " << plane_correspondence);

                TicToc t_solver;
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4; // 在非线性优化求解中最多进行几次迭代
                options.minimizer_progress_to_stdout = false;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
                gt_solve = t_solver.toc();
            }
            // 更新激光雷达在世界坐标系下的位姿
            t_w_curr = t_w_curr + q_w_curr * t_last_curr;   // t_w_curr = t_w_last + q_w_last * t_last_curr
            q_w_curr = q_w_curr * q_last_curr;              // q_w_curr = q_w_last * q_last_curr
        }

        // publish odometry
        nav_msgs::Odometry laserOdometry;
        laserOdometry.header.frame_id = "/laser_link";
        laserOdometry.child_frame_id = "/laser_link";
        laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
        laserOdometry.pose.pose.orientation.x = q_w_curr.x();
        laserOdometry.pose.pose.orientation.y = q_w_curr.y();
        laserOdometry.pose.pose.orientation.z = q_w_curr.z();
        laserOdometry.pose.pose.orientation.w = q_w_curr.w();
        laserOdometry.pose.pose.position.x = t_w_curr.x();
        laserOdometry.pose.pose.position.y = t_w_curr.y();
        laserOdometry.pose.pose.position.z = t_w_curr.z();
        pubLaserOdometry.publish(laserOdometry);
        // pub path
        geometry_msgs::PoseStamped laserPose;
        laserPose.header = laserOdometry.header;
        laserPose.pose = laserOdometry.pose.pose;
        laserPath.header.stamp = laserOdometry.header.stamp;
        laserPath.poses.push_back(laserPose);
        laserPath.header.frame_id = "/laser_link";
        pubLaserPath.publish(laserPath); // 发布激光里程计的路径

        // update/initialize: curr->last
        pcl::PointCloud<PointType>::Ptr laserCloudTemp;
        laserCloudTemp = cornerPointsLessSharp;     // all corners(sharp/less-sharp) are used
        cornerPointsLessSharp = laserCloudCornerLast;
        laserCloudCornerLast = laserCloudTemp;
        laserCloudCornerLastNum = laserCloudCornerLast->points.size();
        kdtreeCornerLast->setInputCloud(laserCloudCornerLast); // 构建kd树，用来找最近点用

        laserCloudTemp = surfPointsLessFlat;        // all surf(down-sampled) are used
        surfPointsLessFlat = laserCloudSurfLast;
        laserCloudSurfLast = laserCloudTemp;
        laserCloudSurfLastNum = laserCloudSurfLast->points.size();
        kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

        // pub last data.
        g_skip_counter++;
        if(g_skip_counter % g_SKIP_FRAME == 0){
            sensor_msgs::PointCloud2 laserCloudCornerLast2;
            pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
            laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
            laserCloudCornerLast2.header.frame_id = "/laser_link";
            pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

            sensor_msgs::PointCloud2 laserCloudSurfLast2;
            pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
            laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
            laserCloudSurfLast2.header.frame_id = "/laser_link";
            pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

            sensor_msgs::PointCloud2 laserCloudFullRes3;
            pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
            laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
            laserCloudFullRes3.header.frame_id = "/laser_link";
            pubLaserCloudFullRes.publish(laserCloudFullRes3);
            g_skip_counter = 0;
        }

        gt_total = t_total.toc();
        ROS_INFO_STREAM("Time: "<< gt_total << ". getData: " << gt_getData << ", associate: " << gt_associate
                                << ", optimize(num): " << gt_solve << "(" << corner_correspondence << "/" << plane_correspondence << ")");
    }

    
    return 0;
}