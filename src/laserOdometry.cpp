
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Dense>

#include <cmath>
#include <mutex>
#include <queue>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "lidarFactor.hpp"

#define DISTORTION 0
constexpr double SCAN_PERIOD = 0.1; // 激光雷达的频率，0.1s
constexpr double DISTANCE_SQ_THRESHOLD = 25; // 找最近点的距离平方的阈值
constexpr double NEARBY_SCAN = 2.5; // 找点时最远激光层的阈值


std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
// pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());

// pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
// pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());


Eigen::Quaterniond q_w_curr(1, 0, 0, 0); // 激光雷达在世界坐标系中的位姿，用四元数来表示方向
Eigen::Vector3d t_w_curr(0, 0, 0);
double para_q[4] = {0, 0, 0, 1}; // ceres用来优化时的数组，四元数
double para_t[3] = {0, 0, 0}; // 平移
Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

std::mutex mBuf;


// undistort lidar point 将激光点转换到这一帧起始时刻的坐标系下（也相当于是上一帧的末尾）
void TransformToStart(PointType const *const pi, PointType *const po){
    //interpolation ratio
    double s;
    if (DISTORTION)
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD; // 根据在这一帧中的时间（intensity的小数部分）进行位姿转换（去畸变）
    else
        s = 1.0;
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

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2){
    mBuf.lock();
    fullPointsBuf.push(laserCloudFullRes2);
    mBuf.unlock();
}



int main(int argc, char **argv){
    ros::init(argc, argv, "laserOdometry");
    ros::NodeHandle nh;

    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, laserCloudSharpHandler);
    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, laserCloudLessSharpHandler);
    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud_2", 100, laserCloudFullResHandler);

    // ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);
    // ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);
    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);
    nav_msgs::Path laserPath;

    // some global variables fomr A-LOAM
    int corner_correspondence = 0;
    bool system_inited = false;
    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();
        if(cornerSharpBuf.empty() || cornerLessSharpBuf.empty() || fullPointsBuf.empty())
            continue;
        
        // step 1. Check time sync.
        double timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
        double timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
        double timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();
        if (timeCornerPointsSharp != timeLaserCloudFullRes || timeCornerPointsLessSharp != timeLaserCloudFullRes){
            ROS_ERROR("unsync message!");
            continue;
        }

        // STEP 2. To ros msgs.
        mBuf.lock();
        cornerPointsSharp->clear();
        pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
        cornerSharpBuf.pop();

        cornerPointsLessSharp->clear();
        pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
        cornerLessSharpBuf.pop();

        laserCloudFullRes->clear();
        pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
        fullPointsBuf.pop();
        mBuf.unlock();

        // STEP 3. init and running
        if(!system_inited){         // if not inited, jump the 'else' part for initialization
            system_inited = true;
        }
        else{
            int cornerPointsSharpNum = cornerPointsSharp->points.size();
            for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter) {       // opitmize twice
                corner_correspondence = 0;
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1); // 定义一个0.1阈值的huber核函数，优化时抑制离群点用
                ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
                ceres::Problem::Options problem_options;
                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(para_q, 4, q_parameterization); // 设定优化参数的自变量，q_curr_last(x, y, z, w)和t_curr_last，四元数旋转和平移
                problem.AddParameterBlock(para_t, 3);

                pcl::PointXYZI pointSel;
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;
                for(int i=0; i<cornerPointsSharpNum; ++i){
                    TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);
                    kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                    int closestPointInd = -1, minPointInd2 = -1;
                    if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD){ // 如果最近点距离的平方小于25
                        closestPointInd = pointSearchInd[0];          // 最近点的index
                        int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity); // 最近点在scan第几层（intensity的整数部分）
                        double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                        // search 2 nearest edge-point at different layers.
                        for(int j=closestPointInd+1; j<(int)laserCloudCornerLast->points.size(); ++j){
                            if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                                continue;
                            // if not in nearby scans, end the loop 不要在太远的scan层
                            if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                break;
                            auto pt= laserCloudCornerLast->points[j];
                            // double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                            //                         (laserCloudCornerLast->points[j].x - pointSel.x) +
                            //                     (laserCloudCornerLast->points[j].y - pointSel.y) *
                            //                         (laserCloudCornerLast->points[j].y - pointSel.y) +
                            //                     (laserCloudCornerLast->points[j].z - pointSel.z) *
                            //                         (laserCloudCornerLast->points[j].z - pointSel.z);
                            double pointSqDis = (pt.x - pointSel.x) * (pt.x - pointSel.x) + (pt.y - pointSel.y) * (pt.y - pointSel.y) + (pt.z - pointSel.z) * (pt.z - pointSel.z);
                            if (pointSqDis < minPointSqDis2) {      // find nearer point
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }
                        // search in the direction of decreasing scan line
                        for (int j = closestPointInd - 1; j >= 0; --j) { // 跟上面一样，只不过这回从反方向遍历
                            // if in the same scan line, continue
                            if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                                continue;
                            // if not in nearby scans, end the loop
                            if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                break;
                            // double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                            //                         (laserCloudCornerLast->points[j].x - pointSel.x) +
                            //                     (laserCloudCornerLast->points[j].y - pointSel.y) *
                            //                         (laserCloudCornerLast->points[j].y - pointSel.y) +
                            //                     (laserCloudCornerLast->points[j].z - pointSel.z) *
                            //                         (laserCloudCornerLast->points[j].z - pointSel.z);
                            auto pt= laserCloudCornerLast->points[j];
                            double pointSqDis = (pt.x - pointSel.x) * (pt.x - pointSel.x) + (pt.y - pointSel.y) * (pt.y - pointSel.y) + (pt.z - pointSel.z) * (pt.z - pointSel.z);

                            if (pointSqDis < minPointSqDis2) { // find nearer point
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }
                    }
                    // 找到两个不在同一层上的Edge Points之后，计算点到这两个Edge Points拟合的直线上的距离，作为误差函数
                    if (minPointInd2 >= 0) {            // both closestPointInd and minPointInd2 is valid
                        Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,cornerPointsSharp->points[i].y,cornerPointsSharp->points[i].z);
                        Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                    laserCloudCornerLast->points[closestPointInd].y,
                                                    laserCloudCornerLast->points[closestPointInd].z);
                        Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                    laserCloudCornerLast->points[minPointInd2].y,
                                                    laserCloudCornerLast->points[minPointInd2].z);

                        double s;
                        if (DISTORTION) // 如果激光点云没有做过去畸变，这里用s来计算需要插值的比例（时间）
                            s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
                        else
                            s = 1.0;
                        // 在cost_function里定义误差函数和求导相关
                        ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                        problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                        corner_correspondence++;
                    }
                }
                ROS_INFO_STREAM("Corner correspondence: " << corner_correspondence);        // about 270
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
                options.minimizer_progress_to_stdout = false;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
            }

            t_w_curr = t_w_curr + q_w_curr * t_last_curr; // 相当于是t_w_curr = t_w_last + q_w_last * t_last_curr
            q_w_curr = q_w_curr * q_last_curr;            // 相当于是q_w_curr = q_w_last * q_last_curr
            ROS_INFO_STREAM("T: [" << t_w_curr.x() << ", " << t_w_curr.y() << ", " << t_w_curr.z() << "].");
            // ROS_INFO_STREAM("111");
        }
        
        // STEP 5. Inited/update, to "last"
        pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
        cornerPointsLessSharp = laserCloudCornerLast;
        laserCloudCornerLast = laserCloudTemp;
        kdtreeCornerLast->setInputCloud(laserCloudCornerLast);


        // STEP 6. Pub mseeagse
        nav_msgs::Odometry laserOdometry;
        laserOdometry.header.frame_id = "/laser_link";
        laserOdometry.child_frame_id = "/do_not_know_what";
        laserOdometry.header.stamp = ros::Time().fromSec(timeLaserCloudFullRes);
        laserOdometry.pose.pose.orientation.x = q_w_curr.x();
        laserOdometry.pose.pose.orientation.y = q_w_curr.y();
        laserOdometry.pose.pose.orientation.z = q_w_curr.z();
        laserOdometry.pose.pose.orientation.w = q_w_curr.w();
        laserOdometry.pose.pose.position.x = t_w_curr.x();
        laserOdometry.pose.pose.position.y = t_w_curr.y();
        laserOdometry.pose.pose.position.z = t_w_curr.z();
        pubLaserOdometry.publish(laserOdometry); // 发布激光里程计的位姿

        geometry_msgs::PoseStamped laserPose;
        laserPose.header = laserOdometry.header;
        laserPose.pose = laserOdometry.pose.pose;
        laserPath.header.stamp = laserOdometry.header.stamp;
        laserPath.poses.push_back(laserPose);
        laserPath.header.frame_id = "/laser_link";
        pubLaserPath.publish(laserPath); // 发布激光里程计的路径

        rate.sleep();
    }
    return 0;
}


