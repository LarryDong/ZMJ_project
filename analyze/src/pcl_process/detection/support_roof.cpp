
#include "support_roof.h"


#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/distances.h>


int SupportRoof::marker_id_ = 0;

void SupportRoof::detectRoof(const PlaneParameters& pp){

    // Step 1. Filtering.
    // pass filter
    pcl::PassThrough<MyPoint> pass;
    MyPointCloud::Ptr pc(new MyPointCloud);
    *pc = raw_cloud_;
    pass.setInputCloud(pc);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(pp.x_min_, pp.x_max_);
    pass.filter(*pc);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(pp.z_min_, pp.z_max_);
    pass.filter(*pc);

    // statictical filter
    int K = 50;
    double std = 1;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(pc);
    sor.setMeanK(K);
    sor.setStddevMulThresh(std);
    sor.filter(*pc);
    plane_cloud_ = *pc;

    // Step 2. Segment plane by SAC.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(pc);
    seg.segment(*inliers, *coefficients);

    // extract plane pointcloud;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(pc);
    extract.setIndices(inliers);
    extract.setNegative(false); //如果设为true,可以提取指定index之外的点云
    extract.filter(*pc);
    plane_cloud_ = *pc;

    // Step 3. Calcualte parameters of plane.
    // save plane parameters;
    auto v = coefficients->values;
    plane_normal_(0) = v[0];
    plane_normal_(1) = v[1];
    plane_normal_(2) = v[2];
    plane_normal_ = v[2] < 0 ? plane_normal_ * (-1) : plane_normal_;

    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> moment;
    moment.setInputCloud(pc);
    moment.compute();
    moment.getMassCenter(plane_center_);
    
    // calcualte size;
    MyPoint min_p, max_p;
    pcl::getMinMax3D(plane_cloud_, min_p, max_p);


    // calculate directions
    Eigen::Vector3f x, y, z;
    z = plane_normal_;
    z.normalize();

    y = Eigen::Vector3f(0, -1, 0);
    y = y - z.dot(y) * z;
    y.normalize();

    x = y.cross(z);
    x.normalize();

    Eigen::Matrix3f R;
    R.col(0) = x;
    R.col(1) = y;
    R.col(2) = z;
    Eigen::Quaternionf q_tmp(R);
    q_ = q_tmp;

    cout << "Plane find. Center: " << plane_center_.transpose() << ", normal: " << plane_normal_.transpose() << endl;

#ifdef DEBUG
    std::cerr << "Plane coefficients (abc, d): " << v[0] << ", " << v[1] << ", " << v[2] << ", " << v[3] << endl;
#endif
}


visualization_msgs::Marker SupportRoof::createMarker(Eigen::Vector4f color){
    visualization_msgs::Marker marker;
    marker.header.frame_id="/laser_link";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.b = color(0);
    marker.color.g = color(1);
    marker.color.r = color(2);
    marker.color.a = color(3);
    marker.id = marker_id_++;               // must be unique id
    marker.pose.position.x = plane_center_(0);
    marker.pose.position.y = plane_center_(1);
    marker.pose.position.z = plane_center_(2);
    marker.pose.orientation.x = q_.x();
    marker.pose.orientation.y = q_.y();
    marker.pose.orientation.z = q_.z();
    marker.pose.orientation.w = q_.w();
    plane_height_ = 2.5;        // from settings.
    plane_width_ = 2;
    marker.scale.x = plane_height_;
    marker.scale.y = plane_width_;
    marker.scale.z = 0.01;
    return marker;
}