
#include "support_cylinder.h"
#include <pcl/common/distances.h>



int Cylinder::marker_id_ = 0;

bool Cylinder::detectCylinder(const CylinderParameters& cp){
    pcl::NormalEstimation<MyPoint, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<MyPoint, pcl::Normal> seg;
    pcl::ExtractIndices<MyPoint> extract;
    pcl::search::KdTree<MyPoint>::Ptr tree(new pcl::search::KdTree<MyPoint>());
    MyPointCloud::Ptr cloud(new MyPointCloud);
    // *cloud = raw_cloud_;
    pcl::copyPointCloud(raw_cloud_, *cloud);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

#ifdef DEBUG_OUTPUT
    cout << "Parameters-----" << endl;
    cout << "rdius: " << cp.search_radius_ << endl
         << "weight: " << cp.normal_distance_weight_ << endl
         << "distance_th: " << cp.distance_threshold_ << endl
         << "iter: " << cp.max_iteration_ << endl
         << "radius:[" << cp.min_radius_ << ", " << cp.max_radius_ << ") " << endl;
#endif

    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(cp.search_radius_);
    ne.compute(*cloud_normals);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(cp.normal_distance_weight_); // the weight of normals, when calculating cylinder. TODO: physical meanings
    seg.setDistanceThreshold(cp.distance_threshold_);       // outliers. If larger than this value, regards as outliers.
    seg.setMaxIterations(cp.max_iteration_);
    seg.setRadiusLimits(cp.min_radius_, cp.max_radius_);
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);
    pcl::ModelCoefficients::Ptr raw_coeff (new pcl::ModelCoefficients);
    seg.segment(*inliers_cylinder, *raw_coeff);

    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(cylinder_cloud_);

    if (cylinder_cloud_.empty()){
        ROS_WARN("Cannot find cylinders....");
        is_find_ = false;
        return false;
    }

    is_find_ = true;

    // calculate coeff
    calCylinderCoeff(raw_coeff);        // get cylinder_center_;
    auto v = raw_coeff->values;
    cylinder_dir_ = Eigen::Vector3d(v[3], v[4], v[5]);

    // get none-cylinder cloud
    extract.setNegative(true);
    extract.filter(non_cylinder_cloud_);

    return true;
}


void Cylinder::calCylinderCoeff(pcl::ModelCoefficients::Ptr raw_coeff){
    
    cylinder_filter(cylinder_cloud_);           // filter cylinder. make sure no outliers
    MyPoint min_p, max_p;
    pcl::getMinMax3D(cylinder_cloud_, min_p, max_p);

    auto v = raw_coeff->values;             // coeff: [0:2], any point on axis; [3:5], axis direction; [6], radius.
    Eigen::Vector3d any_point, delta;       // any_point + delta = center
    any_point(0) = v[0];
    any_point(1) = v[1];
    any_point(2) = v[2];
    
    delta(2) = (max_p.z + min_p.z) / 2 - v[2];
    delta(0) = delta(2) * (v[3] / v[5]);    // tangent.
    delta(1) = delta(2) * (v[4] / v[5]);

    cylinder_center_ = any_point + delta;
    cylinder_length_ = max_p.z - min_p.z;
    cylinder_radius_ = v[6];

    // calculate quart. First, calculate axis xyz. 
    Eigen::Vector3d x, y, z;
    z(0) = v[3];
    z(1) = v[4];
    z(2) = v[5];
    z.normalize();

    // x1^2 + x2^2 + x3^2 = 1, x \cdot z = 0; set x3=0, calcaulate x1 and x2
    double b_over_a = z(2) / z(1);      // 
    x(0) = sqrtf(1 / (1 + b_over_a * b_over_a));    
    x(1) = -b_over_a * x(0);
    x(2) = 0;                           // x(2) is free-variable
    x.normalize();

    y = z.cross(x);
    y.normalize();

    Eigen::Matrix3d R;
    R.col(0) = x;
    R.col(1) = y;
    R.col(2) = z;
    Eigen::Quaterniond q_tmp(R);
    q_ = q_tmp;
}

#if 0
visualization_msgs::Marker Cylinder::calMarker(double b, double g, double r, double a){
    visualization_msgs::Marker marker;
    marker.header.frame_id="/laser_link";
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.b = b;
    marker.color.g = g;
    marker.color.r = r;
    marker.color.a = a;
    marker.id = marker_id_++;               // must be unique id
    marker.pose.position.x = cylinder_center_(0);
    marker.pose.position.y = cylinder_center_(1);
    marker.pose.position.z = cylinder_center_(2);
    marker.pose.orientation.x = q_.x();
    marker.pose.orientation.y = q_.y();
    marker.pose.orientation.z = q_.z();
    marker.pose.orientation.w = q_.w();
    marker.scale.x = cylinder_radius_ * 2;
    marker.scale.y = cylinder_radius_ * 2;
    marker.scale.z = cylinder_length_;
    return marker;
}
#endif

void Cylinder::cylinder_filter(MyPointCloud& pc){
    int K = 50;
    double std = 1;                 // 1 sigma
    pcl::StatisticalOutlierRemoval<MyPoint> sor;
    MyPointCloud::Ptr pt(new MyPointCloud);
    *pt = pc;
    sor.setInputCloud(pt);
    sor.setMeanK(K);
    sor.setStddevMulThresh(std);
    sor.filter(pc);
}

