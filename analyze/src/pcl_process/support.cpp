
#include "support.h"
#include <pcl/common/distances.h>


// int Support::id_ = 0;
// void Support::detectCylinder(const CylinderParameters& cp){
//     cout << "Detect support id: " << id_ << endl;
// }


bool detectCylinder(const MyPointCloud& input_cloud, MyPointCloud& output_cylinder, const CylinderParameters& cp){
    pcl::NormalEstimation<MyPoint, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<MyPoint, pcl::Normal> seg;
    pcl::ExtractIndices<MyPoint> extract;
    pcl::search::KdTree<MyPoint>::Ptr tree(new pcl::search::KdTree<MyPoint>());
    MyPointCloud::Ptr cloud(new MyPointCloud());
    // *cloud = raw_cloud_;
    pcl::copyPointCloud(input_cloud, *cloud);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

// #ifdef DEBUG
//     cout << "Parameters-----" << endl;
//     cout << "rdius: " << cp.search_radius_ << endl
//          << "weight: " << cp.normal_distance_weight_ << endl
//          << "distance_th: " << cp.distance_threshold_ << endl
//          << "iter: " << cp.max_iteration_ << endl
//          << "radius:[" << cp.min_radius_ << ", " << cp.max_radius_ << ") " << endl;
// #endif

    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(cp.search_radius_);
    ne.compute(*cloud_normals);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(cp.normal_distance_weight_); // the weight of normals, when calculating cylinder.
    seg.setDistanceThreshold(cp.distance_threshold_);       // outliers. If larger than this value, regards as outliers.
    seg.setMaxIterations(cp.max_iteration_);
    seg.setRadiusLimits(cp.min_radius_, cp.max_radius_);
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);
    pcl::ModelCoefficients::Ptr raw_coeff (new pcl::ModelCoefficients);
    seg.segment(*inliers_cylinder, *raw_coeff);
    // coefficient = *coefficients_cylinder;
#ifdef DEBUG
    // std::cerr << "Cylinder coefficients: " << *raw_coeff << std::endl;
#endif

    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    
    output_cylinder.clear();
    extract.filter(output_cylinder);

    if (output_cylinder.empty()){
        ROS_WARN("Cannot find cylinders....");
        return false;
    }

    // calCylinderCoeff(raw_coeff); // TODO:

    return true;
}
