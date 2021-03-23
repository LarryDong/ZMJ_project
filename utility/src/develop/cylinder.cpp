
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <eigen3/Eigen/Core>

#include <gflags/gflags.h>
#include "process/defination.h"
#include "process/tool.h"

using namespace std;

DEFINE_string(file_save_support, "./support/", "all supports save file.");
DEFINE_double(cylinder_search_radius, 0.2, "cylinder_search_radius");
DEFINE_double(cylinder_normal_distance_weight, 0.05, "cylinder_normal_distance_weight");
DEFINE_double(cylinder_distance_threshould, 0.2, "cylinder_distance_threshould");
DEFINE_double(cylinder_radius_min, 0.1, "cylinder_radius_min");
DEFINE_double(cylinder_radius_max, 1.0, "cylinder_radius_max");
DEFINE_int32(cylinder_max_iteration, 1000, "max iteration for optimazaion");
DEFINE_double(support_segment_y, 2.5, "segment along y axis range");


class CylinderParameters{

public:

    CylinderParameters(){}

    CylinderParameters(const pcl::ModelCoefficients& coeff){
        cout << "pointer init..." << endl;
        point.x = coeff.values[0];
        point.y = coeff.values[1];
        point.z = coeff.values[2];
        direction.x = coeff.values[3];
        direction.y = coeff.values[4];
        direction.z = coeff.values[5];
        radius = coeff.values[6];
    }

    void showInfo(void){
        cout << "Point: " << point << ", direction: " << direction << ", radius: " << radius << endl;
    }

    MyPoint point;
    MyPoint direction;
    double radius;

};


bool extractCylinder(const MyPointCloud::Ptr input_pc, MyPointCloud& output_cylinder, pcl::ModelCoefficients& coefficient){

    
    pcl::NormalEstimation<MyPoint, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<MyPoint, pcl::Normal> seg;
    pcl::ExtractIndices<MyPoint> extract;
    pcl::search::KdTree<MyPoint>::Ptr tree(new pcl::search::KdTree<MyPoint>());
    pcl::PointCloud<MyPoint>::Ptr cloud(new pcl::PointCloud<MyPoint>);
    cloud = input_pc;

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(FLAGS_cylinder_search_radius);
    ne.compute(*cloud_normals);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(FLAGS_cylinder_normal_distance_weight);     // the weight of normals, when calculating cylinder. TODO: physical meanings
    seg.setDistanceThreshold(FLAGS_cylinder_distance_threshould);           // outliers. If larger than this value, regards as outliers.
    seg.setMaxIterations(FLAGS_cylinder_max_iteration);
    seg.setRadiusLimits(FLAGS_cylinder_radius_min, FLAGS_cylinder_radius_max);
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    coefficient = *coefficients_cylinder;

    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
    // CylinderParameters cylinder(coefficients_cylinder);
    // cylinder.showInfo();
    
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    output_cylinder.clear();
    extract.filter(output_cylinder);

    if (output_cylinder.empty()){
        ROS_WARN("Cannot find cylinders....");
        return false;
    }
    
    return true;

}



int main(int argc, char **argv){

    google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "cylinder_node");
    ros::NodeHandle nh;
    ROS_WARN("cylinder_node node begin...");

    std::vector<ros::Publisher> pubEachSupport;
    ros::Publisher pubFullPc = nh.advertise<sensor_msgs::PointCloud2>("/full_pointcloud", 1);
    ros::Publisher pubCylinder = nh.advertise<sensor_msgs::PointCloud2>("/cylinder", 1);
    ros::Publisher pubCylinderAxisPoints = nh.advertise<sensor_msgs::PointCloud2>("/cylinder_axis_points", 1);
    ros::Publisher drawMarkers = nh.advertise<visualization_msgs::MarkerArray>("/cyliner_markers", 1);

    MyPointCloud full_pc;
    std::vector<MyPointCloud::Ptr> v_support;
    
    for(int i=0; i<100; ++i){
        string filename = FLAGS_file_save_support + "support_" + to_string(i) + ".pcd";
        MyPointCloud::Ptr pc (new MyPointCloud());
        if (pcl::io::loadPCDFile<MyPoint>(filename, *pc) == -1){
            cout << "[Error]. Cannot open '" << filename << "'. " << endl;
            break;
        }
        v_support.push_back(pc);
        full_pc += *pc;
    }
    cout << "Load total: " << v_support.size() << " supports;" << endl;
    for (int i = 0; i < v_support.size(); i++){
        ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/support_" + std::to_string(i), 1);
        pubEachSupport.push_back(tmp);
    }
    vector<double> distance = {-0.37, -3.44, -6.54, -9.40, -12.25};       // from car_path. 


    MyPointCloud all_cylinder_pc;
    std::vector<pcl::ModelCoefficients> v_coefficients;
    std::vector<CylinderParameters> cylinders;

    // extract cylinder from each half.
    for(int i=0; i<v_support.size(); ++i){
        MyPointCloud::Ptr one_support(new MyPointCloud()), left_half(new MyPointCloud()), right_half(new MyPointCloud());
        MyPointCloud one_cylinder;
        pcl::ModelCoefficients coeff;

        one_support = v_support[i];
        pcl::PassThrough<pcl::PointXYZ> pass;       // seg half.
        pass.setInputCloud(one_support);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(distance[i] - FLAGS_support_segment_y / 2, distance[i]);
        pass.filter(*left_half);
        pass.setFilterLimits(distance[i], distance[i] + FLAGS_support_segment_y / 2);
        pass.filter(*right_half);
        
        if(extractCylinder(left_half, one_cylinder, coeff)){
            all_cylinder_pc += one_cylinder;
            v_coefficients.push_back(coeff);
            // CylinderParameters ctmp(coeff);
            cylinders.push_back(coeff);
        }
        if(extractCylinder(right_half, one_cylinder, coeff)){
            all_cylinder_pc += one_cylinder;
            v_coefficients.push_back(coeff);
            // cylinders.push_back(CylinderParameters(coeff));
            cylinders.push_back(coeff);
        }
    }

    
    ROS_INFO_STREAM("Total cylinder: "<< v_coefficients.size());
    const int cylinder_number = v_coefficients.size();


    visualization_msgs::MarkerArray markers;
    for(int i=0; i<cylinders.size(); ++i){
        // ros::Publisher tmp = nh.advertise<visualization_msgs::>("/cylinder_marker_" + std::to_string(i), 1);
        // drawEachCylinder.push_back(tmp);

        // extract each cylinder
        CylinderParameters cp = cylinders[i];
        visualization_msgs::Marker marker;
        

        // calculate orientation:
        // R = [x, y, z]; z = direction.norm(); x, y orth. to z.
        // TODO:
        
        Eigen::Vector3d x, y, z;
        z(0) = cp.direction.x;
        z(1) = cp.direction.y;
        z(2) = cp.direction.z;
        z.normalize();
        cout << "No. " << i << ",Z: " << z.transpose() << endl;
        // z \cdot x = 0 --> z0x0+z1x1+z2x2=0 --> let x2=0, x0/x1 can be calculated.
        double b_over_a = z(2) / z(1);                      // z(2)/z(1)
        x(0) = sqrtf(1 / (1 + b_over_a * b_over_a));
        x(1) = -b_over_a * x(0);
        x(2) = 0;
        x.normalize();

        y = z.cross(x);
        y.normalize();

        Eigen::Matrix3d R;
        R.col(0) = x;
        R.col(1) = y;
        R.col(2) = z;
        cout << "Rotation matrix: " << R << endl;
        Eigen::Quaterniond q(R);

        marker.id = i;       // Important! Not overwirte the last marker.
        marker.header.frame_id="/laser_link";
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = cp.point.x;
        marker.pose.position.y = cp.point.y;
        marker.pose.position.z = cp.point.z;
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.scale.x = cp.radius * 2;
        marker.scale.y = cp.radius * 2;
        marker.scale.z = 5;
        marker.color.b = 0.0; 
        marker.color.g = 1.0;
        marker.color.r = 0.0;
        marker.color.a = 0.5;

        // v_markers.push_back(marker);
        markers.markers.push_back(marker);
    }
    

    ros::Rate r(10);
    while(ros::ok()){

        for (int i = 0; i < pubEachSupport.size(); ++i){
            pubEachSupport[i].publish(tool::pointCloud2RosMsg(*v_support[i]));
        }
        pubFullPc.publish(tool::pointCloud2RosMsg(full_pc));
        pubCylinder.publish(tool::pointCloud2RosMsg(all_cylinder_pc));
        
        // for(int i=0; i<cylinders.size(); ++i){
        //     drawEachCylinder[i].publish(v_markers[i]);
        // }
        drawMarkers.publish(markers);

        ros::spinOnce();
        r.sleep();
    }

    // cout << "Cylinder test..." << endl;
    return 0;
}
