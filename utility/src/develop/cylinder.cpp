
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// #include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>

#include <eigen3/Eigen/Core>

#include <gflags/gflags.h>
#include "process/defination.h"
#include "process/tool.h"
#include "process/support_cylinder.h"

#include <pcl/common/distances.h>


using namespace std;

DEFINE_string(file_save_support, "./support/", "all supports save file.");
DEFINE_double(cylinder_search_radius, 0.2, "cylinder_search_radius");
DEFINE_double(cylinder_normal_distance_weight, 0.05, "cylinder_normal_distance_weight");
DEFINE_double(cylinder_distance_threshould, 0.2, "cylinder_distance_threshould");
DEFINE_double(cylinder_radius_min, 0.1, "cylinder_radius_min");
DEFINE_double(cylinder_radius_max, 1.0, "cylinder_radius_max");
DEFINE_int32(cylinder_max_iteration, 1000, "max iteration for optimazaion");
DEFINE_double(support_segment_y, 2.5, "segment along y axis range");

void base_process(const MyPointCloud& pc_in, MyPointCloud& pc_out);

Eigen::Matrix4f findModelInScene(const MyPointCloud& cloud_model, const MyPointCloud& cloud_scene, Eigen::Vector3f init_T);

int g_counter = 0;
visualization_msgs::Marker createBaseMarker(const Eigen::Matrix4f& pose, double b=0.8, double g=0.8, double r=0, double a=0.8){
    visualization_msgs::Marker marker;
    marker.header.frame_id="/laser_link";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource="file:/home/larrydong/base_surf.dae";
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.b = b;
    marker.color.g = g;
    marker.color.r = r;
    marker.color.a = a;
    marker.id = g_counter++;               // must be unique id
    marker.pose.position.x = pose(0,3);
    marker.pose.position.y = pose(1,3);
    marker.pose.position.z = pose(2,3);
    Eigen::Matrix3d R = pose.topLeftCorner(3,3).cast<double>();
    Eigen::Quaterniond q(R);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    return marker;
}



int main(int argc, char **argv){

    google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "cylinder_node");
    ros::NodeHandle nh;
    ROS_WARN("cylinder_node node begin...");

    std::vector<ros::Publisher> pubEachSupport;
    ros::Publisher pubFullPc = nh.advertise<sensor_msgs::PointCloud2>("/full_pointcloud", 1);
    ros::Publisher pubCylinder = nh.advertise<sensor_msgs::PointCloud2>("/cylinder", 1);
    ros::Publisher pubNoneCylinder = nh.advertise<sensor_msgs::PointCloud2>("/none_cylinder", 1);
    ros::Publisher pubNoneCylinder_clean = nh.advertise<sensor_msgs::PointCloud2>("/none_cylinder_clean", 1);
    ros::Publisher pubCylinderAxisPoints = nh.advertise<sensor_msgs::PointCloud2>("/cylinder_axis_points", 1);
    ros::Publisher drawMarkers = nh.advertise<visualization_msgs::MarkerArray>("/cyliner_markers", 1);
    ros::Publisher drawBaseMarker = nh.advertise<visualization_msgs::MarkerArray>("/base_markers", 1);

    ros::Publisher pubModel = nh.advertise<sensor_msgs::PointCloud2>("/model", 1);
    ros::Publisher pubModelInScene = nh.advertise<sensor_msgs::PointCloud2>("/model_in_scene", 1);

    visualization_msgs::MarkerArray marker_array_base;


    MyPointCloud base_model, all_base_in_scene;
    string model_filename = "/home/larrydong/base.pcd";
    // pcl::PCLPointCloud2 model_ply;
    if (pcl::io::loadPCDFile(model_filename, base_model) != 0){
        cout << "Cannot load file: " << model_filename << endl;
        // pcl::fromPCLPointCloud2(model_ply, base_model);
    }

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


    MyPointCloud all_cylinder_pc, non_cylinder_pc;
    std::vector<pcl::ModelCoefficients> v_coefficients;
    std::vector<Cylinder> v_cylinders;
    CylinderParameters cylinder_settings(FLAGS_cylinder_search_radius, FLAGS_cylinder_normal_distance_weight, FLAGS_cylinder_distance_threshould,
                        FLAGS_cylinder_max_iteration, FLAGS_cylinder_radius_min, FLAGS_cylinder_radius_max);

    // extract cylinder from each half.
    for (int i = 0; i < v_support.size(); ++i){
    // int i = 2;

        MyPointCloud::Ptr one_support(new MyPointCloud()), left_half(new MyPointCloud()), right_half(new MyPointCloud());
        one_support = v_support[i];
        pcl::PassThrough<pcl::PointXYZ> pass;       // seg half.
        pass.setInputCloud(one_support);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(distance[i] - FLAGS_support_segment_y / 2, distance[i]);
        pass.filter(*left_half);
        pass.setFilterLimits(distance[i], distance[i] + FLAGS_support_segment_y / 2);
        pass.filter(*right_half);
        
        Cylinder cylinder(*left_half);
        if(cylinder.detectCylinder(cylinder_settings)==true){
            v_cylinders.push_back(cylinder);
            all_cylinder_pc += cylinder.getCylinderPointCloud();
            non_cylinder_pc += cylinder.getNoneCylinderPointCloud();
        }
        cylinder.resetPointCloud(*right_half);
        if(cylinder.detectCylinder(cylinder_settings)==true){
            v_cylinders.push_back(cylinder);
            all_cylinder_pc += cylinder.getCylinderPointCloud();
            non_cylinder_pc += cylinder.getNoneCylinderPointCloud();
        }

        Eigen::Vector3f init_T(2.0, distance[i], -1);
        cout << "Model size: " << base_model.size() << ", scene size: " << non_cylinder_pc.size() << endl;
        Eigen::Matrix4f T = findModelInScene(base_model, non_cylinder_pc, init_T);
        MyPointCloud one_model_in_scene;
        pcl::transformPointCloud(base_model, one_model_in_scene, T);
        all_base_in_scene += one_model_in_scene;
        marker_array_base.markers.push_back(createBaseMarker(T));
    }

    ROS_INFO_STREAM("Total cylinder: "<< v_cylinders.size());
    MyPointCloud clean_non_cylinder_pc;
    // base_process(non_cylinder_pc, clean_non_cylinder_pc);

    visualization_msgs::MarkerArray marker_array;
    for(int i=0; i<v_cylinders.size(); ++i){
        marker_array.markers.push_back(v_cylinders[i].calMarker());
    }
    

    ros::Rate r(10);
    while(ros::ok()){

        for (int i = 0; i < pubEachSupport.size(); ++i){
            pubEachSupport[i].publish(tool::pointCloud2RosMsg(*v_support[i]));
        }
        pubFullPc.publish(tool::pointCloud2RosMsg(full_pc));
        pubCylinder.publish(tool::pointCloud2RosMsg(all_cylinder_pc));
        pubNoneCylinder.publish(tool::pointCloud2RosMsg(non_cylinder_pc));
        pubNoneCylinder_clean.publish(tool::pointCloud2RosMsg(clean_non_cylinder_pc));
        drawMarkers.publish(marker_array);
        drawBaseMarker.publish(marker_array_base);

        pubModel.publish(tool::pointCloud2RosMsg(base_model));
        pubModelInScene.publish(tool::pointCloud2RosMsg(all_base_in_scene));

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}



void base_process(const MyPointCloud& cloud_in, MyPointCloud& cloud_out){
    pcl::PassThrough<pcl::PointXYZ> pass;
    MyPointCloud::Ptr pc (new MyPointCloud), base_pc(new MyPointCloud);
    *pc = cloud_in;
    pass.setInputCloud(pc);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1, 1.5);
    pass.filter(*base_pc);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(base_pc);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1);
    sor.filter(cloud_out);
    pcl::PLYWriter writer;
    writer.write("/home/larrydong/base.ply", cloud_out);
}


Eigen::Matrix4f findModelInScene(const MyPointCloud& cloud_model, const MyPointCloud& cloud_scene, Eigen::Vector3f init_T){
    MyPointCloud::Ptr pc_model(new MyPointCloud), pc_scene(new MyPointCloud);
    *pc_model = cloud_model;
    *pc_scene = cloud_scene;

    pcl::IterativeClosestPoint<MyPoint, MyPoint> icp;
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    init_guess.col(3).head(3) = init_T;
    // cout << "Guess: " << init_guess << endl;

    MyPointCloud tmp;
    icp.setInputSource(pc_model);
    icp.setInputTarget(pc_scene);
    icp.align(tmp, init_guess);

    Eigen::Matrix4f T;

    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

    cout << "Transformation: ---- \n" << icp.getFinalTransformation() << endl;
    return icp.getFinalTransformation();
}

