
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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
#include "process/support_base.h"

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


string base_model_filename = "/home/larrydong/base.pcd";
string base_mesh_filename = "/home/larrydong/base_surf.dae";


int main(int argc, char **argv){

    google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "cylinder_node");
    ros::NodeHandle nh;
    ROS_WARN("cylinder_node node begin...");

    std::vector<ros::Publisher> pubEachSupport;
    ros::Publisher pubFullPc = nh.advertise<sensor_msgs::PointCloud2>("/full_pointcloud", 1);
    ros::Publisher pubCylinder = nh.advertise<sensor_msgs::PointCloud2>("/cylinder", 1);
    ros::Publisher pubCylinderAxisPoints = nh.advertise<sensor_msgs::PointCloud2>("/cylinder_axis_points", 1);
    ros::Publisher drawCylinderMarkers = nh.advertise<visualization_msgs::MarkerArray>("/cyliner_markers", 1);
    ros::Publisher drawBaseMarker = nh.advertise<visualization_msgs::MarkerArray>("/base_markers", 1);

    ros::Publisher pubModelInScene = nh.advertise<sensor_msgs::PointCloud2>("/model_in_scene", 1);
    visualization_msgs::MarkerArray marker_array_cylinder, marker_array_base;


    MyPointCloud full_pc, all_base_in_scene;
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
    std::vector<Cylinder> v_cylinders;
    CylinderParameters cylinder_settings(FLAGS_cylinder_search_radius, FLAGS_cylinder_normal_distance_weight, FLAGS_cylinder_distance_threshould,
                        FLAGS_cylinder_max_iteration, FLAGS_cylinder_radius_min, FLAGS_cylinder_radius_max);

    // extract cylinder from each half.
    for (int i = 0; i < v_support.size(); ++i){

        MyPointCloud::Ptr one_support(new MyPointCloud()), left_half(new MyPointCloud()), right_half(new MyPointCloud());
        one_support = v_support[i];
        pcl::PassThrough<pcl::PointXYZ> pass;       // seg half.
        pass.setInputCloud(one_support);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(distance[i] - FLAGS_support_segment_y / 2, distance[i]);
        pass.filter(*left_half);
        pass.setFilterLimits(distance[i], distance[i] + FLAGS_support_segment_y / 2);
        pass.filter(*right_half);
        
        MyPointCloud one_support_without_cylinder;
        Cylinder cylinder(*left_half);
        if(cylinder.detectCylinder(cylinder_settings)==true){
            v_cylinders.push_back(cylinder);
            all_cylinder_pc += cylinder.getCylinderPointCloud();
            one_support_without_cylinder += cylinder.getNoneCylinderPointCloud();
        }
        cylinder.resetPointCloud(*right_half);
        if(cylinder.detectCylinder(cylinder_settings)==true){
            v_cylinders.push_back(cylinder);
            all_cylinder_pc += cylinder.getCylinderPointCloud();
            one_support_without_cylinder += cylinder.getNoneCylinderPointCloud();
        }

        SupportBase base(base_model_filename, base_mesh_filename);              // create base and detect.
        Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
        init_guess.col(3).head(3) = Eigen::Vector3f(2.0, distance[i], -1);      // parameters from settings...
        if(base.detectBase(one_support_without_cylinder, init_guess)){
            marker_array_base.markers.push_back(base.createBaseMarker());
            MyPointCloud one_model_in_scene;
            pcl::transformPointCloud(base.model_cloud_, one_model_in_scene, base.transformation_);
            all_base_in_scene += one_model_in_scene;
        }
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
        drawCylinderMarkers.publish(marker_array);
        drawBaseMarker.publish(marker_array_base);
        pubModelInScene.publish(tool::pointCloud2RosMsg(all_base_in_scene));

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

#if 0
//extract base. Not used but will be used later.
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
#endif
