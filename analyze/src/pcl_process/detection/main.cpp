
#include "pcl_process/car_path.h"
#include "pcl_process/scene_cloud.h"
// #include "pcl_process/support.h"
#include "pcl_process/support_cylinder.h"
#include "pcl_process/support_roof.h"

#include "defination.h"
#include "config.h"

#include <gflags/gflags.h>

#include <iostream>

#include <pcl/common/transforms.h>
#include <eigen3/Eigen/Core>
#include <pcl/io/ply_io.h>

using namespace std;

DEFINE_string(isolated_support_path, "/home/larrydong/lidar_ws/output/result/", "isolated_support");
DEFINE_string(isolated_support_distance, "/home/larrydong/lidar_ws/output/result/distance.txt", "isolated_support's distance");

// point cloud filter
DEFINE_double(filter_passthrough_xmin, -0.5, "passthrough fitler x range");
DEFINE_double(filter_passthrough_xmax, 5, "passthrough fitler x range");
DEFINE_double(filter_downsampling_size, 0.03, "voxel downsampling size");

// suplane_parameterort parameters
DEFINE_double(suplane_parameterort_width, 2, "suplane_parameterort width");
DEFINE_double(suplane_parameterort_height, 5, "suplane_parameterort height");
DEFINE_int32(suplane_parameterort_direction, 1, "on +x or -x direction");
//   
DEFINE_double(roof_norm_angle, M_PI/10, "roof norm along z-axis");
DEFINE_double(roof_x_min, -1, "x-range");
DEFINE_double(roof_x_max, 1, "x-range");
DEFINE_double(roof_z_min, 1, "z-range");
DEFINE_double(roof_z_max, 5, "z-range");
// segment
DEFINE_double(support_segment_y, 2.5, "segment along y axis range");
DEFINE_double(support_segment_z_min, 0, "segment along z axis min");
DEFINE_double(support_segment_z_max, 100, "segment along z axis max");
DEFINE_double(support_segment_x_min, -1, "segment along x axis min");
DEFINE_double(support_segment_x_max, 10, "segment along x axis max");

// cylinder settings
DEFINE_double(cylinder_search_radius, 0.2, "cylinder_search_radius");
DEFINE_double(cylinder_normal_distance_weight, 0.05, "cylinder_normal_distance_weight");
DEFINE_double(cylinder_distance_threshould, 0.2, "cylinder_distance_threshould");
DEFINE_double(cylinder_radius_min, 0.1, "cylinder_radius_min");
DEFINE_double(cylinder_radius_max, 1.0, "cylinder_radius_max");
DEFINE_int32(cylinder_max_iteration, 1000, "max iteration for optimazaion");


MyPointCloud g_all_cylinderPC;


using namespace std;


void loadData(vector<MyPointCloud>& v_supports, vector<MyPoint>& v_distance){
    v_supports.resize(0);
    v_distance.resize(0);
    for(int i=0; i<100; ++i){
        MyPointCloud::Ptr cloud_in(new MyPointCloud());
        string filename = FLAGS_isolated_support_path + to_string(i)+".ply";
        if (pcl::io::loadPLYFile(filename, *cloud_in) < 0){
            cout << "Error! Cannot load file: " << filename << endl;
            break;
        }
        v_supports.push_back(*cloud_in);
    }
    cout << "Loaded " << v_supports.size() << "supports." << endl;
    // load distance from carpath
    ifstream in_file(FLAGS_isolated_support_distance, ios::in);
    if(!in_file.is_open()) {
        cout << "Error! Cannot load file: " << FLAGS_isolated_support_distance << endl;
        std::abort();
    }
    for (int i = 0; i < v_supports.size(); ++i){
        double x, y, z;
        in_file >> x >> y >> z;
        v_distance.push_back(MyPoint(x, y, z));
        cout << "Support position: (" << x << ", " << y << ", " << z << ")" << endl;
    }
}



//////////////////////////////////////////  MAIN  //////////////////////////////////////////////////////


int main(int argc, char **argv){

	ros::init(argc, argv, "pcl_process");
	ros::NodeHandle nh;
    google::ParseCommandLineFlags(&argc, &argv, true);

    // load data.
    vector<MyPointCloud> v_supports;
    vector<MyPoint> v_distance;
    loadData(v_supports, v_distance);
    int isolated_number = v_supports.size();
        
    // settings.
    CylinderParameters cylinder_settings(FLAGS_cylinder_search_radius, FLAGS_cylinder_normal_distance_weight, FLAGS_cylinder_distance_threshould,FLAGS_cylinder_max_iteration, FLAGS_cylinder_radius_min, FLAGS_cylinder_radius_max);
    PlaneParameters plane_settings(FLAGS_roof_x_min, FLAGS_roof_x_max, FLAGS_roof_z_min, FLAGS_roof_z_max);


    // debug for viewer
    vector<MyPointCloud> v_left_cylinders, v_right_cylinders;
    MyPointCloud none_cylinderPC, all_cylinderPC, all_roofPC, all_basePC;
    

    //////////////////////////////   MAIN PROCESS  //////////////////////////////

    for(int i=0; i<isolated_number; ++i){
        // 0. extract support / position
        MyPointCloud::Ptr one_support(new MyPointCloud()), left_half(new MyPointCloud()), right_half(new MyPointCloud());
        *one_support = v_supports[i];
        double distance = v_distance[i].y;

        // 1. get cylinders.
        pcl::PassThrough<pcl::PointXYZ> pass; // seg half.
        pass.setInputCloud(one_support);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(distance - FLAGS_support_segment_y / 2, distance);
        pass.filter(*left_half);
        pass.setFilterLimits(distance, distance + FLAGS_support_segment_y / 2);
        pass.filter(*right_half);

        double x, y, z;
        MyPointCloud one_support_without_cylinder;
        Cylinder l_cylinder(*left_half);
        if (l_cylinder.detectCylinder(cylinder_settings)){
            g_all_cylinderPC += l_cylinder.getCylinderPointCloud();
            one_support_without_cylinder += l_cylinder.getNoneCylinderPointCloud();
            l_cylinder.getCenterPos(x, y, z);
            cout << "Cylinder center: [" << x << ", " << y << ", " << z << "]. " << endl;
        }

        Cylinder r_cylinder(*right_half);
        if (r_cylinder.detectCylinder(cylinder_settings)){
            g_all_cylinderPC += r_cylinder.getCylinderPointCloud();
            one_support_without_cylinder += r_cylinder.getNoneCylinderPointCloud();
            r_cylinder.getCenterPos(x, y, z);
            cout << "Cylinder center: [" << x << ", " << y << ", " << z << "]. " << endl;
        }

        // 2. get roof  // TODO:
        if(l_cylinder.is_find_ && r_cylinder.is_find_){
            cout << "Suppot [ " << i << " ] founded. Detect roof/base now." << endl;
            SupportRoof roof(one_support_without_cylinder);
            none_cylinderPC += one_support_without_cylinder;
            roof.detectRoof(plane_settings);
            all_roofPC += roof.plane_cloud_;
        }
    }


    // view.
    vector<ros::Publisher> v_pubLeft, v_pubRight;
    for (int i = 0; i < v_left_cylinders.size(); i++){
        ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/left_support_" + std::to_string(i), 1);
        v_pubLeft.push_back(tmp);
    }
    for (int i = 0; i < v_right_cylinders.size(); i++){
        ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/right_support_" + std::to_string(i), 1);
        v_pubRight.push_back(tmp);
    }
    ros::Publisher pubCylinders = nh.advertise<sensor_msgs::PointCloud2>("/cylinders", 1);
    ros::Publisher pubRoofs = nh.advertise<sensor_msgs::PointCloud2>("/roofs", 1);
    // ros::Publisher pubBases = nh.advertise<sensor_msgs::PointCloud2>("/bases", 1);

    ros::Rate r(1);
    while(ros::ok()){
        // pub all planes (before selecting roofs)
        for (int i = 0; i < v_left_cylinders.size(); i++){
            v_pubLeft[i].publish(tool::pointCloud2RosMsg(v_left_cylinders[i]));
        }
        for (int i = 0; i < v_right_cylinders.size(); i++){
            v_pubRight[i].publish(tool::pointCloud2RosMsg(v_right_cylinders[i]));
        }
        pubCylinders.publish(tool::pointCloud2RosMsg(all_cylinderPC));
        pubRoofs.publish(tool::pointCloud2RosMsg(all_roofPC));
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
