
#include "pcl_process/support_cylinder.h"
#include "pcl_process/support_roof.h"
#include "pcl_process/support_base.h"
#include "pcl_process/extract_coeff.h"
#include "defination.h"
#include "config.h"
#include "tool.h"

#include <gflags/gflags.h>

#include <iostream>

#include <pcl/common/transforms.h>
#include <eigen3/Eigen/Core>
#include <pcl/io/ply_io.h>


using namespace std;


// load parameters
DEFINE_string(isolated_support_path, "/home/larrydong/lidar_ws/output/result/", "isolated_support");
DEFINE_string(isolated_support_distance, "/home/larrydong/lidar_ws/output/result/distance.txt", "isolated_support's distance");
DEFINE_string(base_model_file, "/home/larrydong/lidar_ws/output/result/base_model.ply", "base model");
DEFINE_string(clean_car_path, "/home/larrydong/lidar_ws/output/result/clean_car_path.txt", "car path");
DEFINE_string(support_model_dae, "/home/larrydong/lidar_ws/output/result/xxx.dae", "support dae file"); // TODO:

// segment each cylinder
DEFINE_double(support_width, 2.5, "each support's width");

// cylinder fitting parameters
DEFINE_double(cylinder_search_radius, 0.2, "cylinder_search_radius");
DEFINE_double(cylinder_normal_distance_weight, 0.05, "cylinder_normal_distance_weight");
DEFINE_double(cylinder_distance_threshould, 0.2, "cylinder_distance_threshould");
DEFINE_double(cylinder_radius_min, 0.1, "cylinder_radius_min");
DEFINE_double(cylinder_radius_max, 1.0, "cylinder_radius_max");
DEFINE_int32(cylinder_max_iteration, 1000, "max iteration for optimazaion");

// roof paramers for plane-fitting
DEFINE_double(roof_norm_angle, M_PI/10, "roof norm along z-axis");
DEFINE_double(roof_x_min, -1, "x-range");
DEFINE_double(roof_x_max, 1, "x-range");
DEFINE_double(roof_z_min, 1, "z-range");
DEFINE_double(roof_z_max, 5, "z-range");

// base parameters for icp-init
DEFINE_double(icp_dx, 2, "base-icp init dx");
DEFINE_double(icp_dy, 0, "base-icp init dy");
DEFINE_double(icp_dz, -1, "base-icp init dz");

// roof parameters for coeff calculating
DEFINE_double(roof_width, 2.0, "roof width");
DEFINE_double(roof_height, 2.7, "roof height");


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

    ///////////////  car-trace result.  ///////////////
    TraceCoeff trace(FLAGS_clean_car_path);
    trace.printResult();

    ///////////////  support results.   ///////////////
    vector<MyPointCloud> v_supports;
    vector<MyPoint> v_distance;
    loadData(v_supports, v_distance);
    int isolated_number = v_supports.size();

    // settings.
    CylinderParameters cylinder_settings(FLAGS_cylinder_search_radius, FLAGS_cylinder_normal_distance_weight, FLAGS_cylinder_distance_threshould,FLAGS_cylinder_max_iteration, FLAGS_cylinder_radius_min, FLAGS_cylinder_radius_max);
    RoofParameters roof_settings(FLAGS_roof_x_min, FLAGS_roof_x_max, FLAGS_roof_z_min, FLAGS_roof_z_max);

    // debug for viewer
    vector<MyPointCloud> v_left_cylinders, v_right_cylinders;
    MyPointCloud all_scenePC, none_cylinderPC, all_cylinderPC, all_roofPC, all_basePC;

    //////////////////////////////   MAIN PROCESS  //////////////////////////////
    for(int i=0; i<isolated_number; ++i){
        cout << "============================= Index: " << i << "=============================" << endl;

        // 0. extract support / position
        MyPointCloud::Ptr one_support(new MyPointCloud()), left_half(new MyPointCloud()), right_half(new MyPointCloud());
        *one_support = v_supports[i];
        all_scenePC += *one_support;
        double distance = v_distance[i].y;

        // 1. get cylinders.
        pcl::PassThrough<pcl::PointXYZ> pass; // seg half.
        pass.setInputCloud(one_support);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(distance - FLAGS_support_width / 2, distance);
        pass.filter(*left_half);
        pass.setFilterLimits(distance, distance + FLAGS_support_width / 2);
        pass.filter(*right_half);

        double x, y, z;
        MyPointCloud one_support_without_cylinder;
        Cylinder l_cylinder(*left_half), r_cylinder(*right_half);
        if (l_cylinder.detectCylinder(cylinder_settings)){
            all_cylinderPC += l_cylinder.getCylinderPointCloud();
            l_cylinder.getCenterPos(x, y, z);
            one_support_without_cylinder += l_cylinder.getNoneCylinderPointCloud();
#ifdef DEBUG_OUTPUT
            cout << "Cylinder center: [" << x << ", " << y << ", " << z << "]. " << endl;
#endif
        }
        if (r_cylinder.detectCylinder(cylinder_settings)){
            all_cylinderPC += r_cylinder.getCylinderPointCloud();
            r_cylinder.getCenterPos(x, y, z);
            one_support_without_cylinder += r_cylinder.getNoneCylinderPointCloud();
#ifdef DEBUG_OUTPUT
            cout << "Cylinder center: [" << x << ", " << y << ", " << z << "]. " << endl;
#endif
        }

        // 2. get roof & base
        if(l_cylinder.is_find_ && r_cylinder.is_find_){
            cout << "Suppot [ " << i << " ] finds 2 cylinders. Calculate base and roof then." << endl;

            // roof process
            SupportRoof roof(one_support_without_cylinder);
            none_cylinderPC += one_support_without_cylinder;
            roof.detectRoof(roof_settings);
            all_roofPC += roof.plane_cloud_;

            // base process
            SupportBase base(FLAGS_base_model_file, FLAGS_support_model_dae);
            Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
            init_guess.col(3).head(3) = Eigen::Vector3f(v_distance[i].x + FLAGS_icp_dx, v_distance[i].y, v_distance[i].z + FLAGS_icp_dz);
            base.detectBase(one_support_without_cylinder, init_guess);
            MyPointCloud one_model_in_scene;
            pcl::transformPointCloud(base.model_cloud_, one_model_in_scene, base.transformation_);
            all_basePC += one_model_in_scene;

            // coeffs;
            SupportCoeff coeff(
                l_cylinder.cylinder_center_, 
                l_cylinder.cylinder_dir_,
                r_cylinder.cylinder_center_,
                r_cylinder.cylinder_dir_,
                base.transformation_.cast<double>(), 
                roof.plane_center_.cast<double>(), 
                roof.plane_normal_.cast<double>()
            );
            coeff.calcResult();
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
    ros::Publisher pubScenePC = nh.advertise<sensor_msgs::PointCloud2>("/scene", 1);
    ros::Publisher pubCylinders = nh.advertise<sensor_msgs::PointCloud2>("/cylinders", 1);
    ros::Publisher pubRoofs = nh.advertise<sensor_msgs::PointCloud2>("/roofs", 1);
    ros::Publisher pubBaseModel = nh.advertise<sensor_msgs::PointCloud2>("/base_model", 1);
    ros::Publisher pubBases = nh.advertise<sensor_msgs::PointCloud2>("/bases", 1);

    ros::Rate r(1);
    while(ros::ok()){
        for (int i = 0; i < v_left_cylinders.size(); i++){
            v_pubLeft[i].publish(tool::pointCloud2RosMsg(v_left_cylinders[i]));
        }
        for (int i = 0; i < v_right_cylinders.size(); i++){
            v_pubRight[i].publish(tool::pointCloud2RosMsg(v_right_cylinders[i]));
        }
        pubScenePC.publish(tool::pointCloud2RosMsg(all_scenePC));
        pubCylinders.publish(tool::pointCloud2RosMsg(all_cylinderPC));
        pubRoofs.publish(tool::pointCloud2RosMsg(all_roofPC));
        // pubBaseModel.publish(tool::pointCloud2RosMsg(model_pc));
        pubBases.publish(tool::pointCloud2RosMsg(all_basePC));
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
