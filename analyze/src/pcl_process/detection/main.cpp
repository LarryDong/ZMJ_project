
#include "pcl_process/car_path.h"
#include "pcl_process/scene_cloud.h"
// #include "pcl_process/support.h"
#include "pcl_process/support_cylinder.h"
#include "pcl_process/support_plane.h"

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



#define LOAD_SUPPORT


MyPointCloud g_all_cylinderPC;



using namespace std;

Eigen::Matrix4d calcGlobalT(const CarPath& cp){
    Eigen::Vector3d pb = tool::xyz2vector(cp.getBeginPoint());  // path_begin
    Eigen::Vector3d pe = tool::xyz2vector(cp.getEndPoint());
    Eigen::Vector3d dir = pe - pb;
    dir.normalize();

#ifdef DEBUG_OUTPUT
    cout << "Car from: (" << pb(0) << ", " << pb(1) << ", " << pb(2)
        << "), to: (" << pe(0) << ", " << pe(1) << ", " << pe(2) << ")" << endl;
#endif
    Eigen::Vector3d xn, yn, zn;
    zn << 0, 0, 1;
    yn = dir - dir.dot(zn) * zn;      // project x to z;
    yn = -yn;                         // rotate to original direction; moving direction is always (-y).

    yn.normalize();
    xn = yn.cross(zn);
    xn.normalize();

    Eigen::Matrix3d R;      // R: from new coordinate to the old; R*vn = v
    R.col(0) = xn;
    R.col(1) = yn;
    R.col(2) = zn;
    // explain the rotation:
    // v: points vector in old coordinate; vn: points vector in new coordiante;
    // x, y, z: new coordiante's axis representated by old coordiante axis;
    // [x, y, z]*v = [xn, yn, zn]*vn + t, where t = pb;  --> vn = -R^(-1)*v - R^(-1)*pb
    
    Eigen::Matrix4d globalT = Eigen::Matrix4d::Identity();
    // globalT.topLeftCorner<3, 3>(0, 0) = R.transpose();   // This line is an error 
    globalT.topLeftCorner(3,3)= R.transpose();
    globalT.col(3).head(3) = -R.transpose() * pb;

    // cout << "Final transform (from old points to new points): \n" << globalT << endl;
    return globalT;
}

void updateCoordinate(CarPath& cp, SceneCloud& sc, const Eigen::Matrix4d& T){       // reset scene_cloud and car_path by new T.
    pcl::PointCloud<pcl::PointXYZ> new_scene_pc;
    pcl::transformPointCloud(sc.getFullPointCloud(), new_scene_pc, T.cast<float>());
    // sc.resetFullPointCloud(new_scene_pc);
    *sc.pc_ = new_scene_pc;
    
    pcl::PointCloud<pcl::PointXYZ> new_path_pc;
    pcl::transformPointCloud(cp.getFullPointCloud(), new_path_pc, T.cast<float>());
    // cp.resetFullPointCloud(new_path_pc);
    *cp.pc_ = new_path_pc;
}




//////////////////////////////////////////  MAIN  //////////////////////////////////////////////////////


int main(int argc, char **argv){

	ros::init(argc, argv, "pcl_process");
	ros::NodeHandle nh;
    google::ParseCommandLineFlags(&argc, &argv, true);



//////////////////////////////////////////////////////////////////////////////////////
#ifdef LOAD_SUPPORT
    cout << "------- DEBUG -----------" << endl;
    vector<MyPointCloud> v_supports;
    int N = 6;
    // save isolated support.
    for(int i=0; i<N; ++i){
        MyPointCloud::Ptr cloud_in(new MyPointCloud());
        string filename = FLAGS_isolated_support_path + to_string(i)+".ply";
        if (pcl::io::loadPLYFile(filename, *cloud_in) < 0){
            cout << "Error! Cannot load file: " << filename << endl;
            std::abort();
        }
        v_supports.push_back(*cloud_in);
    }
    cout << "Loaded " << v_supports.size() << "supports." << endl;

    // load distance from carpath
    vector<MyPoint> v_distance;
    ifstream in_file(FLAGS_isolated_support_distance, ios::in);
    if(!in_file.is_open()) {
        cout << "Error! Cannot load file: " << FLAGS_isolated_support_distance << endl;
        std::abort();
    }
    for(int i=0; i<N; ++i){
        double x, y, z;
        in_file >> x >> y >> z;
        v_distance.push_back(MyPoint(x, y, z));
        cout << "Support position: (" << x << ", " << y << ", " << z << ")" << endl;
    }

    // settings.
    CylinderParameters cylinder_settings(FLAGS_cylinder_search_radius, FLAGS_cylinder_normal_distance_weight, FLAGS_cylinder_distance_threshould,FLAGS_cylinder_max_iteration, FLAGS_cylinder_radius_min, FLAGS_cylinder_radius_max);
    PlaneParameters plane_settings(FLAGS_roof_x_min, FLAGS_roof_x_max, FLAGS_roof_z_min, FLAGS_roof_z_max);

    // debug
    vector<MyPointCloud> v_left_cylinders, v_right_cylinders;
    MyPointCloud none_cylinderPC, all_cylinderPC, all_roofPC, all_basePC;
    
    // process each supports
    for(int i=0; i<N; ++i){
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
            SupportPlane roof(one_support_without_cylinder);
            none_cylinderPC += one_support_without_cylinder;
            // TODO:
            // roof.detectPlane(plane_settings);
            // all_plane_pc += plane.plane_cloud_;
            // marker_array_plane.markers.push_back(plane.createMarker());
        }
    }
    




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
        ros::spinOnce();
        r.sleep();
    }


    return 1;
#endif
//////////////////////////////////////////////////////////////////////////////////////






















    cout << "Load ver.pcd from: " << FLAGS_raw_ver << endl;
    cout << "Load car path from: " << FLAGS_raw_car_path << endl;

    // STEP 1. Load data.
    ROS_WARN("Loading data...");
    ros::Time tbegin = ros::Time().now();
    CarPath car_path(FLAGS_raw_car_path);
    SceneCloud scene_cloud(FLAGS_raw_ver);

#ifdef DEBUG_OUTPUT
    cout << "Load time: " << ros::Time().now() - tbegin << " s" << endl;
#endif


    // 1.1 data preprocess.
    ROS_WARN("Calculating global transform based on car parth ...");
    Eigen::Matrix4d globalTransform = calcGlobalT(car_path);
    // TODO: save globalT
    updateCoordinate(car_path, scene_cloud, globalTransform);

#ifdef DEBUG_OUTPUT
    cout << " ------------- Global Transform ------------- " << endl << globalTransform << endl;
#endif

    scene_cloud.filter(
        FLAGS_filter_downsampling_size, 
        FLAGS_filter_passthrough_xmin, 
        FLAGS_filter_passthrough_xmax
    );
    car_path.digitalize();          // to 1cm point cloud.
    // car_path.saveCarPathToFile(FLAGS_clean_car_path);
    
    // Step 2. Find roofs
    // 2.1 Filter by clustering. 
    ClusterParameter cluster_parameter(
        FLAGS_cluster_angle1, 
        FLAGS_cluster_distance1, 
        FLAGS_cluster_size_min1, 
        FLAGS_cluster_size_max1, 
        FLAGS_cluster_radius
    );
    PlaneParameter plane_parameter(
        FLAGS_plane_l1l3_ratio1, 
        FLAGS_plane_l1l2_ratio1
    );
    ROS_WARN("Filtering the PC by mergeing small planes.");
    scene_cloud.filerByClustering(cluster_parameter, plane_parameter); // merge all plane by a "strict" criteria
    
    // 2.2 Extract all planes
    cluster_parameter.reset(
        FLAGS_cluster_angle2, 
        FLAGS_cluster_distance2, 
        FLAGS_cluster_size_min2, 
        FLAGS_cluster_size_max2, 
        FLAGS_cluster_radius
    );
    plane_parameter.reset(
        FLAGS_plane_l1l3_ratio2, 
        FLAGS_plane_l1l2_ratio2
    );
    ROS_WARN("Extracting all planes in clustering-fileterd pc.");
    scene_cloud.extractPlanes(cluster_parameter, plane_parameter);    // extract planes by a "loose" criteria


    // 2.3 Select roofs based on support size.
    SupportParameter support_parameter;
    support_parameter.setRoof(FLAGS_roof_x_min, FLAGS_roof_x_max, FLAGS_roof_z_min, FLAGS_roof_z_max, FLAGS_roof_norm_angle);
    support_parameter.setSegment(FLAGS_support_segment_x_min, FLAGS_support_segment_x_max, FLAGS_support_segment_y, FLAGS_support_segment_z_min, FLAGS_support_segment_z_max);
    scene_cloud.selectRoofsAndSegment(car_path, support_parameter);

    int support_number = scene_cloud.getSupportsVectorPC().size();
    // save isolated support.
    for(int i=0; i<support_number; ++i){
        pcl::io::savePLYFile(FLAGS_isolated_support_path + to_string(i)+".ply", scene_cloud.getSupportsVectorPC()[i]);
    }
    cout << "Saved " << support_number << " isolated support into ." << endl;
// save carpath
    ofstream out(FLAGS_isolated_support_distance, ios::out);
    for(auto pt : scene_cloud.getSupportDistance()){
        out << pt.x << " " << pt.y << " " << pt.z << endl;
    }
    std::cout << "Saved support's distance to " << FLAGS_isolated_support_distance << endl;
    out.close();

    // STEP 3. Each support processing.
    // 1. Load supports
    // Support v_support;

    // view.
    ros::Publisher pubSceneCloud = nh.advertise<sensor_msgs::PointCloud2>("/scene_cloud", 1);
    ros::Publisher pubCarPath = nh.advertise<sensor_msgs::PointCloud2>("/car_path", 1);
    ros::Publisher pubClusteredPC = nh.advertise<sensor_msgs::PointCloud2>("/cluster_filtered_PC", 1);
    ros::Publisher pubPlanePC = nh.advertise<sensor_msgs::PointCloud2>("/planes_PC", 1);
    ros::Publisher pubRoofPC = nh.advertise<sensor_msgs::PointCloud2>("/roof_PC", 1);
    vector<ros::Publisher> v_pubEachPlane, v_pubEachSupport;
    for (int i = 0; i < scene_cloud.getRoofsVectorPC().size(); i++){
        ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/plane_" + std::to_string(i), 1);
        v_pubEachPlane.push_back(tmp);
    }
    for (int i = 0; i < scene_cloud.getSupportsVectorPC().size(); i++){
        ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/support_" + std::to_string(i), 1);
        v_pubEachSupport.push_back(tmp);
    }

    ros::Rate rate(1);
    while(ros::ok()){
        // publish data for review
        pubSceneCloud.publish(tool::pointCloud2RosMsg(scene_cloud.getFullPointCloud()));
        pubCarPath.publish(tool::pointCloud2RosMsg(car_path.getFullPointCloud()));
        pubClusteredPC.publish(tool::pointCloud2RosMsg(scene_cloud.getClusterFilteredPC()));
        pubPlanePC.publish(tool::pointCloud2RosMsg(scene_cloud.getPlanesWholePC()));
        pubRoofPC.publish(tool::pointCloud2RosMsg(scene_cloud.getRoofsWholePC()));

        // pub all planes (before selecting roofs)
        for(int i=0; i<scene_cloud.getRoofsVectorPC().size(); i++){
            v_pubEachPlane[i].publish(tool::pointCloud2RosMsg(scene_cloud.getRoofsVectorPC()[i]));
        }        
        // pub all supports
        for(int i=0; i<scene_cloud.getSupportsVectorPC().size(); i++){
            v_pubEachSupport[i].publish(tool::pointCloud2RosMsg(scene_cloud.getSupportsVectorPC()[i]));
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
