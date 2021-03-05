

#include "process/scene_cloud.h"
#include "process/car_path.h"
#include "process/tool.h"
#include "process/process.h"
#include "process/defination.h"


using namespace std;


DEFINE_string(file_scene_cloud, "ver.pcd", "scene cloud path");
DEFINE_string(file_model_cloud, "model.pcd", "model cloud path");
DEFINE_string(file_carpath, "nav_msgs.txt", "car path messages");


DEFINE_double(filter_passthrough_xmin, -0.5, "passthrough fitler x range");
DEFINE_double(filter_passthrough_xmax, 5, "passthrough fitler x range");
DEFINE_double(filter_downsampling_size, 0.03, "voxel downsampling size");

DEFINE_double(cluster_radius, 0.1, "clustering nn search radius");
DEFINE_double(cluster_angle1, M_PI/10, "smaller angle tolerance");
DEFINE_double(cluster_angle2, M_PI, "larger angle tolerance");
DEFINE_double(cluster_distance1, 0.2, "clustering distance tolerance");
DEFINE_double(cluster_distance2, 0.2, "clustering distance tolerance");
DEFINE_double(cluster_size_min1, 100, "minimum cluster size");
DEFINE_double(cluster_size_max1, 100000, "max cluster size");
DEFINE_double(cluster_size_min2, 100, "minimum cluster size");
DEFINE_double(cluster_size_max2, 100000, "max cluster size");

// plane
DEFINE_double(plane_l1l3_ratio1, 50, "plane l1/l3");
DEFINE_double(plane_l1l2_ratio1, 5, "plane l1/l2");
DEFINE_double(plane_l1l3_ratio2, 50, "plane l1/l3");
DEFINE_double(plane_l1l2_ratio2, 5, "plane l1/l2");

// support parameters
DEFINE_double(support_width, 2, "support width");
DEFINE_double(support_height, 5, "support height");
DEFINE_int32(support_direction, 1, "on +x or -x direction");
// roofs
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



MyPointCloud gCloud1, gCloud2;        // for debug.


ros::Publisher pubPlanes, pubPlaneCenters;
ros::Publisher pubNormalizedCloud, pubNormlizedCar;

Eigen::Matrix4d gGlobalTransform = Eigen::Matrix4d::Identity();



int main(int argc, char **argv){

    google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "process");
    ros::NodeHandle nh;
    ROS_WARN("Process node begin...");

    std::vector<ros::Publisher> pubEachPlane, pubEachSupport;
    pubPlanes = nh.advertise<sensor_msgs::PointCloud2>("/all_planes", 1);
    ros::Publisher pubRoofs = nh.advertise<sensor_msgs::PointCloud2>("/all_roofs", 1);
    ros::Publisher pubValidRoofs = nh.advertise<sensor_msgs::PointCloud2>("/valid_roofs", 1);
    pubPlaneCenters = nh.advertise<sensor_msgs::PointCloud2>("/all_plane_centers", 1);
    pubNormalizedCloud = nh.advertise<sensor_msgs::PointCloud2>("/normalized_clouds", 1);
    pubNormlizedCar = nh.advertise<sensor_msgs::PointCloud2>("/normalized_car_path", 1);
    
    // ros::Publisher debugPub = nh.advertise<sensor_msgs::PointCloud2>("/car_path_before", 1);

    ROS_WARN("Loading data...");
    SceneCloud scene_cloud(nh, FLAGS_file_scene_cloud);
    CarPath car_path(nh, FLAGS_file_carpath);

    ROS_WARN("Calculating global transform...");
    gGlobalTransform = calcGlobalT(car_path);
    updateCoordinate(car_path, scene_cloud, gGlobalTransform);
    scene_cloud.filter(FLAGS_filter_downsampling_size, FLAGS_filter_passthrough_xmin, FLAGS_filter_passthrough_xmax);
    car_path.digitalize();

    ROS_WARN("Segment...");
    ClusterParameter cp(FLAGS_cluster_angle1, FLAGS_cluster_distance1, FLAGS_cluster_size_min1, FLAGS_cluster_size_max1, FLAGS_cluster_radius);
    PlaneParameter pp(FLAGS_plane_l1l3_ratio1, FLAGS_plane_l1l2_ratio1);
    scene_cloud.mergeAllPlanes(cp, pp);

    cp.reset(FLAGS_cluster_angle2, FLAGS_cluster_distance2, FLAGS_cluster_size_min2, FLAGS_cluster_size_max2, FLAGS_cluster_radius);
    pp.reset(FLAGS_plane_l1l3_ratio2, FLAGS_plane_l1l2_ratio2);
    scene_cloud.extractAllRoofs(cp, pp);

    for (int i = 0; i < scene_cloud.getAllRoofs().size(); i++){
        ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/plane_" + std::to_string(i), 1);
        pubEachPlane.push_back(tmp);
    }

    SupportParameter sp;
    sp.setRoof(FLAGS_roof_x_min, FLAGS_roof_x_max, FLAGS_roof_z_min, FLAGS_roof_z_max, FLAGS_roof_norm_angle);
    sp.setSegment(FLAGS_support_segment_x_min, FLAGS_support_segment_x_max, FLAGS_support_segment_y, FLAGS_support_segment_z_min, FLAGS_support_segment_z_max);
    scene_cloud.selectRoofs(car_path, sp);

    for (int i = 0; i < scene_cloud.getAllSupports().size(); i++){
        ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/support_" + std::to_string(i), 1);
        pubEachSupport.push_back(tmp);
    }



    ros::Rate r(10);
    while(ros::ok()){

        scene_cloud.pub();      // full scene cloud
        car_path.pub();        // car path 
        pubPlanes.publish(tool::pointCloud2RosMsg(scene_cloud.getMergedPlanes()));
        pubRoofs.publish(tool::pointCloud2RosMsg(scene_cloud.getMergedRoofs()));
        pubPlaneCenters.publish(tool::pointCloud2RosMsg(*scene_cloud.plane_centers_));
        pubValidRoofs.publish(tool::pointCloud2RosMsg(scene_cloud.getMergedValidRoofs()));

        for (int i = 0; i < pubEachPlane.size(); ++i){
            pubEachPlane[i].publish(tool::pointCloud2RosMsg(scene_cloud.getAllRoofs()[i]));
        }
        for (int i = 0; i < pubEachSupport.size(); ++i){
            pubEachSupport[i].publish(tool::pointCloud2RosMsg(scene_cloud.getAllSupports()[i]));
        }

        ros::spinOnce();
        r.sleep();
    }

#if 0

    scene_cloud.detectPlanes();

    for(int i=0; i<scene_cloud.v_planes_.size(); ++i){
        cout << "No. " << i << " plane, size: " << scene_cloud.v_planes_[i].size() << endl;
    }

    // TODO: Isolate each support.
    // Step 1. Check center->CarPath direction & distance
    // Step 2. Get coordinate along CarPath at each center point
    // Step 3. Rotate to normalize, and isolate by axis;

    // pub each plane
    for(int i = 0; i < scene_cloud.v_planes_.size(); i++){
        ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/plane_" + std::to_string(i), 1);
        pubEachPlane.push_back(tmp);
    }

    ros::Rate r(10);
    while(ros::ok()){

        scene_cloud.pub();      // full scene cloud
        car_path.pub();        // car path 
        
        pubPlanes.publish(tool::pointCloud2RosMsg(gCloud1)); // pub 
        pubPlaneCenters.publish(tool::pointCloud2RosMsg(*scene_cloud.plane_centers_));

        // pub each plane
        for(int i=0; i<pubEachPlane.size(); ++i){
            pubEachPlane[i].publish(tool::pointCloud2RosMsg(scene_cloud.v_planes_[i]));
        }

        ros::spinOnce();
        r.sleep();
    }
#endif

    return 0;
}
