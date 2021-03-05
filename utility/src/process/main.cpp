


#include "process/scene_cloud.h"
#include "process/car_path.h"
#include "process/tool.h"
#include "process/process.h"


using namespace std;


DEFINE_string(file_scene_cloud, "ver.pcd", "scene cloud path");
DEFINE_string(file_model_cloud, "model.pcd", "model cloud path");
DEFINE_string(file_carpath, "nav_msgs.txt", "car path messages");

DEFINE_double(support_width, 2, "support width");
DEFINE_double(support_height, 5, "support height");

DEFINE_int32(support_direction, 1, "on +x or -x direction");

DEFINE_double(filter_passthrough_xmin, -0.5, "passthrough fitler x range");
DEFINE_double(filter_passthrough_xmax, 5, "passthrough fitler x range");
DEFINE_double(filter_downsampling_size, 0.03, "voxel downsampling size");

DEFINE_double(cluster_radius, 0.1, "clustering nn search radius");
DEFINE_double(cluster_angle_small, M_PI/10, "smaller angle tolerance");
DEFINE_double(cluster_angle_large, M_PI, "larger angle tolerance");
DEFINE_double(cluster_distance_small, 0.2, "clustering distance tolerance");
DEFINE_double(cluster_distance_large, 0.2, "clustering distance tolerance");
DEFINE_double(cluster_size_min, 100, "minimum cluster size");
DEFINE_double(cluster_size_max, 100000, "max cluster size");

DEFINE_double(plane_l1l3_ratio, 50, "plane l1/l3");
DEFINE_double(plane_l1l2_ratio, 5, "plane l1/l2");



pcl::PointCloud<pcl::PointXYZ> gCloud1, gCloud2;        // for debug.
std::vector<ros::Publisher> pubEachPlane;

ros::Publisher pubPlanes, pubPlaneCenters;
ros::Publisher pubNormalizedCloud, pubNormlizedCar;

Eigen::Matrix4d gGlobalTransform = Eigen::Matrix4d::Identity();



int main(int argc, char **argv){

    google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "process");
    ros::NodeHandle nh;

    // pubs for debug
    pubPlanes = nh.advertise<sensor_msgs::PointCloud2>("/all_planes", 1);
    pubPlaneCenters = nh.advertise<sensor_msgs::PointCloud2>("/all_plane_centers", 1);
    pubNormalizedCloud = nh.advertise<sensor_msgs::PointCloud2>("/normalized_clouds", 1);
    pubNormlizedCar = nh.advertise<sensor_msgs::PointCloud2>("/normalized_car_path", 1);
    
    // ros::Publisher debugPub = nh.advertise<sensor_msgs::PointCloud2>("/car_path_before", 1);


    SceneCloud scene_cloud(nh, FLAGS_file_scene_cloud);
    CarPath car_path(nh, FLAGS_file_carpath);

    gGlobalTransform = calcGlobalT(car_path);
    updateCoordinate(car_path, scene_cloud, gGlobalTransform);
    scene_cloud.filter(FLAGS_filter_downsampling_size, FLAGS_filter_passthrough_xmin, FLAGS_filter_passthrough_xmax);
    car_path.digitalize();

    ros::Rate r(10);
    while(ros::ok()){

        scene_cloud.pub();      // full scene cloud
        car_path.pub();        // car path 
        car_path.pubOld();

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
