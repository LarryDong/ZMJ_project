
// 1. load pc
// 2. show pc in rviz
// 3. ..



#include "process/scene_cloud.h"
#include "process/car_path.h"
#include "process/tool.h"
#include "process/process.h"


using namespace std;


pcl::PointCloud<pcl::PointXYZ> gCloud1, gCloud2;        // for debug.
std::vector<ros::Publisher> pubEachPlane;

ros::Publisher pubPlanes, pubPlaneCenters;
ros::Publisher pubNormalizedCloud, pubNormlizedCar;

Eigen::Matrix4d gGlobalTransform = Eigen::Matrix4d::Identity();



int main(int argc, char **argv){

    ros::init(argc, argv, "process");
    ros::NodeHandle nh;

    // pubs for debug
    pubPlanes = nh.advertise<sensor_msgs::PointCloud2>("/all_planes", 1);
    pubPlaneCenters = nh.advertise<sensor_msgs::PointCloud2>("/all_plane_centers", 1);
    pubNormalizedCloud = nh.advertise<sensor_msgs::PointCloud2>("/normalized_clouds", 1);
    pubNormlizedCar = nh.advertise<sensor_msgs::PointCloud2>("/normalized_car_path", 1);

    string filename = "ver.pcd";
    SceneCloud scene_cloud(nh, filename);

    string nav_file = "nav_msgs.txt";
    CarPath car_path(nh, nav_file);

    gGlobalTransform = calcGlobalT(car_path);
    updateCoordinate(car_path, scene_cloud, gGlobalTransform);
    scene_cloud.filter();



    ros::Rate r(10);
    while(ros::ok()){

        scene_cloud.pub();      // full scene cloud
        car_path.pub();        // car path 

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