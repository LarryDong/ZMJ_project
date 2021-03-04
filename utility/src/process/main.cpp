
// 1. load pc
// 2. show pc in rviz
// 3. ..


#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>

#include <eigen3/Eigen/Core>

#include "process/scene_cloud.h"
#include "process/car_path.h"
#include "process/tool.h"


using namespace std;


pcl::PointCloud<pcl::PointXYZ> gCloud1, gCloud2;        // for debug.
std::vector<ros::Publisher> pubEachPlane;

ros::Publisher pubPlanes, pubPlaneCenters;
ros::Publisher pubNormalizedCloud, pubNormlizedCar;

Eigen::Matrix4d gGlobalTransform = Eigen::Matrix4d::Identity();


void calcGlobalT(const CarPath& cp){
    Eigen::Vector3d pb = tool::xyz2vector(cp.getBeginPoint());  // path_begin
    Eigen::Vector3d pe = tool::xyz2vector(cp.getEndPoint());
    Eigen::Vector3d dir = pe - pb;
    dir.normalize();

    cout << "car from: (" << pb(0) << ", " << pb(1) << ", " << pb(2)
        << "), to: (" << pe(0) << ", " << pe(1) << ", " << pe(2) << ")" << endl;

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
    gGlobalTransform.topLeftCorner<3, 3>(0, 0) = R.transpose();
    gGlobalTransform.col(3).head(3) = -R.transpose() * pb;

    cout << "Final transform (from old points to new points): \n" << gGlobalTransform << endl;
}


void updateCoordinate(CarPath& cp, SceneCloud& sc, const Eigen::Matrix4d& T){
    pcl::PointCloud<pcl::PointXYZ> new_scene_pc;
    pcl::transformPointCloud(*(sc.pc_), new_scene_pc, T.cast<float>());
    *(sc.pc_) = new_scene_pc;
    
    pcl::PointCloud<pcl::PointXYZ> new_path_pc;
    pcl::transformPointCloud(*(cp.pc_), new_path_pc, T.cast<float>());
    *(cp.pc_) = new_path_pc;
}


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

    calcGlobalT(car_path);
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