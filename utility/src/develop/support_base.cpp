
#include "process/support_base.h"


int SupportBase::marker_id_ = 0;

bool SupportBase::detectBase(const MyPointCloud& scene_cloud, const Eigen::Matrix4f& init_guess){
    MyPointCloud::Ptr pc_model(new MyPointCloud), pc_scene(new MyPointCloud);
    *pc_model = model_cloud_;
    *pc_scene = scene_cloud;

    pcl::IterativeClosestPoint<MyPoint, MyPoint> icp;
    icp.setInputSource(pc_model);
    icp.setInputTarget(pc_scene);
    
    MyPointCloud tmp;       // not useful.
    icp.align(tmp, init_guess);

    if(icp.hasConverged()){
        cout<<"Base detected. score: " << icp.getFitnessScore() << endl;
        transformation_ = icp.getFinalTransformation();
        return true;
    }
    else{
        cout << "[Warning]. Base not detected in scene..." << endl;
        return false;
    }
}


visualization_msgs::Marker SupportBase::createBaseMarker(Eigen::Vector4f color){
    visualization_msgs::Marker marker;
    marker.header.frame_id="/laser_link";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "file:" + base_mesh_file_;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.b = color(0);
    marker.color.g = color(1);
    marker.color.r = color(2);
    marker.color.a = color(3);
    marker.id = marker_id_++;               // must be unique id
    marker.pose.position.x = transformation_(0,3);
    marker.pose.position.y = transformation_(1,3);
    marker.pose.position.z = transformation_(2,3);
    Eigen::Matrix3f R = transformation_.topLeftCorner(3,3);
    Eigen::Quaternionf q(R);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    return marker;
}
