
#include "process/process.h"
#include "process/tool.h"

DEFINE_double(support_width, 2.2, "Support width.");
DEFINE_double(support_height, 5, "Support height.");
DEFINE_string(path_cloud, "/home/larrydong/ver.pcd", "default scene cloud path");
DEFINE_string(path_car, "/home/larrydong/nav_msgs.txt", "default car path file");

class SupportConfig{
public:
    SupportConfig() { cout << "Error. Not allow empty init." << endl; }
    SupportConfig(double w, double h) : 
        width_(w), height_(h)
    {
        ;
    }
public:
    double width_;
    double height_;
};

using namespace std;





int main(int argc, char **argv){

    google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "extract_model_node");
    ros::NodeHandle nh;

    // check the parameter
    ROS_INFO_STREAM("Support size: " << FLAGS_support_height << ", " << FLAGS_support_width);

    ros::Publisher pubFullOld = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_old", 1);
    ros::Publisher pubFullNew = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_new", 1);
    ros::Publisher pubModel = nh.advertise<sensor_msgs::PointCloud2>("/model", 1);


    pcl::PointCloud<pcl::PointXYZ> pc, pc_new, model;
    string filename = FLAGS_path_cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, pc) == -1){
        cout << "[Error]. Cannot open '" << filename << "'. " << endl;
        return -1;
    }
    string nav_file = FLAGS_path_car;
    CarPath car_path(nav_file);

    Eigen::Matrix4d T = calcGlobalT(car_path);
    pcl::transformPointCloud(pc, pc_new, T.cast<float>());

    // extract model
    


    // pub
    ros::Rate r(10);
    while(ros::ok()){


        pubFullOld.publish(tool::pointCloud2RosMsg(pc));
        pubFullNew.publish(tool::pointCloud2RosMsg(pc_new));
        // pubModel.publish(tool::pointCloud2RosMsg(model));
        
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}
