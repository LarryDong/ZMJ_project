
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <Eigen/Core>
#include <string>

using namespace std;

typedef pcl::PointXYZ MyPoint;
int main(int argc, char **argv){

    pcl::PointCloud<MyPoint>::Ptr cloud_in(new pcl::PointCloud<MyPoint>());

    string fin= "isolated_support.pcd", fout="normalized_support.pcd";
    fin = (argc >= 2) ? argv[1] : fin;
    fout = (argc >= 3) ? argv[2] : fout;

    string type = "zxy";
    type = (argc == 4) ? argv[3] : type;
    cout << "Axis type: " << type << endl;

    if(pcl::io::loadPCDFile<MyPoint>(fin, *cloud_in) == -1){
        cout << "Cannot open '" << fin << "'. " << endl;
        return -1;
    }

    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> moment;
    moment.setInputCloud(cloud_in);
    moment.compute();

    Eigen::Vector3f center, vx, vy, vz;

    // principle axis are different.
    if (type == "xyz")
        moment.getEigenVectors(vx, vy, vz);
    else if (type == "xzy")
        moment.getEigenVectors(vx, vz, vy);
    else if (type == "yxz")
        moment.getEigenVectors(vy, vx, vz);
    else if (type == "yzx")
        moment.getEigenVectors(vy, vz, vx);
    else if (type == "zxy")
        moment.getEigenVectors(vz, vx, vy);
    else if (type == "zyy")
        moment.getEigenVectors(vz, vy, vx); // z-x-y
    else {        
        cout << "Input type error!" << endl;
        return -1;
    }

    moment.getMassCenter(center);
    cout << "Center: (" << center[0] << ", " << center[1] << ", " << center[2] << ")." << endl;

    Eigen::Matrix3f rotation;
    rotation.col(0) = vx[0] > 0 ? vx : -vx;     // avoid "upside down"
    rotation.col(1) = vy[1] > 0 ? vy : -vy;
    rotation.col(2) = vz[2] > 0 ? vz : -vz;
    cout << "Rotation matrix: ------ \n" << rotation << endl;

    pcl::PointCloud<MyPoint>::Ptr cloud_out(new pcl::PointCloud<MyPoint>());
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.topLeftCorner(3, 3) = rotation.transpose();
    T.topRightCorner(3, 1) = -center;
    pcl::transformPointCloud(*cloud_in, *cloud_out, T);

    pcl::io::savePCDFileASCII(fout, *cloud_out);
    cout << "Normalized pointcloud saved. Size: " << cloud_out->size() << endl;

    // Initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color(cloud_in, 255, 255, 255);
    viewer_final->addPointCloud<pcl::PointXYZ>(cloud_in, cloud_in_color, "input cloud color");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "input cloud color");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_color(cloud_out, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(cloud_out, cloud_out_color, "out cloud color");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "out cloud color");


    // Starting visualizer
    viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();

    while (!viewer_final->wasStopped()){
        viewer_final->spinOnce(100);
    }
}
