
#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main(int argc, char **argv){

    cout << "Project all points to the ground" << endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    string fin = "default.pcd";
    fin = (argc >= 2) ? argv[1] : fin;

    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>(fin, *cloud_in);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud_in);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_out);

    // Initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color(cloud_in, 255, 255, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_in, cloud_in_color, "input cloud color");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "input cloud color");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_color(cloud_out, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_out, cloud_out_color, "output cloud color");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "output cloud color");

    // Starting visualizer
    viewer->addCoordinateSystem(1.0, "global");
    viewer->initCameraParameters();

    while (!viewer->wasStopped()){
        viewer->spinOnce(100);
    }

    return (0);
}