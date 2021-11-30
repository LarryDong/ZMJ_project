#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>

using namespace std;

int main(int argc, char **argv){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    string fin = "default.pcd";
    fin = (argc >= 2) ? argv[1] : fin;
    int K = 50;
    double std = 1;

    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZ>(fin, *cloud_in);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud_in << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_in);
    sor.setMeanK(K);
    sor.setStddevMulThresh(std);
    sor.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("statistical_filtered.pcd", *cloud_filtered, false);

    // Initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color(cloud_in, 255, 255, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_in, cloud_in_color, "input cloud color");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "input cloud color");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_filtered_color(cloud_filtered, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, cloud_filtered_color, "filtered cloud color");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "filtered cloud color");

    // Starting visualizer
    viewer->addCoordinateSystem(1.0, "global");
    viewer->initCameraParameters();

    while (!viewer->wasStopped()){
        viewer->spinOnce(100);
    }

    return (0);
}