
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <string>

using namespace std;

typedef pcl::PointXYZ MyPoint;
int main(int argc, char **argv){

    pcl::PointCloud<MyPoint>::Ptr cloud_in(new pcl::PointCloud<MyPoint>());

    string filename = "isolated_support.pcd";
    if(argc == 2)
        filename = argv[1];

    if(pcl::io::loadPCDFile<MyPoint>(filename, *cloud_in) == -1){
        cout << "Cannot open '" << filename << "'. " << endl;
    }

    // Initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color(cloud_in, 255, 255, 255);
    viewer_final->addPointCloud<pcl::PointXYZ>(cloud_in, cloud_in_color, "input cloud color");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "input cloud color");

    // Starting visualizer
    viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();

    while (!viewer_final->wasStopped()){
        viewer_final->spinOnce(100);
    }
}
