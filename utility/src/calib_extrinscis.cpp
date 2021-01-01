// NDT-ICP algorithm. To align two points cloud, for extrinsics.
//                      -- created by dongy. Modified: 20210101


#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

typedef pcl::PointXYZ MyPoint;
int main(int argc, char **argv){

    pcl::PointCloud<MyPoint>::Ptr cloud_in(new pcl::PointCloud<MyPoint>());
    pcl::PointCloud<MyPoint>::Ptr cloud_out(new pcl::PointCloud<MyPoint>());

    if(pcl::io::loadPCDFile<MyPoint>("ver.pcd", *cloud_in) == -1){
        cout << "Cannot open 'ver.pcd'. " << endl;
    }
    if(pcl::io::loadPCDFile<MyPoint>("hor.pcd", *cloud_out) == -1){
        cout << "Cannot open 'hor.pcd'. " << endl;
    }

    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations (35);
    ndt.setInputSource (cloud_in);
    ndt.setInputTarget (cloud_out);

    // Set initial alignment estimate found using robot odometry.
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    // Calculating required rigid transform to align the input cloud to the target cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr registered_pc(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*registered_pc, init_guess);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged() << " score: " << ndt.getFitnessScore() << std::endl;
    auto deltaT = ndt.getFinalTransformation();
    std::cout << "Transform: \n" << deltaT << std::endl;

    std::ofstream my_file;
    my_file.open("delta_T.txt", ios::out | ios::binary);
    my_file << deltaT;
    my_file.close();
    std::cout << "Saved to 'delta_T.txt'" << std::endl;

    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud(*cloud_in, *registered_pc, ndt.getFinalTransformation());

    // Initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color(cloud_in, 0, 0, 255);
    viewer_final->addPointCloud<pcl::PointXYZ>(cloud_in, cloud_in_color, "input cloud color");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "input cloud color");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_color(cloud_out, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(cloud_out, cloud_out_color, "input cloud registered");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "input cloud registered");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(registered_pc, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(registered_pc, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "output cloud");

    // Starting visualizer
    viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();

    std::cout << "[Color]: red-gt, green-registered, blue-original." << std::endl;

    while (!viewer_final->wasStopped()){
        viewer_final->spinOnce(100);
    }
}
