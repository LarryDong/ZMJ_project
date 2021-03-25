
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
// #include <pcl/PCLPointCloud2.h>
#include <iostream>
#include <string>

using namespace std;

typedef pcl::PointXYZ MyPoint;


int main(int argc, char **argv){

    pcl::PointCloud<MyPoint>::Ptr cloud_in(new pcl::PointCloud<MyPoint>());

    if(argc != 2){
        cout << "Error! Input filename..." << endl;
        return -1;
    }

    string filename = argv[1];

    if (pcl::io::loadPCDFile(filename + ".pcd", *cloud_in) < 0){
        cout << "Error! Cannot load file: " << filename << endl;
        return -2;
    }

    pcl::PLYWriter writer;
    writer.write(filename + ".ply", *cloud_in);

    cout << "Saved..." << endl;

    return 0;
}