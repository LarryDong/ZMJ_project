
// #include "dataloader.hpp"
#include "pcl_process/car_path.h"
#include "pcl_process/scene_cloud.h"

#include <gflags/gflags.h>

#include <iostream>

using namespace std;

DEFINE_string(ver_saving, "/home/larrydong/lidar_ws/output/raw/ver.pcd", "vertical point cloud saved");
// DEFINE_string(hor_saving, "/home/larrydong/lidar_ws/output/raw/hor.pcd", "horizontal point cloud saved");
DEFINE_string(car_path_saving, "/home/larrydong/lidar_ws/output/raw/car_path.txt", "car path pointcloud.");



int main(int argc, char **argv){

    google::ParseCommandLineFlags(&argc, &argv, true);
    cout << "Load ver.pcd from: " << FLAGS_ver_saving << endl;
    cout << "Load car path from: " << FLAGS_car_path_saving << endl;

    CarPath car_path(FLAGS_car_path_saving);
    SceneCloud scene_cloud(FLAGS_ver_saving);

    return 0;
}

