
// #include <iostream>
// #include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/Dense>

// using namespace std;

// int main(int argc, char **argv){

//     cout << "Test" << endl;

//     Eigen::Quaterniond q(0, 0, 0, 1);
//     Eigen::Matrix4d T;
//     T.topLeftCorner(3,3) = q.toRotationMatrix();

//     Eigen::Vector4d t;
//     t << 1, 2, 3, 1;
//     T.topRightCorner(4,1) = Eigen::Vector4d(1,2,3,4);
//     cout << T << endl;
//     return 0;
// }

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width = 50;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (auto &point : *cloud)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    std::cerr << "Cloud before filtering: " << std::endl;
    for (const auto &point : *cloud)
        std::cerr << "    " << point.x << " "
                  << point.y << " "
                  << point.z << std::endl;

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*cloud_filtered);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("x");
    // pass.setFilterLimits(0.0, 1.0);
    pass.filter(*cloud_filtered);

        pass.setFilterFieldName("y");
    // pass.setFilterLimits(0.0, 1.0);
    pass.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    for (const auto &point : *cloud_filtered)
        std::cerr << "    " << point.x << " "
                  << point.y << " "
                  << point.z << std::endl;

    return (0);
}
