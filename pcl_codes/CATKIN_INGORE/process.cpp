
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Core>

#include "tool.h"
#include <cstdlib>      // rand()

using namespace std;

class color{            // random color
public:
    color(int size){
        this->size = size;
        r = rand() % 255;
        g = rand() % 255;
        b = rand() % 255;
    }
    double r;
    double g;
    double b;
    double size; // 点的大小
};



void extractEuclideanClusters(
    const pcl::PointCloud<pcl::PointXYZ> &cloud, const pcl::PointCloud<pcl::Normal> &normals,
    float tolerance, const pcl::search::Search<pcl::PointXYZ>::Ptr &tree,
    std::vector<pcl::PointIndices> &clusters, double eps_angle,
    unsigned int min_pts_per_cluster = 1,
    unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max)()){

    if (tree->getInputCloud ()->points.size () != cloud.points.size ()){
        PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!\n", tree->getInputCloud ()->points.size (), cloud.points.size ());
        return;
    }
    if (cloud.points.size () != normals.points.size ()){
        PCL_ERROR ("[pcl::extractEuclideanClusters] Number of points in the input point cloud (%lu) different than normals (%lu)!\n", cloud.points.size (), normals.points.size ());
        return;
    }

    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed (cloud.points.size (), false);
    std::vector<int> nn_indices;
    std::vector<float> nn_distances;

    // Process all points in the indices vector
    for (size_t i = 0; i < cloud.points.size (); ++i){

        if (processed[i])
            continue;

        std::vector<unsigned int> seed_queue;
        int sq_idx = 0;
        seed_queue.push_back(static_cast<int>(i));      //~ find the first seed.
        processed[i] = true;

        while (sq_idx < static_cast<int> (seed_queue.size ())){
            // Search for sq_idx
            if (!tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances)){
                sq_idx++;
                continue;
            }
            for (size_t j = 1; j < nn_indices.size (); ++j){        // nn_indices[0] should be sq_idx
                if (processed[nn_indices[j]])                       // Has this point been processed before ?
                    continue;

                auto pt1 = normals.points[i].normal, pt2 = normals.points[nn_indices[j]].normal;
                double dot_p = pt1[0] * pt2[0] + pt1[1] * pt2[1] + pt1[2] * pt2[2];

                if (nn_distances[j] < tolerance && fabs(acos(dot_p)) < eps_angle){ //  fabs (acos (dot_p)) < eps_angle
                    processed[nn_indices[j]] = true;
                    seed_queue.push_back (nn_indices[j]);
                }
            }
            sq_idx++;
        }

        // If this queue is satisfactory, add to the clusters
        if (seed_queue.size() >= min_pts_per_cluster && seed_queue.size() <= max_pts_per_cluster){
            pcl::PointIndices r;
            r.indices.resize(seed_queue.size());
            for (size_t j = 0; j < seed_queue.size(); ++j)
                r.indices[j] = seed_queue[j];
            r.header = cloud.header;
            clusters.push_back(r); // TODO: We could avoid a copy by working directly in the vector
        }
    }
}



int main (int argc, char** argv){

    // STEP 0. Read pointcloud and construct fnntree
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    string file_name = (argc == 2) ? argv[1] : "ver_trans.pcd";
    if(reader.read(file_name, *cloud_in)){
        cout << "[Error] cannot read file: " << file_name << endl;
        return -1;
    }

    pcl::visualization::PCLVisualizer viewer("cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_handler(cloud_in, 128, 128, 128);      // default color:
    viewer.addPointCloud(cloud_in, cloud_in_handler, "cloud_in");

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_in);
    ne.setKSearch(25);
    ne.compute(*cloud_normals);


    // STEP 1. Cluster with a strict value, get all clusters in cloud_in;
    double eps_angle = M_PI / 10;       // \delta angle of normals
    float tolerance=0.2;
    std::vector<pcl::PointIndices> clusters;
    extractEuclideanClusters(*cloud_in, *cloud_normals, tolerance, tree, cluster_indices, eps_angle, 800, 20000);
    std::cout << "[Info] Find " << cluster_indices.size() << " clusters for the first time..." << std::endl;

    // STEP 2. Select planes in all clusters and merge into a full pointcloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_all_planes (new pcl::PointCloud<pcl::PointXYZ>);

    int counter = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr one_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        // STEP 2.1. extract all points in each cluster to formuate pointcloud
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            one_cluster->push_back((*cloud_in)[*pit]);           // TODO: maybe can be improved by not using one-by-one push.
        
        string cluster_id = "cluster_" + to_string(counter++);


        // STEP 2.2. Check each cluster whether to be a plane
        pcl::MomentOfInertiaEstimation<pcl::PointXYZ> moment;
        moment.setInputCloud(one_cluster);
        moment.compute();
        Eigen::Vector3f vx, vy, vz;         // eigen vector xyz
        float l1, l2, l3;                   // lamba 1-3
        moment.getEigenVectors(vx, vy, vz);
        moment.getEigenValues(l1, l2, l3);

        // check plane by e-value. l1 > l2 >> l3
        bool is_plane = false;
        if (l1 > 100*l3 && l1 < 2*l2)       // plane is sqare, so l1 \approx l2
            is_plane = true;
        
        if(is_plane){       // merge into a full cloud;
            *cloud_all_planes += *one_cluster;
            color rc(1);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_cluster_color_handler(one_cluster, rc.r, rc.g, rc.b);
            // viewer.addPointCloud(one_cluster, cloud_cluster_color_handler, cluster_id);
        }
        // else{
        //     continue;
        // }
    }
    if(cloud_all_planes->empty()){
        cout << "[Error]. Empty cloud" << endl;
        return -1;
    }

    // STEP 3. Use new full pointcloud to find plane, using a more loose parameters
    // STEP 3.1. Filter points by experiences.
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_roof_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    double height_min = 1.5, height_max = 5;        // TODO: parameters loaded from configure files
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_all_planes);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(height_min, height_max);
    pass.filter(*all_roof_cloud);
    if(all_roof_cloud->empty()){
        cout<<"[Error]. Empyt roof_cloud"<<endl;
    }

    // STEP 3.2. Cluster again using loose parameters;
    // TODO: duplicated variables
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> cluster_indices2;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne2;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    tree2->setInputCloud(all_roof_cloud);
    ne2.setSearchMethod(tree2);
    ne2.setInputCloud(all_roof_cloud);
    ne2.setKSearch(25);
    ne2.compute(*cloud_normals2);    
    eps_angle = M_PI;       // this time, angle is +- pi \degree
    tolerance = 0.1;
    extractEuclideanClusters(*all_roof_cloud, *cloud_normals2, tolerance, tree2, cluster_indices2, eps_angle, 300, 400000);

    // STEP 3.3. Extract each powered-proof by roof center
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vProofs;        // TODO:
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_roof_cloud_view(new pcl::PointCloud<pcl::PointXYZ>);
    // TODO: use a function instead of a second loop.
    for (auto it = cluster_indices2.begin (); it != cluster_indices2.end (); ++it){
        pcl::PointCloud<pcl::PointXYZ>::Ptr one_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            one_cluster->push_back((*all_roof_cloud)[*pit]);    // TODO: maybe can be improved by not using one-by-one push.
        
        // TODO: check each cluster to make sure is a roof
        // could use cross(normal, y-axis) to check.

        string cluster_id = "roof_id_" + to_string(counter++);
        // merge all proof for view
        *all_roof_cloud_view += *one_cluster;
        color rc(2);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_cluster_color_handler(one_cluster, rc.r, rc.g, rc.b);
        viewer.addPointCloud(one_cluster, cloud_cluster_color_handler, cluster_id);

        vProofs.push_back(one_cluster);

        
    }
    // TODO: isolate each powered proof.

    cout << "[Info]. roof number: " << vProofs.size() << endl;

    viewer.addCoordinateSystem(10.0);

    while (!viewer.wasStopped ()) 
    { 
        viewer.spinOnce ();
    }

    return (0);
}