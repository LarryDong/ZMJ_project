
#include "process/scene_cloud.h"
#include "process/tool.h"

extern pcl::PointCloud<pcl::PointXYZ> gCloud1, gCloud2;

SceneCloud::SceneCloud(ros::NodeHandle &nh, string filename) : 
    nh_(nh),
    pc_(new pcl::PointCloud<pcl::PointXYZ>()),
    plane_centers_(new pcl::PointCloud<pcl::PointXYZ>())
{
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *pc_) == -1){
        cout << "[Error]. Cannot open '" << filename << "'. " << endl;
        return ;
    }
    pubCloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/scene_cloud", 5);
    preProcess();
}


void SceneCloud::pub(void){     // pub pointcloud
    sensor_msgs::PointCloud2 scene_cloud_msg;
    pcl::toROSMsg(*pc_, scene_cloud_msg);
    scene_cloud_msg.header.frame_id = "/laser_link";
    pubCloud_.publish(scene_cloud_msg);
}


void SceneCloud::preProcess(void){
    ROS_INFO("Preprocess. TODO.");
}

void SceneCloud::filter(void){    
    // 
    ROS_INFO("Filtering SceneCloud. downsampling, passthrough, statistical");

    pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
    double sz = 0.03;
    downSizeFilter.setLeafSize(sz, sz, sz);
    downSizeFilter.setInputCloud(pc_);
    downSizeFilter.filter(*pc_);
    
    // statistical outiers filter
    int K = 50;
    double std = 1;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(pc_);
    sor.setMeanK(K);
    sor.setStddevMulThresh(std);
    sor.filter(*pc_);
    
    // passthrough filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pc_);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.5, 5.0);        // Filter by parameters;
    pass.filter(*pc_);
}

void SceneCloud::detectPlanes(void){

    ROS_INFO("Detect planes in scene...");

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setSearchMethod(tree);
    ne.setInputCloud(pc_);
    ne.setRadiusSearch(0.1);
    ne.compute(*cloud_normals);

    ClusterParameter cp(M_PI / 18, 0.2, 100, 100000);
    // vector<pcl::PointIndices> clusters;
    vector<pcl::PointIndices> cluster_indices;
    extractClusters(*pc_, *cloud_normals, tree, cp, cluster_indices);
    ROS_INFO_STREAM("[Info] Find " << cluster_indices.size() << " clusters for the first time...");


    // STEP 2. Select planes in all clusters and merge into a full pointcloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_all_planes (new pcl::PointCloud<pcl::PointXYZ>);

    int counter = 0;
    int plane_number = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr one_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        // STEP 2.1. extract all points in each cluster to formuate pointcloud
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            one_cluster->push_back((*pc_)[*pit]);           // TODO: maybe can be improved by not using one-by-one push.
        // string cluster_id = "cluster_" + to_string(counter++);

        // STEP 2.2. Check each cluster whether to be a plane
        pcl::MomentOfInertiaEstimation<pcl::PointXYZ> moment;
        moment.setInputCloud(one_cluster);
        moment.compute();
        Eigen::Vector3f vx, vy, vz;         // eigen vector xyz
        float l1, l2, l3;                   // lamba 1-3
        moment.getEigenVectors(vx, vy, vz);
        moment.getEigenValues(l1, l2, l3);

        // check plane by e-value. l1 > l2 >> l3
        bool is_plane = false;           // TODO: check...
        if (l1 > 50*l3 && l1 < 5*l2){       // plane is sqare, so l1 \approx l2
            is_plane = true;
            plane_number++;
        }
        
        if(is_plane){       // merge into a full cloud;
            *cloud_all_planes += *one_cluster;
            RandColor rc;
        }
    }

    ROS_INFO_STREAM("Find plane number: " << plane_number);

    if(cloud_all_planes->empty()){
        ROS_ERROR("[Error]. Empty cloud");
        return ;
    }
    gCloud1 = *cloud_all_planes;



    // TODO: Step 3. Merge again.

    // STEP 3. Use new full pointcloud to find plane, using a more loose parameters
    v_planes_.resize(0);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> cluster_indices2;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne2;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    tree2->setInputCloud(cloud_all_planes);
    ne2.setSearchMethod(tree2);
    ne2.setInputCloud(cloud_all_planes);
    ne2.setRadiusSearch(0.1);
    ne2.compute(*cloud_normals2);
    ClusterParameter cp2(M_PI, 0.1, 1000, 100000);
    extractClusters(*cloud_all_planes, *cloud_normals2, tree2, cp2, cluster_indices2);

    for (auto it = cluster_indices2.begin (); it != cluster_indices2.end (); ++it){
        pcl::PointCloud<pcl::PointXYZ>::Ptr one_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            one_cluster->push_back((*cloud_all_planes)[*pit]);    // TODO: maybe can be improved by not using one-by-one push.
        
        // TODO: Maybe check the eigen value again...
        v_planes_.push_back(*one_cluster);
    }
    ROS_INFO_STREAM("Detect " << v_planes_.size() << " planes");


    // calculate centers
    Eigen::Vector4d center;
    for(auto pc : v_planes_){
        pcl::compute3DCentroid(pc, center);
        pcl::PointXYZ c(center(0), center(1), center(2));
        plane_centers_->push_back(c);
    }

    ROS_INFO("Detect plane finished.");
}

void SceneCloud::extractClusters(
        const pcl::PointCloud<pcl::PointXYZ> &cloud,
        const pcl::PointCloud<pcl::Normal> &normals,
        const pcl::search::Search<pcl::PointXYZ>::Ptr &tree,
        const ClusterParameter &param,
        std::vector<pcl::PointIndices> &clusters)
{
    if (tree->getInputCloud ()->points.size () != cloud.points.size ()){
        PCL_ERROR ("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset (%lu) than the input cloud (%lu)!\n", tree->getInputCloud ()->points.size (), cloud.points.size ());
        return;
    }
    if (cloud.points.size () != normals.points.size ()){
        PCL_ERROR ("[pcl::extractEuclideanClusters] Number of points in the input point cloud (%lu) different than normals (%lu)!\n", cloud.points.size (), normals.points.size ());
        return;
    }

        // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed(cloud.points.size(), false);
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
            if (!tree->radiusSearch (seed_queue[sq_idx], param.delta_distance, nn_indices, nn_distances)){
                sq_idx++;
                continue;
            }
            for (size_t j = 1; j < nn_indices.size (); ++j){        // nn_indices[0] should be sq_idx
                if (processed[nn_indices[j]])                       // Has this point been processed before ?
                    continue;
                auto pt1 = normals.points[i].normal, pt2 = normals.points[nn_indices[j]].normal;
                double dot_p = pt1[0] * pt2[0] + pt1[1] * pt2[1] + pt1[2] * pt2[2];
                if (nn_distances[j] < param.delta_distance && fabs(acos(dot_p)) < param.delta_angle){
                    processed[nn_indices[j]] = true;
                    seed_queue.push_back (nn_indices[j]);
                }
            }
            sq_idx++;
        }

        // If this queue is satisfactory, add to the clusters
        if (seed_queue.size() >= param.min_num && seed_queue.size() <= param.max_num){
            pcl::PointIndices r;
            r.indices.resize(seed_queue.size());
            for (size_t j = 0; j < seed_queue.size(); ++j)
                r.indices[j] = seed_queue[j];
            r.header = cloud.header;
            clusters.push_back(r); // TODO: We could avoid a copy by working directly in the vector
        }
    }
}