
#include "scene_cloud.h"


SceneCloud::SceneCloud(string filename) : pc_(new MyPointCloud()),
                                            plane_centers_(new MyPointCloud())
{
    if (pcl::io::loadPCDFile<MyPoint>(filename, *pc_) == -1){
        cout << "[Error]. Cannot open '" << filename << "'. " << endl;
        return ;
    }
    cout << "Loaded " << pc_->size() << " pts in sceneCloud from: " << filename << endl;
}


void SceneCloud::filter(double sz, double xmin, double xmax){    
    ROS_INFO_STREAM("Filter. ds: " << sz << ", ps: [" << xmin << ", " << xmax << "], and statistical");
    assert(xmax > xmin && sz > 0);

    pcl::VoxelGrid<MyPoint> downSizeFilter;
    downSizeFilter.setLeafSize(sz, sz, sz);
    downSizeFilter.setInputCloud(pc_);
    downSizeFilter.filter(*pc_);
    
    // statistical outiers filter
    int K = 50;
    double std = 1;
    pcl::StatisticalOutlierRemoval<MyPoint> sor;
    sor.setInputCloud(pc_);
    sor.setMeanK(K);
    sor.setStddevMulThresh(std);
    sor.filter(*pc_);
    
    // passthrough filter
    pcl::PassThrough<MyPoint> pass;
    pass.setInputCloud(pc_);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(xmin, xmax);        // Filter by parameters;
    pass.filter(*pc_);
}


int SceneCloud::filerByClustering(const ClusterParameter& cp, const PlaneParameter& pp){
    ROS_INFO("Merge all planes in scene cloud...");
    ROS_INFO_STREAM("Settings. r: " << cp.search_radius << ", angle/dist: " << cp.delta_angle << "/" << cp.delta_distance 
                    << ", size:(" << cp.min_num << ", " << cp.max_num << "), ratio: " << pp.l1l3 << "/" << pp.l1l2);
    vector<pcl::PointIndices> cluster_indices;
    extractClusters(*pc_, cp, cluster_indices);
    ROS_INFO_STREAM("[Info] Find " << cluster_indices.size() << " clusters for the first time...");

    int plane_number = 0;
    cluster_filtered_pc_.clear();
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
        MyPointCloud::Ptr one_cluster(new MyPointCloud);
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            one_cluster->push_back((*pc_)[*pit]);           // TODO: maybe can be improved by not using one-by-one push.

        if(checkIsPlane(*one_cluster, pp)){       // merge into a full cloud;
            plane_number++;
            cluster_filtered_pc_ += *one_cluster;
        }
    }

    ROS_INFO_STREAM("Find plane number: " << plane_number);
    return plane_number;
}

int SceneCloud::extractPlanes(const ClusterParameter& cp, const PlaneParameter& pp){
    ROS_INFO("Extract all roofs......");
    ROS_INFO_STREAM("Settings. r: " << cp.search_radius << ", angle/dist: " << cp.delta_angle << "/" << cp.delta_distance 
                    << ", size:(" << cp.min_num << ", " << cp.max_num << "), ratio: " << pp.l1l3 << "/" << pp.l1l2);
    vector<pcl::PointIndices> cluster_indices;
    extractClusters(cluster_filtered_pc_, cp, cluster_indices);
    ROS_INFO_STREAM("[Info] Find " << cluster_indices.size() << " clusters after the 2nd time...");

    int plane_number = 0;
    merged_plane_pc_.clear();
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
        MyPointCloud::Ptr one_cluster(new MyPointCloud);
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            one_cluster->push_back(cluster_filtered_pc_[*pit]);

        if (checkIsPlane(*one_cluster, pp)){
            merged_plane_pc_ += *one_cluster;
            v_roofs_.push_back(*one_cluster);
        }
    }
    ROS_INFO_STREAM("Extract roof number: " << v_roofs_.size());
    return v_roofs_.size();
}


void SceneCloud::extractClusters(const MyPointCloud &cloud, const ClusterParameter &param, std::vector<pcl::PointIndices> &clusters){
    // ROS_INFO_STREAM("Clustering... Settings: r: " << param.search_radius << ", angle: " << param.delta_angle << ", dist: " << param.delta_distance << ", size: (" << param.min_num << ", " << param.max_num << ").");
    MyPointCloud::Ptr pc_ptr (new MyPointCloud);
    pcl::search::KdTree<MyPoint>::Ptr tree(new pcl::search::KdTree<MyPoint>);
    pcl::NormalEstimation<MyPoint, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal> normals;
    ne.setSearchMethod(tree);
    *pc_ptr = cloud;
    ne.setInputCloud(pc_ptr);
    ne.setRadiusSearch(param.search_radius);
    ne.compute(normals);

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




bool SceneCloud::checkIsPlane(const MyPointCloud &cloud_in, const PlaneParameter &pp){
    pcl::MomentOfInertiaEstimation<MyPoint> moment;
    MyPointCloud::Ptr pc_ptr(new MyPointCloud());
    *pc_ptr = cloud_in;
    moment.setInputCloud(pc_ptr);
    moment.compute();
    Eigen::Vector3f vx, vy, vz;         // eigen vector xyz
    float l1, l2, l3;                   // lamba 1-3
    moment.getEigenVectors(vx, vy, vz);
    moment.getEigenValues(l1, l2, l3);


    // check plane by e-value. l1 > l2 >> l3
    bool is_plane = false;
    if (l1 > pp.l1l3 * l3 && l1 < pp.l1l2 * l2) // plane is sqare, so l1 \approx l2
        is_plane = true;

#ifdef DEBUG_OUTPUT
    cout << "[Debug]. plane? " << is_plane << ". l1/l3: " << std::setprecision(2) << l1 / l3 << ", l1/l2: " << std::setprecision(2) << l1 / l2 << endl;
#endif
    return is_plane;
}


int SceneCloud::selectRoofsAndSegment(const CarPath& cp, const SupportParameter& sp){

    vector<bool> is_valid_roofs;
    // vector<MyPoint> v_car_path_points_;
    
    cout << "--------- Select roofs ----------" << endl;
    cout << "Select param: x: [" << sp.roof_x_min << ", " << sp.roof_x_max << "], z: [" << sp.roof_z_min << ", " << sp.roof_z_max << "]. " << endl;
    merged_roof_pc_.clear();
    for(int i=0; i<v_roofs_.size(); ++i){
        Eigen::Vector4f center4;
        pcl::compute3DCentroid(v_roofs_[i], center4);
        // check.
        MyPoint center = tool::vector2xyz(center4.head(3).cast<double>());
        plane_centers_->push_back(center);
        MyPoint np = cp.getAnyPoint((int)(fabs(center.y) * (1.0f / cp.step_))); // nearest path point.

        // cout << "--> Center: [" << center.x << ", " << center.y << ", " << center.z << "]" << endl;
        // cout << "    Path  : [" << np.x << ", " << np.y << ", " << np.z << "]" << endl;

        // check by settings
        bool is_valid = false;
        // 1. check the position:
        if (np.x + sp.roof_x_min < center.x && np.x + sp.roof_x_max > center.x 
        && np.z + sp.roof_z_min < center.z && np.z + sp.roof_z_max > center.z){
            is_valid = true;
        }
        // 2. check to norm direction; TODO: Not implemented yet.

        if(is_valid){
            v_car_path_points_.push_back(np);
            v_valid_roofs_.push_back(v_roofs_[i]);
            merged_roof_pc_ += v_roofs_[i];
        }
        is_valid_roofs.push_back(is_valid);
    }

    cout << "Valid number: "<<v_valid_roofs_.size()<<". Are: ";
    for(int i=0; i<v_roofs_.size(); ++i){
        if(is_valid_roofs[i]==true)
            cout << i << ",";
    }
    cout << endl;


    // segment:
    for(int i=0; i<v_car_path_points_.size(); ++i){
        MyPoint p = v_car_path_points_[i];

        // passthrough filter
        pcl::PassThrough<MyPoint> pass;
        MyPointCloud::Ptr one_pc (new MyPointCloud);

        pass.setInputCloud(pc_);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(p.x + sp.segment_x_min, p.x + sp.segment_x_max);        // Filter by parameters;
        pass.filter(*one_pc);

        pass.setInputCloud(one_pc);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(p.y - sp.segment_y/2, p.y + sp.segment_y/2);        // Filter by parameters;
        pass.filter(*one_pc);

        pass.setInputCloud(one_pc);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(p.z + sp.segment_z_min, p.z + sp.segment_z_max);        // Filter by parameters;
        pass.filter(*one_pc);

        v_supports_.push_back(*one_pc);
    }
    ROS_INFO("Segment support finished...");

    // cout << "Sp: x(" << sp.segment_x_min << ", " << sp.segment_x_max << "), z: (" << sp.segment_z_min << ", " << sp.segment_z_max << "). " << endl;

    return 0;
}

