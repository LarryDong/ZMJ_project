
#include "process/car_path.h"
#include "process/tool.h"

#include <pcl/filters/passthrough.h>

// bool comp_smaller(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2){
//     return p1.y < p2.y;
// }
// bool comp_bigger(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2){
//     return p1.y > p2.y;
// }
bool comp(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2){
    return fabs(p1.y) < fabs(p2.y);
}

CarPath::CarPath(ros::NodeHandle &nh, string filename) : 
    nh_(nh),
    step_(0.01),
    pc_(new pcl::PointCloud<pcl::PointXYZ>()),
    pc_ori_(new pcl::PointCloud<pcl::PointXYZ>())

{
    ifstream in(filename);
    if (!in.is_open()){
        cout << "[Error]. Cannot load nav_msgs" << endl;
        return;
    }
    while (!in.eof()){
        double tmp; // orientation information is useless;
        pcl::PointXYZ p;
        in >> p.x >> p.y >> p.z >> tmp >> tmp >> tmp >> tmp;
        pc_->push_back(p);
    }
    
    if(pc_->size()==0){
        cout << "[Error]. Empty car path" << endl;
        return ;
    }

    pubCloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/car_path_cloud", 5);

    *pc_ori_ = *pc_;
    pubCloud_ori_= nh_.advertise<sensor_msgs::PointCloud2>("/car_path_cloud_old", 5);


    moving_direction_ = -1;     // TODO: load settings.
    preProcess();
}


void CarPath::pub(void) {
    sensor_msgs::PointCloud2 car_msg;
    pcl::toROSMsg(*pc_, car_msg);
    car_msg.header.frame_id = "/laser_link";
    pubCloud_.publish(car_msg);
}


// Not used. Only for debug.
void CarPath::pubOld(void) {
    sensor_msgs::PointCloud2 car_msg;
    pcl::toROSMsg(*pc_ori_, car_msg);
    car_msg.header.frame_id = "/laser_link";
    pubCloud_ori_.publish(car_msg);
}



// smooth the path;
void CarPath::preProcess(void){
    vector<pcl::PointXYZ> points, pts2;
    points.resize(pc_->size());
    for(int i=0; i<pc_->size(); ++i){
        points[i] = (*pc_)[i];
    }

    // 1. delete dense_points region. May stayed at some places.
    sort(points.begin(), points.end(), comp);       // no matter to y-/y+, always abs smaller.
    assert(points.size() != 0);
    pcl::PointXYZ p_old = points[0];
    for(int i=1; i<points.size()-1; i++){
        pcl::PointXYZ p = points[i];
        if(fabs(p.y - p_old.y) > 0.02){         // 2 cm in y-axis;
            pts2.push_back(p);
        }
        p_old = p;
    }

    // 2. Smooth by two-neighbor average. (1:2:1)/4
    pc_->resize(0);
    for(int i=1; i<pts2.size()-1; ++i){
        pcl::PointXYZ pb = pts2[i - 1], p = pts2[i], pn = pts2[i + 1];
        p.x = (pb.x + pn.x + p.x * 2) / 4;
        p.y = (pb.y + pn.y + p.y * 2) / 4;
        p.z = (pb.z + pn.z + p.z * 2) / 4;
        pc_->push_back(p);
    }
}


double CarPath::getClosestPointInPath(const pcl::PointXYZ& in, pcl::PointXYZ& out){
    int min_idx = -1;
    double min_distance = 999;
    for (int i = 0; i < pc_->size(); ++i){
        pcl::PointXYZ p = (*pc_)[i];
        double dist = tool::calDistance(in, p);
        if (dist < min_distance){
            min_distance = dist;
            min_idx = i;
        }
    }
    out = (*pc_)[min_idx];
    return min_distance;
}


// to 1cm along y-axis. ATTENTION: always -y direction;
void CarPath:: digitalize(void){

    vector<pcl::PointXYZ> pts;      // save pointcloud tmply.
    pts.resize(pc_->size());
    for(int i=0; i<pc_->size(); ++i)
        pts[i] = (*pc_)[i];

    pcl::PointXYZ pb = getBeginPoint(), pe = getEndPoint();
    int idx_num = (int)(fabs(pe.y - pb.y) / this->step_);       // index from 0 - end, each 1cm
    pc_->clear();
    pc_->resize(0);

    // interpolation (linear) along y-axis;
    for(int i=0; i<idx_num; ++i){
        double py = -i * this->step_;
        pcl::PointXYZ p_pre = pb;       // init values
        pcl::PointXYZ p_next = pe;
        pcl::PointXYZ p_insert;
        for(int j=0; j<pts.size() - 1; ++j){
            // cout << "p-j: " << getPoint(j).y << ", p-j+1: " << getPoint(j + 1).y << endl;
            if(fabs(py) < fabs(pts[j].y) || fabs(py) > fabs(pts[j+1].y))     // skip
                continue;
            if(fabs(py) >= fabs(pts[j].y))
                p_pre = pts[j];
            if(fabs(py) <= fabs(pts[j+1].y))
                p_next = pts[j+1];
        }
        double dx = p_next.x - p_pre.x;
        double dy = p_next.y - p_pre.y;
        double dz = p_next.z - p_pre.z;
        p_insert.y = py;
        double scale = fabs((p_insert.y - p_pre.y) / dy);
        p_insert.x = p_pre.x + (dx * scale);        // y-axis is negative;
        p_insert.z = p_pre.z + (dz * scale);        // z-axis is positive;

        pc_->push_back(p_insert);
    }

}