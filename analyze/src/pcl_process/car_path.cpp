
#include "pcl_process/car_path.h"
#include "tool.h"

#include <pcl/filters/passthrough.h>


bool comp(const MyPoint &p1, const MyPoint &p2){
    return fabs(p1.y) < fabs(p2.y);
}

CarPath::CarPath(string filename) : 
    step_(0.01),
    pc_(new MyPointCloud()),
    pc_ori_(new MyPointCloud())

{
    ifstream in(filename);
    if (!in.is_open()){
        cout << "[Error]. Cannot load nav_msgs" << endl;
        return;
    }
    while (!in.eof()){
        double tmp; // orientation information is useless;
        MyPoint p;
        in >> p.x >> p.y >> p.z >> tmp >> tmp >> tmp >> tmp;
        pc_->push_back(p);
    }
    if(pc_->size()==0){
        cout << "[Error]. Empty car path" << endl;
        return ;
    }
    cout << "Loaded " << pc_->size() << " carPath from " << filename << endl;

    *pc_ori_ = *pc_;
    preProcess();
}



// smooth the path;
void CarPath::preProcess(void){
    vector<MyPoint> points, pts2;
    points.resize(pc_->size());
    for(int i=0; i<pc_->size(); ++i){
        points[i] = (*pc_)[i];
    }

    // 1. delete dense_points region. May stayed at some places.
    sort(points.begin(), points.end(), comp);       // no matter to y-/y+, always abs smaller.
    assert(points.size() != 0);
    MyPoint p_old = points[0];
    for(int i=1; i<points.size()-1; i++){
        MyPoint p = points[i];
        if(fabs(p.y - p_old.y) > 0.02){         // 2 cm in y-axis;
            pts2.push_back(p);
        }
        p_old = p;
    }

    // 2. Smooth by two-neighbor average. (1:2:1)/4
    pc_->resize(0);
    for(int i=1; i<pts2.size()-1; ++i){
        MyPoint pb = pts2[i - 1], p = pts2[i], pn = pts2[i + 1];
        p.x = (pb.x + pn.x + p.x * 2) / 4;
        p.y = (pb.y + pn.y + p.y * 2) / 4;
        p.z = (pb.z + pn.z + p.z * 2) / 4;
        pc_->push_back(p);
    }
}

// since car_path are 1cm increasing in x-axis, only find by x-coordinate.
double CarPath::getClosestPointInPath(const MyPoint& in, MyPoint& out){
    int idx = abs((int)(in.y / this->step_));
    out = (*pc_)[idx];
    return tool::calDistance(in, out);
}


// to 1cm along y-axis. ATTENTION: always -y direction;
void CarPath:: digitalize(void){

    vector<MyPoint> pts;      // save pointcloud tmply.
    pts.resize(pc_->size());
    for(int i=0; i<pc_->size(); ++i)
        pts[i] = (*pc_)[i];

    MyPoint pb = getBeginPoint(), pe = getEndPoint();
    int idx_num = (int)(fabs(pe.y - pb.y) / this->step_);       // index from 0 - end, each 1cm
    pc_->clear();
    pc_->resize(0);

    // interpolation (linear) along y-axis;
    for(int i=0; i<idx_num; ++i){
        double py = -i * this->step_;
        MyPoint p_pre = pb;       // init values
        MyPoint p_next = pe;
        MyPoint p_insert;
        for(int j=0; j<pts.size() - 1; ++j){
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