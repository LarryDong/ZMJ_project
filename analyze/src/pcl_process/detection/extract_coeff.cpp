
#include "pcl_process/extract_coeff.h"

#include <pcl/features/moment_of_inertia_estimation.h>      // calculate straightness of car-path

#include <gflags/gflags.h>

// roof parameters
DECLARE_double(roof_width);
DECLARE_double(roof_height);
DECLARE_int32(roof_direction);

// requirements:
// Cylinder: 立柱外沿中线位置(TODO:)；相对于地面的俯仰角、朝向；
// Base：底座推移杆伸出长度
// Roof：护帮距离底座的高度；护帮的开合角度
void SupportCoeff::calcResult(void){
    cout << "Calculate result...... " << endl;
    // Cylinder
    lc_dir_ *= (lc_dir_[2] < 0) ? -1 : 1; // convert to "z+" positive;
    rc_dir_ *= (rc_dir_[2] < 0) ? -1 : 1;
    // 1. cylinder's pitch angle to ground.
    double dx, dy, dz;
    dx = lc_dir_[0];
    dy = lc_dir_[1];
    dz = lc_dir_[2];
    double lc_theta = abs(tool::rad2degree(atan2(dz, sqrtf64(dx * dx + dy * dy))));
    dx = rc_dir_[0];
    dy = rc_dir_[1];
    dz = rc_dir_[2];
    double rc_theta = abs(tool::rad2degree(atan2(dz, sqrtf64(dx * dx + dy * dy))));
    cout << "[cylinder] angle: " << lc_theta << " / " << rc_theta << " degree." << endl;
    
    // 2. cylinders' direction
    cout << "[cylinder] direction: " << lc_dir_.transpose() << " / " << rc_dir_.transpose() << endl;

    // base;
    V3d t = base_transform_.topRightCorner(3,1);
    cout << "[base] out distance: " << abs(t[0]) << endl;

    // Roof;
    // calculate the topest point of each roof. center + 1/2 length. Ignore the roll error.
    dx = roof_normal_[0];
    dy = roof_normal_[1];
    dz = roof_normal_[2];
    double pitch = atan2(dz, sqrtf64(dx * dx + dy * dy));
    double delta_h = 0.5 * FLAGS_roof_height * sin(0.5 * PI - pitch);
    double height = roof_center_[2] + delta_h - t[2];
    cout << "[roof] center: " << roof_center_.transpose() << ". Height: " << height << endl;
}


void TraceCoeff::calcStraightness(void){
#ifdef DEBUG_OUTPUT
    cout << "Calculate the straightness of the car path" << endl;
#endif
    pcl::MomentOfInertiaEstimation<MyPoint> moment;
    MyPointCloud::Ptr pc_ptr(new MyPointCloud());
    for(auto p : v_trace_)
        pc_ptr->push_back(p);
    
    moment.setInputCloud(pc_ptr);
    moment.compute();
    Eigen::Vector3f vx, vy, vz;         // eigen vector xyz
    float l1, l2, l3;                   // lambda 1-3
    moment.getEigenVectors(vx, vy, vz);
    moment.getEigenValues(l1, l2, l3);
    straightness_ = l1 / (l1 + l2 + l3);

#ifdef DEBUG_OUTPUT
    cout << "Eigen values: " << l1 << ", " << l2 << ", " << l3 << ". " << endl;
    cout << "Straightness: " << straightness_ << " (1.0 means good). " << endl;
#endif
}



void TraceCoeff::calcCarMoveDistance(void){
    total_distance_ = 0.0;
    for(int i=1; i<v_trace_.size(); ++i){
        total_distance_ += tool::calDistance(v_trace_[i], v_trace_[i-1]);
    }
    cout << "total distance L: " << total_distance_ << endl;
}