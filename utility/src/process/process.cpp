
#include "process/process.h"
#include "process/scene_cloud.h"
#include "process/car_path.h"
#include "process/tool.h"



Eigen::Matrix4d calcGlobalT(const CarPath& cp){
    Eigen::Vector3d pb = tool::xyz2vector(cp.getBeginPoint());  // path_begin
    Eigen::Vector3d pe = tool::xyz2vector(cp.getEndPoint());
    Eigen::Vector3d dir = pe - pb;
    dir.normalize();

    cout << "--> car from: (" << pb(0) << ", " << pb(1) << ", " << pb(2)
        << "), to: (" << pe(0) << ", " << pe(1) << ", " << pe(2) << ")" << endl;

    Eigen::Vector3d xn, yn, zn;
    zn << 0, 0, 1;
    yn = dir - dir.dot(zn) * zn;      // project x to z;
    yn = -yn;                         // rotate to original direction; moving direction is always (-y).

    yn.normalize();
    xn = yn.cross(zn);
    xn.normalize();

    Eigen::Matrix3d R;      // R: from new coordinate to the old; R*vn = v
    R.col(0) = xn;
    R.col(1) = yn;
    R.col(2) = zn;
    
    // explain the rotation:
    // v: points vector in old coordinate; vn: points vector in new coordiante;
    // x, y, z: new coordiante's axis representated by old coordiante axis;
    // [x, y, z]*v = [xn, yn, zn]*vn + t, where t = pb;  --> vn = -R^(-1)*v - R^(-1)*pb
    Eigen::Matrix4d globalT = Eigen::Matrix4d::Identity();
    globalT.topLeftCorner<3, 3>(0, 0) = R.transpose();
    globalT.col(3).head(3) = -R.transpose() * pb;

    cout << "Final transform (from old points to new points): \n" << globalT << endl;
    return globalT;
}


void updateCoordinate(CarPath& cp, SceneCloud& sc, const Eigen::Matrix4d& T){
    pcl::PointCloud<pcl::PointXYZ> new_scene_pc;
    pcl::transformPointCloud(*(sc.pc_), new_scene_pc, T.cast<float>());
    *(sc.pc_) = new_scene_pc;
    
    pcl::PointCloud<pcl::PointXYZ> new_path_pc;
    pcl::transformPointCloud(*(cp.pc_), new_path_pc, T.cast<float>());
    *(cp.pc_) = new_path_pc;
}
