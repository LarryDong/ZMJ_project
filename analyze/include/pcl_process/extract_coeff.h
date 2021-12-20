
#ifndef EXTRACT_COEFF_H_
#define EXTRACT_COEFF_H_

#include <iostream>
#include <eigen3/Eigen/Core>

using namespace std;

typedef Eigen::Vector3d V3d;
typedef Eigen::Matrix4d M4d;

class SupportCoeff{

public: 
    SupportCoeff(const V3d& lc_center, const V3d& lc_dir,
                const V3d& rc_center, const V3d& rc_dir,
                const M4d& base_transform, 
                const V3d& roof_center, const V3d& roof_normal)
                :
        lc_center_(lc_center),
        lc_dir_(lc_dir),
        rc_center_(rc_center),
        rc_dir_(rc_dir),
        roof_center_(roof_center),
        roof_normal_(roof_normal),
        base_transform_(base_transform){}
        

    void printCoeff(void){
        cout << "Cylinder: ----------------------------------------------------" << endl;
        cout << "Left. Center: " << lc_center_.transpose() << ", dir: " << lc_dir_.transpose() << endl;
        cout << "Righ. Center: " << rc_center_.transpose() << ", dir: " << rc_dir_.transpose() << endl;

        cout << "Base: ----------------------------------------------------" << endl;
        cout << base_transform_ << endl;

        cout << "Roof: ----------------------------------------------------" << endl;
        cout << "center: " << roof_center_.transpose() << ", normal: " << roof_normal_.transpose() << endl;
    }

    void calcResult(void);


private:
    
    V3d lc_center_, lc_dir_;        // left-cylinder center/direction
    V3d rc_center_, rc_dir_;
    V3d roof_center_, roof_normal_;
    M4d base_transform_;

};


#endif 
