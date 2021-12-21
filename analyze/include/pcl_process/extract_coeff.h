
#ifndef EXTRACT_COEFF_H_
#define EXTRACT_COEFF_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <eigen3/Eigen/Core>

#include "defination.h"
#include "tool.h"

using namespace std;

typedef Eigen::Vector3d V3d;
typedef Eigen::Matrix4d M4d;


// 工作面直线度、小车运行总距离L、轨迹采样点数m、轨迹采样间距、轨迹坐标。
class TraceCoeff{
public:
    TraceCoeff(string filename){
        ifstream in(filename);
        if(!in.is_open()){
            cout << "Error. Cannot load Trace from: " << filename << endl;
            std::abort();
        }
        double x, y, z;
        while(!in.eof()){
            if(in.fail())
                break;
            in >> x >> y >> z;
            v_trace_.push_back(MyPoint(x, y, z));
        }
        calcStraightness();
        calcDistance();
    }
    void printResult(void){
        cout << "---------------------------------------" << endl;
        cout << "Straightness: " << straightness_ << endl;
        cout << "Total dist L: " << total_distance_ << endl;
        cout << "---------------------------------------" << endl;
    }

public:
    void calcStraightness(void);
    void calcDistance(void);
    
    vector<MyPoint> v_trace_;
    double straightness_;
    double total_distance_;
};



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


    void setRefPosition(double distance) { ref_pos_ = distance; }

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
    double ref_pos_;                // reference pose by carPath
    V3d lc_center_, lc_dir_;        // left-cylinder center/direction
    V3d rc_center_, rc_dir_;
    V3d roof_center_, roof_normal_;
    M4d base_transform_;

};


void outputResult(const TraceCoeff& trace, const SupportCoeff& support, string filename);

#endif 
