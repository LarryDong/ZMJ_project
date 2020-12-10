
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;

int main(int argc, char **argv){

    cout << "Test" << endl;

    Eigen::Quaterniond q(0, 0, 0, 1);
    Eigen::Matrix4d T;
    T.topLeftCorner(3,3) = q.toRotationMatrix();
    
    Eigen::Vector4d t;
    t << 1, 2, 3, 1;
    T.topRightCorner(4,1) = Eigen::Vector4d(1,2,3,4);
    cout << T << endl;
    return 0;
}