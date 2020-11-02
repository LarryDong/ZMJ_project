
#ifndef DEFINATION_H_
#define DEFINATION_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <sys/time.h>
#include <iostream>

class TicToc{
public:
    TicToc() {};
    void begin(){
        gettimeofday(&tBegin, NULL);
    }
    double end(){
        timeval tEnd;
        gettimeofday(&tEnd, NULL);
        double t = 1000 * (tEnd.tv_sec - tBegin.tv_sec) + (tEnd.tv_usec - tBegin.tv_usec) / 1000.0f;
        return t;
    }
private:
    timeval tBegin;
};

#endif
