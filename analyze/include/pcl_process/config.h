
#pragma once

#ifndef CONFIG_H
#define CONFIG_H

#include "defination.h"


class ClusterParameter{
public:
    ClusterParameter(double angle, double dist, int mi, int ma, double radius) : 
        delta_angle(angle), delta_distance(dist), min_num(mi), max_num(ma), search_radius(radius) {}

    inline void reset(double angle, double dist, int mi, int ma, double radius){
        delta_angle = angle;
        delta_distance =dist;
        search_radius = radius;
        min_num = mi;
        max_num = ma;
    }

public:
    double search_radius;
    double delta_angle, delta_distance;
    int min_num, max_num;
};


class PlaneParameter{
public:
    PlaneParameter(double r1r3, double r1r2) : 
        l1l3(r1r3), l1l2(r1r2) {}
    inline void reset(double r1r3, double r1r2){
        l1l3 = r1r3;
        l1l2 = r1r2;
    }
    double l1l2, l1l3;
};


class SupportParameter{
public:
    SupportParameter(){};
    inline void setRoof(double xmin, double xmax, double zmin, double zmax, double angle){
        roof_norm_angle = angle;
        roof_x_min = xmin;
        roof_x_max = xmax;
        roof_z_min = zmin;
        roof_z_max = zmax;
    }
    inline void setSegment(double xmin, double xmax, double y_range, double zmin, double zmax){
        segment_x_min = xmin;
        segment_x_max = xmax;
        segment_y = y_range;
        segment_z_min =zmin;
        segment_z_max=zmax;
    }

public: 
    // roof parameters
    double roof_norm_angle;
    double roof_x_min, roof_x_max;
    double roof_z_min, roof_z_max;

    // segment parameters
    double segment_y;
    double segment_z_min, segment_z_max;
    double segment_x_min, segment_x_max;
};


#endif
