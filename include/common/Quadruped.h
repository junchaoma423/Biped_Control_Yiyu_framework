/*!
 * @file Aliengo.h
 * @brief stores dynamics information
 * Leg 0: Left front; Leg 1: right front;
 * Leg 2: Left rear ; Leg 3: right rear;
 */ 
#ifndef PROJECT_QUADRUPED_H
#define PROJECT_QUADRUPED_H

#include <vector>
#include "cppTypes.h"
#include <string>
class Quadruped{
  public:
    void setQuadruped(int robot_id){
        robot_index = robot_id;
        if(robot_id == 1){ // Aliengo
            mass = 19;

            leg_offset_x = 0.2399;
            leg_offset_y = 0.051;
            leg_offset_z = 0.0;

            hipLinkLength = 0.0838;
            thighLinkLength = 0.25;
            calfLinkLength = 0.25;
        }
        if(robot_id == 2){ // A1
            mass = 12;

            leg_offset_x = 0.1805;
            leg_offset_y = 0.047;
            leg_offset_z = 0.0;

            hipLinkLength = 0.0838; // hip offset in const.xacro
            thighLinkLength = 0.2;
            calfLinkLength = 0.2;
        }
        if(robot_id == 3){ // B1
            mass = 55;

            leg_offset_x = 0.3455;
            leg_offset_y = 0.072;
            leg_offset_z = 0;

            hipLinkLength = 0.12675;
            thighLinkLength = 0.35;
            calfLinkLength = 0.35;
        }
        if(robot_id == 4){ // Biped
            mass = 13.9;

            leg_offset_x = 0.0;
            leg_offset_y = -0.047;
            leg_offset_z = -0.1265;

            leg_offset_x2 = 0.0;
            leg_offset_y2 = -0.047;
            leg_offset_z2 = -0.1365;

            hipLinkLength = 0.06;
            thighLinkLength = 0.22;
            calfLinkLength = 0.22;
        }
    }
    int robot_index; // 1 for Aliengo, 2 for A1
    double hipLinkLength;
    double thighLinkLength;
    double calfLinkLength;
    double leg_offset_x;
    double leg_offset_y;
    double leg_offset_z;
    double leg_offset_x2;
    double leg_offset_y2;
    double leg_offset_z2;
    double mass;

    // Modified it to use in biped
    Vec3<double> getHipLocation(int leg){
        assert(leg >=0 && leg <2);
        Vec3<double> pHip = Vec3<double>::Zero();
        if (leg == 0){
            pHip(0) = leg_offset_x;
            pHip(1) = leg_offset_y;
            pHip(2) = leg_offset_z;
        }
        if (leg == 1){
            pHip(0) = leg_offset_x;
            pHip(1) = -leg_offset_y;
            pHip(2) = leg_offset_z;
        }

        return pHip;
    };

    Vec3<double> getHip2Location(int leg){
        assert(leg >= 0 && leg <2);
        Vec3<double> pHip2 = Vec3<double>::Zero();
        if (leg == 0){
            pHip2(0) = leg_offset_x2;
            pHip2(1) = leg_offset_y2;
            pHip2(2) = leg_offset_z2;
        }
        return pHip2;
    }

};

#endif