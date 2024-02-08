/**
 * @file movement.h
 * @author Soldera Marco (marco.soldera@studenti.unitn.it) - Group Soldera Marco and Morandin Marco
 * @version 0.1
 * @date 2024-02-05
 * 
 * @copyright Copyright (c) 2024
 */


#ifndef __MOVEMENT_H__
#define __MOVEMENT_H__


#include "kinematics.h"

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iostream>


using namespace std;
using namespace Eigen;


/// @brief Max time for the trajectory
inline float maxT = 6;


/**
 * Function prototypes
*/

/**
 * @brief Redefinition of multiplication between float and Quaternionf
 * 
 * @param num Scalar that multiplies
 * @param q Quaternion to be multiplied
 * @return Quaternionf
*/
inline Quaternionf operator *(float num, const Quaternionf& q) {
    return Quaternionf(q.x() * num, q.y() * num, q.z() * num, q.w() * num);
}

Vector3f pd(float t, Vector3f xef, Vector3f xe0);
Quaternionf qd(float tb, Quaternionf q0, Quaternionf qf);
VectorXf invDiffKinematicControlCompleteQuaternion(VectorXf q, Vector3f xe, Vector3f xd, Vector3f vd, Vector3f omegad, Quaternionf qe, Quaternionf qd,  Matrix3f Kp, Matrix3f Kq, int f);
void invDiffKinematicControlSimCompleteQuaternion(Vector3f xef, Vector3f phief, double dt, VectorXf jstates, void (*send_j)(VectorXf));


#endif
