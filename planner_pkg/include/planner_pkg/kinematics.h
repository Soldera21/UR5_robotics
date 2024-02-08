/**
 * @file kinematics.h
 * @author Soldera Marco (marco.soldera@studenti.unitn.it) - Group Soldera Marco and Morandin Marco
 * @version 0.1
 * @date 2024-02-05
 * 
 * @copyright Copyright (c) 2024
 */


#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__


#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>


using namespace std;
using namespace Eigen;


/**
 * @brief Structure to store the position and rotation of the end effector
 */
struct frame {
    Vector3f xyz;
    Matrix3f rot;
};


/**
 * Functions prototypes
*/

Matrix4f t10f(float th1);
Matrix4f t21f(float th2);
Matrix4f t32f(float th3);
Matrix4f t43f(float th4);
Matrix4f t54f(float th5);
Matrix4f t65f(float th6);

frame direct_kinematics(VectorXf th);
MatrixXf inverse_kinematics(frame &frame);
MatrixXf jacobian(VectorXf q);

Matrix3f eul2rotm(Vector3f rpy);
Vector3f rotm2eul(Matrix3f R);


#endif
