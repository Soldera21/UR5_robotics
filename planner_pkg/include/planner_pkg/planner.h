/**
 * @file planner.h
 * @author Soldera Marco (marco.soldera@studenti.unitn.it) - Group Soldera Marco and Morandin Marco
 * @version 0.1
 * @date 2024-02-05
 * 
 * @copyright Copyright (c) 2024
 */


#ifndef __PLANNER_H__
#define __PLANNER_H__


#include "movement.h"
#include <planner_pkg/legoDetection.h>
#include <planner_pkg/legoGroup.h>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iostream>
#include <vector>
#include <cmath>


using namespace std;
using namespace Eigen;


/**
 * @brief Position of the components of the gripper
*/
typedef Matrix<float, 2, 1> GripperState;

/// @brief Loop rate of the node
float loop_frequency = 1000.0;
/// @brief Time step
double timeStep = 0.001;
/// @brief Position of the gripper
GripperState actual_gripper;


/// @brief Publisher for the desired joint state
ros::Publisher pub_joint_states;
/// @brief Structure of the message to be published with joint positions
std_msgs::Float64MultiArray jointState_msg_robot;


/**
 * Functions prototype
*/

void listen_lego_detection(ros::Rate rate);
void move_to_home();
Vector3f quat2eul(Quaternionf q);
VectorXf get_joint_states();
void set_joint_states(VectorXf q);
GripperState get_gripper_states();
void open_gripper(float amp);
void close_gripper(float amp);
void waitSec(float t);
void waitJoints(bool waitRot, Vector3f xef, Vector3f phief);


/**
 * Final positions of bricks based on type
*/

Vector3f X1_Y1_Z2(0.92, 0.27, 0.88);
Vector3f X1_Y2_Z1(0.77, 0.27, 0.88);
Vector3f X1_Y2_Z2(0.62, 0.27, 0.88);

Vector3f X1_Y2_Z2_CHAMFER(0.92, 0.42, 0.88);
Vector3f X1_Y2_Z2_TWINFILLET(0.77, 0.42, 0.88);
Vector3f X2_Y2_Z2(0.62, 0.42, 0.89);

Vector3f X2_Y2_Z2_FILLET(0.92, 0.56, 0.89);
Vector3f X1_Y3_Z2(0.77, 0.56, 0.88);
Vector3f X1_Y3_Z2_FILLET(0.62, 0.56, 0.88);

Vector3f X1_Y4_Z1(0.87, 0.72, 0.88);
Vector3f X1_Y4_Z2(0.65, 0.72, 0.88);

map<std::string, Vector3f> models_map{
    {"X1-Y1-Z2", X1_Y1_Z2},
    {"X1-Y2-Z1", X1_Y2_Z1},
    {"X1-Y2-Z2", X1_Y2_Z2},
    {"X1-Y2-Z2-CHAMFER", X1_Y2_Z2_CHAMFER},
    {"X1-Y2-Z2-TWINFILLET", X1_Y2_Z2_TWINFILLET},
    {"X2-Y2-Z2", X2_Y2_Z2},
    {"X2-Y2-Z2-FILLET",X2_Y2_Z2_FILLET},
    {"X1-Y3-Z2", X1_Y3_Z2},
    {"X1-Y3-Z2-FILLET",X1_Y3_Z2_FILLET},
    {"X1-Y4-Z1", X1_Y4_Z1},
    {"X1-Y4-Z2", X1_Y4_Z2}
};


#endif
