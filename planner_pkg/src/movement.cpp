/**
 * @file movement.cpp
 * @author Soldera Marco (marco.soldera@studenti.unitn.it) - Group Soldera Marco and Morandin Marco
 * @brief Functions in this file are used to calculate the trajectory based on quaternions and velocities of the joints
 * @version 0.1
 * @date 2024-02-05
 * 
 * @copyright Copyright (c) 2024
 */


#include "planner_pkg/movement.h"


/**
 * @brief Calculates trajectory for the end-effector position
 *
 * @param t The current time
 * @param xef The desired end-effector position
 * @param xe0 The start end-effector position
 * @return Vector3f
 */
Vector3f pd(float tb, Vector3f xef, Vector3f xe0) {
    float t = tb / maxT;
    if (t > 1) {
        return xef;
    }
    else {
        return t * xef + (1 - t) * xe0;
    }
}

/**
 * @brief Calculates trajectory for the end-effector orientation with quaternions
 *
 * @param tb The current time
 * @param q0 The start end-effector quaternion
 * @param qf The desired end-effector quaternion
 * @return Quaternionf
 */
Quaternionf qd(float tb, Quaternionf q0, Quaternionf qf) {
    float t = tb / maxT;
    if(t > 1) {
        return qf;
    }
    else {
        return q0.slerp(t, qf);
    }
}

/**
 * @brief Calculates joint velocities using the jacobian matrix
 *
 * @param q The current joint config
 * @param xe The current end-effector position
 * @param xd The desired end-effector position
 * @param vd The desired end-effector linear velocity
 * @param omegad The desired end-effector angular velocity
 * @param qe The current end-effector rotation in quaternion
 * @param qd The desired end-effector rotation in quaternion
 * @param Kd The position gain
 * @param Kq The orientation gain
 * @param f counter for debugging
 * @return Vector6f
 */
VectorXf invDiffKinematicControlCompleteQuaternion(VectorXf q, Vector3f xe, Vector3f xd, Vector3f vd, Vector3f omegad, Quaternionf qe, Quaternionf qd,  Matrix3f Kp, Matrix3f Kq, int f) {
    MatrixXf J;
    VectorXf qdot;
    VectorXf ve(6);

    J = jacobian(q);

    Quaternionf qp = qd * qe.conjugate();
    Vector3f eo(qp.x(), qp.y(), qp.z());

    ve << (vd + Kp * (xd - xe)), (omegad + Kq * eo);
    qdot = J.inverse() * ve;

    // Uncomment here for debugging params
    /*if(f == 10) {
        cout << "J: " << J << endl;
        cout << "qe: " << qe.w() << " " << qe.x() << "i " << qe.y() << "j " << qe.z() << "k" << endl;
        cout << "qd: " << qd.w() << " " << qd.x() << "i " << qd.y() << "j " << qd.z() << "k" << endl;
        cout << "qp: " << qp.w() << " " << qp.x() << "i " << qp.y() << "j " << qp.z() << "k" << endl;
        cout << "eo: " << eo << endl;
    }*/

    return qdot;
}

/**
 * @brief Calculates joint configs using quaternions
 *
 * @param xef Desired end-effector position
 * @param phief Desired end-effector orientation
 * @param dt Time step
 * @param jstates Actual state of the joints
 * @param send_j Function to send joint states
 * @return void
 */
void invDiffKinematicControlSimCompleteQuaternion(Vector3f xef, Vector3f phief, double dt, VectorXf jstates, void (*send_j)(VectorXf)) {
    frame now;   // current frame
    frame start; // start frame
    start = direct_kinematics(jstates);

    cout << "Going from: " << start.xyz[0] << " " << start.xyz[1] << " " << start.xyz[2] << endl;
    cout << " Rot:       " << rotm2eul(start.rot)[0] << " " << rotm2eul(start.rot)[1] << " " << rotm2eul(start.rot)[2] << endl;
    cout << "To:         " << xef[0] << " " << xef[1] << " " << xef[2] << endl;
    cout << " Rot:       " << phief[0] << " " << phief[1] << " " << phief[2] << endl;
    cout << "Joints:     " << jstates[0] << " " << jstates[1] << " " << jstates[2] << " " << jstates[3] << " " << jstates[4] << " " << jstates[5] << endl << endl;
    
    VectorXf qk = jstates;          // set the initial joint config
    VectorXf qk1(6);                // joint config

    Matrix3f kp; // position gain
    kp = Matrix3f::Identity() * 10;

    Matrix3f kq; // orientation gain
    kq = Matrix3f::Identity() * -10;

    VectorXf dotqk(6); // joint velocities coefficients

    Vector3f vd; // desired linear velocity
    Vector3f omegad; // desired angular velocity
    
    Quaternionf q0(start.rot);
    q0 = q0.conjugate();
    Quaternionf qf(eul2rotm(phief));
    qf = qf.conjugate();

    // Uncomment here for debugging params
    /*cout << "q0: " << q0.w() << " " << q0.x() << "i " << q0.y() << "j " << q0.z() << "k" << endl;
    cout << "qf: " << qf.w() << " " << qf.x() << "i " << qf.y() << "j " << qf.z() << "k" << endl;*/

    ros::Rate loop_rate(1000.0);
    int f = 1;
    for(float i = dt; i < maxT; i += dt) {
        now = direct_kinematics(qk);

        Quaternionf qe(now.rot);
        qe = qe.conjugate();
        vd = (pd(i, xef, start.xyz) - pd(i - dt, xef, start.xyz)) / dt; // desired linear velocity

        Quaternionf work = (2/dt) * (qd(i + dt, q0, qf) * qd(i, q0, qf).conjugate());
        omegad << work.w(), work.x(), work.y();

        // coefficient
        dotqk = invDiffKinematicControlCompleteQuaternion(qk, now.xyz, pd(i, xef, start.xyz), vd, omegad, qe, qd(i, q0, qf), kp, kq, f);

        // euler integration
        qk1 = qk + dotqk * dt;

        // Uncomment here for debugging params
        /*if(f == 10) {
            cout << "dotqk: " << dotqk[0] << " " << dotqk[1] << " " << dotqk[2] << " " << dotqk[3] << " " << dotqk[4] << " " << dotqk[5] << endl;
            cout << "qk: " << qk << endl;
            cout << "qk1: " << qk1 << endl;
        }*/

        if (qk1(5) >= M_PI) {
            qk1(5) = qk1(5) - M_PI*abs((int)(qk1(5)/M_PI));
        }
        else if (qk1(5) <= -M_PI) {
            qk1(5) = qk1(5) + M_PI*abs((int)(qk1(5)/M_PI));
        }

        qk = qk1;
        send_j(qk);

        f += 1;

        //loop_rate.sleep();
    }
}
