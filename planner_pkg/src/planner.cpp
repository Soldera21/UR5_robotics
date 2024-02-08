/**
 * @file planner.cpp
 * @author Soldera Marco (marco.soldera@studenti.unitn.it) - Group Soldera Marco and Morandin Marco
 * @brief Main function and planning of the movement based on the messages received from the vision;
 *        also getters and setters for joints are present
 * @version 0.1
 * @date 2024-02-05
 * 
 * @copyright Copyright (c) 2024
 */


#include "planner_pkg/planner.h"


int main(int argc, char** argv) {
    cout << "----------------------------------------" << endl;
    cout << "| STARTING PLANNER AND MOVEMENT MODULE |" << endl;
    cout << "----------------------------------------" << endl << endl;

    ros::init(argc, argv, "custom_joint_publisher_main");
    ros::NodeHandle node;
    pub_joint_states = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
    ros::Rate loop_rate(loop_frequency);

    jointState_msg_robot.data.resize(8);

    cout << "STARTED MODULE!" << endl << endl << endl;
    
    while (ros::ok()) {
        listen_lego_detection(loop_rate);
        loop_rate.sleep();
    }

    return 0;
}


/**
 * @brief Listen to the /lego_position topic if messages arrives from the vision node;
 *        for each lego detected it sends the robot to the lego position knowing which type of lego it is
 *
 * @param rate ros rate
 * @return void
 */
void listen_lego_detection(ros::Rate rate) {
    ros::NodeHandle nodeH;
    planner_pkg::legoGroup::ConstPtr msg = ros::topic::waitForMessage<planner_pkg::legoGroup>("/lego_position", nodeH);

    if (msg != 0) {
        vector<planner_pkg::legoDetection> vector = msg->lego_vector;

        cout << "Beginning Task..." << endl << endl << endl;

        VectorXf joints(6);
        for (planner_pkg::legoDetection lego : vector) {
            cout << "PICKING: " << lego.model << endl;
            
            Vector3f pos;
            pos << lego.pose.position.x - 0.5, -(lego.pose.position.y - 0.35), -(lego.pose.position.z - 1.75);
            
            Quaternionf q;
            q.w() = lego.pose.orientation.w;
            q.x() = lego.pose.orientation.x;
            q.y() = lego.pose.orientation.y;
            q.z() = lego.pose.orientation.z;

            Vector3f rot = quat2eul(q);
            if(rot[2] >= M_PI) {
                rot[2] = rot[2] - M_PI*abs((int)(rot[2]/M_PI));
            }
            else if(rot[2] <= -M_PI) {
                rot[2] = rot[2] + M_PI*abs((int)(rot[2]/M_PI));
            }
            rot[2] = -rot[2];

            if(rot[0] == 0 && rot[1] == 0) {
                // blocchetto dritto
                cout << " STRAIGHT LEGO" << endl;

                float grip_amp_o;
                float grip_amp_c;
                if((std::string)lego.model == (std::string)"X2-Y2-Z2" || (std::string)lego.model == (std::string)"X2-Y2-Z2-FILLET") {
                    pos[2] = 0.87;
                    grip_amp_c = 0.1;
                    grip_amp_o = 1.2;
                }
                else {
                    pos[2] = 0.885;
                    grip_amp_c = 0.25;
                    grip_amp_o = 0.8;
                }

                cout << " PICK POS: " << pos[0] << " " << pos[1] << " " << pos[2] << endl;
                cout << " PICK ROT: " << rot[0] << " " << rot[1] << " " << rot[2] << endl << endl;

                Vector3f temp_pos;
                Vector3f temp_rot;

                move_to_home();

                // pick
                joints = get_joint_states();
                temp_pos << pos[0], -0.40, 0.73;
                temp_rot << 0.00, 0.00, 0.00;
                invDiffKinematicControlSimCompleteQuaternion(temp_pos, temp_rot, timeStep, joints, &set_joint_states);
                waitJoints(false, temp_pos, temp_rot);

                joints = get_joint_states();
                if((pos[0] < 0.00 && pos[0] > -0.25) && pos[1] > -0.10) {
                    temp_pos << -0.45, 0.00, 0.73;
                    invDiffKinematicControlSimCompleteQuaternion(temp_pos, temp_rot, timeStep, joints, &set_joint_states);
                    waitJoints(false, temp_pos, temp_rot);

                    joints = get_joint_states();
                }

                temp_pos << pos[0], pos[1], 0.73;
                invDiffKinematicControlSimCompleteQuaternion(temp_pos, temp_rot, timeStep, joints, &set_joint_states);
                waitJoints(false, temp_pos, temp_rot);
                
                open_gripper(grip_amp_o);
                waitSec(2.5);

                joints = get_joint_states();
                invDiffKinematicControlSimCompleteQuaternion(pos, rot, timeStep, joints, &set_joint_states);
                waitJoints(true, pos, rot);

                close_gripper(grip_amp_c);
                waitSec(2.5);

                joints = get_joint_states();
                temp_pos << pos[0], pos[1], 0.73;
                invDiffKinematicControlSimCompleteQuaternion(temp_pos, temp_rot, timeStep, joints, &set_joint_states);
                waitJoints(false, temp_pos, temp_rot);

                // path
                joints = get_joint_states();
                if((pos[0] < 0.00 && pos[0] > -0.25) && pos[1] > -0.10) {
                    temp_pos << -0.45, 0.00, 0.73;
                    invDiffKinematicControlSimCompleteQuaternion(temp_pos, temp_rot, timeStep, joints, &set_joint_states);
                    waitJoints(false, temp_pos, temp_rot);

                    joints = get_joint_states();
                }

                if(pos[1] > -0.40){
                    temp_pos << pos[0], -0.40, 0.73;
                    invDiffKinematicControlSimCompleteQuaternion(temp_pos, temp_rot, timeStep, joints, &set_joint_states);
                    waitJoints(false, temp_pos, temp_rot);
                }

                pos = models_map[lego.model];
                pos << pos[0] - 0.5, -(pos[1] - 0.35), -(pos[2] - 1.75);
                rot << 0.00, 0.00, 0.00;
                
                joints = get_joint_states();
                temp_pos << pos[0], -0.40, 0.73;
                invDiffKinematicControlSimCompleteQuaternion(temp_pos, temp_rot, timeStep, joints, &set_joint_states);
                waitJoints(false, temp_pos, temp_rot);
                
                joints = get_joint_states();
                if((pos[0] > 0.00 && pos[0] < 0.25) && pos[1] > -0.10) {
                    temp_pos << 0.45, 0.00, 0.73;
                    invDiffKinematicControlSimCompleteQuaternion(temp_pos, temp_rot, timeStep, joints, &set_joint_states);
                    waitJoints(false, temp_pos, temp_rot);

                    joints = get_joint_states();
                }
                
                // place
                temp_pos << pos[0], pos[1], 0.73;
                invDiffKinematicControlSimCompleteQuaternion(temp_pos, temp_rot, timeStep, joints, &set_joint_states);
                waitJoints(false, temp_pos, temp_rot);
                
                joints = get_joint_states();
                invDiffKinematicControlSimCompleteQuaternion(pos, rot, timeStep, joints, &set_joint_states);
                waitJoints(false, pos, rot);
                
                open_gripper(grip_amp_o);
                waitSec(2.5);
                
                joints = get_joint_states();
                temp_pos << pos[0], pos[1], 0.73;
                invDiffKinematicControlSimCompleteQuaternion(temp_pos, temp_rot, timeStep, joints, &set_joint_states);
                waitJoints(false, temp_pos, temp_rot);
                
                close_gripper(grip_amp_c);
                waitSec(2.5);
                
                move_to_home();

                cout << endl;
            }
            else if (rot[0] != 0 || rot[1] != 0) {
                cout << "LEGO NOT MOVABLE; INCORRECT ROTATION!" << endl;
            }
        }
        cout << "ENDED TASK, waiting for another from vision!" << endl << endl;
    }
}


/**
 * @brief Moves the robot to the home position
 * 
 * @return void
*/
void move_to_home() {
    Vector3f temp_pos;
    Vector3f temp_rot;

    VectorXf joints = get_joint_states();
    frame actPose = direct_kinematics(joints);
    if(abs(actPose.xyz[0] - 0.00) < 0.03 && abs(actPose.xyz[1] - (-0.40)) < 0.03 && abs(actPose.xyz[2] - 0.73) < 0.03){
        return;
    }

    temp_rot << 0.00, 0.00, 0.00;
    if((actPose.xyz[0] >= 0.00 && actPose.xyz[0] < 0.25) && actPose.xyz[1] > -0.10) {
        temp_pos << 0.45, 0.00, 0.73;
        invDiffKinematicControlSimCompleteQuaternion(temp_pos, temp_rot, timeStep, joints, &set_joint_states);
        waitJoints(true, temp_pos, temp_rot);

        joints = get_joint_states();
    }
    else if((actPose.xyz[0] < 0.00 && actPose.xyz[0] > -0.25) && actPose.xyz[1] > -0.10) {
        temp_pos << -0.45, 0.00, 0.73;
        invDiffKinematicControlSimCompleteQuaternion(temp_pos, temp_rot, timeStep, joints, &set_joint_states);
        waitJoints(true, temp_pos, temp_rot);

        joints = get_joint_states();
    }

    actPose = direct_kinematics(joints);
    if(actPose.xyz[0] >= 0.00) {
        temp_pos << 0.40, -0.40, 0.73;
        invDiffKinematicControlSimCompleteQuaternion(temp_pos, temp_rot, timeStep, joints, &set_joint_states);
        waitJoints(true, temp_pos, temp_rot);

        joints = get_joint_states();
    }
    else if(actPose.xyz[0] < 0.00) {
        temp_pos << -0.40, -0.40, 0.73;
        invDiffKinematicControlSimCompleteQuaternion(temp_pos, temp_rot, timeStep, joints, &set_joint_states);
        waitJoints(true, temp_pos, temp_rot);

        joints = get_joint_states();
    }

    joints = get_joint_states();
    temp_pos << 0.00, -0.40, 0.73;
    invDiffKinematicControlSimCompleteQuaternion(temp_pos, temp_rot, timeStep, joints, &set_joint_states);
    waitJoints(true, temp_pos, temp_rot);
}

/**
 * @brief Convert from Quaternion to Euler Angles
 *
 * @param q Quaternion to convert
 * @return Vector3f
 */
Vector3f quat2eul(Quaternionf q) {
    Vector3f angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    double roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = sqrt(1 + 2 * (q.w() * q.y() - q.x() * q.z()));
    double cosp = sqrt(1 - 2 * (q.w() * q.y() - q.x() * q.z()));
    double pitch = 2 * atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    double yaw = atan2(siny_cosp, cosy_cosp);

    angles << roll, pitch, yaw;
    return angles;
}

/**
 * @brief Read from the topic the actual value of the joint
 *
 * @return Vector6f
 */
VectorXf get_joint_states() {
    ros::NodeHandle nodeH;
    ros::Duration Timeout = ros::Duration(3);

    VectorXf actual_pos(6);

    boost::shared_ptr<const sensor_msgs::JointState_<std::allocator<void>>> msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states", nodeH, Timeout);

    actual_pos[0] = msg->position[4];
    actual_pos[1] = msg->position[3];
    actual_pos[2] = msg->position[0];
    actual_pos[3] = msg->position[5];
    actual_pos[4] = msg->position[6];
    actual_pos[5] = msg->position[7];

    return actual_pos;
}

/**
 * @brief Posts on the topic the vector joint_pos, which contains the value of the angles, that all joints must reach
 *
 * @param joint_pos Vector that conatins the values of each joint angle position to be published
 * @return void
 */
void set_joint_states(VectorXf joint_pos) {
    for (int i = 0; i < 6; i++) {
        jointState_msg_robot.data[i] = joint_pos[i];
    }
    for (int i = 6; i < 8; i++) {
        jointState_msg_robot.data[i] = actual_gripper[i - 6];
    }

    pub_joint_states.publish(jointState_msg_robot);
}

/**
 * @brief Read from the topic the actual value of the gripper joint
 *
 * @return GripperState
 */
GripperState get_gripper_states() {
    ros::NodeHandle nodeH;
    ros::Duration Timeout = ros::Duration(3);

    boost::shared_ptr<const sensor_msgs::JointState_<std::allocator<void>>> msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states", nodeH, Timeout);
    actual_gripper[0] = msg->position[1];
    actual_gripper[1] = msg->position[2];

    return actual_gripper;
}

/**
 * @brief Open the gripper of the robot. We public directly on the topic
 *        the angles that we want to reach with the fingers of the gripper
 *
 * @param amp Required gripper length
 * @return void
 */
void open_gripper(float amp) {
    VectorXf msg(8);
    VectorXf actual_pos = get_joint_states();
    ros::Rate loop_rate(loop_frequency);

    actual_gripper = get_gripper_states();
    while(actual_gripper(0) < amp) {
        for(int i = 0; i < 6; i++) {
            msg(i) = actual_pos(i);
        }
        for(int i = 6; i < 8; i++) {
            msg(i) = actual_gripper(i - 6);
        }
        set_joint_states(msg);
        actual_gripper(0) = actual_gripper(0) + 0.1;
        actual_gripper(1) = actual_gripper(1) + 0.1;

        loop_rate.sleep();
    }
}

/**
 * @brief Close the gripper of the robot. We public directly on the topic
 *        the angles that we want to reach with the fingers of the gripper
 *
 * @param amp Required negative gripper length (positive value)
 * @return void
 */
void close_gripper(float amp) {
    VectorXf msg(8);
    VectorXf actual_pos = get_joint_states();
    ros::Rate loop_rate(loop_frequency);

    actual_gripper = get_gripper_states();
    while(actual_gripper(0) > -amp) {
        for(int i = 0; i < 6; i++) {
            msg(i) = actual_pos(i);
        }
        for(int i = 6; i < 8; i++) {
            msg(i) = actual_gripper(i - 6);
        }
        set_joint_states(msg);
        actual_gripper(0) = actual_gripper(0) - 0.1;
        actual_gripper(1) = actual_gripper(1) - 0.1;

        loop_rate.sleep();
    }
}

/**
 * @brief Wait for the specified time
 * 
 * @param t Time to wait
 * @return void
*/
void waitSec(float t) {
    ros::Rate loop_rate(loop_frequency);

    for(int i = 0; i < loop_frequency*t/10; i++) {
        loop_rate.sleep();
    }
}

/**
 * @brief Wait for joints to be at the final position
 * 
 * @param waitRot wait also for z rotation to be aligned
 * @param xef final positon
 * @param phief final rotation
 * @return void
*/
void waitJoints(bool waitRot, Vector3f xef, Vector3f phief) {
    ros::Rate loop_rate(1000.0);

    frame actPose = direct_kinematics(get_joint_states());
    while(abs(actPose.xyz[0] - xef[0]) > 0.03 || abs(actPose.xyz[1] - xef[1]) > 0.03 || abs(actPose.xyz[2] - xef[2]) > 0.03) {
        actPose = direct_kinematics(get_joint_states());
        // Uncomment here for debugging params
        //cout << "actPos: " << actPose.xyz[0] << " " << actPose.xyz[1] << " " << actPose.xyz[2] << " " << rotm2eul(actPose.rot)[2] << endl;
        loop_rate.sleep();
    }

    if(waitRot) {
        actPose = direct_kinematics(get_joint_states());
        float gap = 0.03;
        bool printed = false;
        while(abs(rotm2eul(actPose.rot)[2] - phief[2]) > gap) {
            actPose = direct_kinematics(get_joint_states());
            // Uncomment here for debugging params
            //cout << "actPos: " << actPose.xyz[0] << " " << actPose.xyz[1] << " " << actPose.xyz[2] << " " << rotm2eul(actPose.rot)[2] << endl;
            gap += 0.007;
            if(gap > 0.10 && !printed) {
                cout << "Waiting for gripper to complete its movement..." << endl << endl;
                printed = true;
            }
            loop_rate.sleep();
        }
    }
}
