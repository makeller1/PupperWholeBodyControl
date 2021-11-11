#ifndef WBCTASK_HH_
#define WBCTASK_HH_

#include <vector>
#include <string>
#include <array>
#include "Eigen/Dense"

enum TaskType{
    BODY_POS,
    BODY_ORI,
    JOINT_POS
};

// Note: for embeddded use, we want to avoid dynamically sized vectors/matrices
struct Task{    
    // Which body this task is for (use JOINT for a joint position task)
    std::string body_id;
    Eigen::Vector3d offset = Eigen::Vector3d::Zero();

    // What type of task is this (body_pos, body_ori, or joint_pos)
    TaskType type;

    // The weight to be used in IHWBC
    float task_weight = 0;

    // The desired task goal and which to consider for this task
    std::vector<bool> active_targets;
    
    Eigen::VectorXd joint_target;
    Eigen::VectorXd joint_measured;
    Eigen::VectorXd last_joint_measured;

    Eigen::Quaternion<double> quat_target;
    Eigen::Quaternion<double> quat_measured;
    Eigen::Quaternion<double> last_quat_measured;

    Eigen::Vector3d pos_target;
    Eigen::Vector3d pos_measured;
    Eigen::Vector3d last_pos_measured;

    // Coefficients for the PD error term 
    double Kp;
    double Kd;

    // Previous Jacobian
    Eigen::MatrixXd j_prev; // Remove for embedded
    bool j_prev_updated = false;

};

#endif