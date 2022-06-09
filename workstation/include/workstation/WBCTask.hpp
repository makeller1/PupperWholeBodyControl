#ifndef WBCTASK_HH_
#define WBCTASK_HH_

#include <vector>
#include <string>
#include <array>
#include "Eigen/Dense"
#include <unordered_map>

enum TaskType{
    BODY_POS,
    BODY_ORI,
    JOINT_POS,
    REACTION_FORCE
};

// Note: for embeddded use, we want to avoid dynamically sized vectors/matrices, but we're stuck for now since RBDL templates require dynamic matrices.
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
    
    Eigen::Matrix<double, 12, 1> joint_target;
    Eigen::Matrix<double, 12, 1> joint_measured;
    Eigen::Matrix<double, 12, 1> last_joint_measured;
    Eigen::Matrix<double, 12, 1> djoint_target; // joint velocity
    Eigen::Matrix<double, 12, 1> djoint_measured; // joint velocity

    Eigen::Quaternion<double> quat_target;
    Eigen::Quaternion<double> quat_measured;
    Eigen::Quaternion<double> last_quat_measured;

    Eigen::Vector3d pos_target;
    Eigen::Vector3d pos_measured;
    Eigen::Vector3d last_pos_measured;
    Eigen::Vector3d dpos_target;
    Eigen::VectorXd dpos_measured; // body velocity - not currently in use
    Eigen::Vector3d x_ddot_ff; // feedforward acceleration - zero initialized

    // Coefficients for the PD error term 
    Eigen::Vector3d Kp;
    Eigen::Vector3d Kd;

    // Previous Jacobian
    Eigen::MatrixXd j_prev; 
    bool j_prev_updated = false;

    // For diagnostics
    Eigen::VectorXd x_ddot_desired; // Desired task accelerations
    Eigen::VectorXd rf_desired; // Desired task reaction force
};

#endif