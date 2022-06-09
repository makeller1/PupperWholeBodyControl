#include "workstation/WBCTaskMaster.hpp"
#include <iostream>
#include <workstation/WBCTask.hpp>
#include "rbdl/rbdl_math.h"
#include "eigen3/Eigen/Dense"
#include <iomanip> // for setting cout precision

using std::cout;
using std::endl;
using std::string;

TaskMaster::TaskMaster(){
    current_goal_ = GoalName::STAND;
    cout << "TASK MASTER INITIALIZED" << endl;
}

void TaskMaster::setGoal(GoalName goal, PupperWBC& WBC_, std::array<bool,4>& feet_in_contact){
    
    switch (goal){
        case GoalName::THREE_LEG_STANCE:
        {
            // =========================================================================================
            // ----------------------------   GOAL: THREE LEG STANCE        ----------------------------
            // =========================================================================================
            current_goal_ = GoalName::THREE_LEG_STANCE;
            init_angles = {-0.4,  M_PI_4,  M_PI_2, // Sets initial simulation configuration
                            0.4, -M_PI_4, -M_PI_2,
                            0.0,  M_PI_4,  M_PI_2,
                            0.0, -M_PI_4, -M_PI_2}; 
            time_init_ms = 5e3;
            time_fall_ms = 5e3; // Same time indicates Pupper will not fall

            // WBC parameters
            WBC_.lambda_q = 0.001; 
            WBC_.lambda_rf_z = 0; 
            WBC_.lambda_rf_xy = 0; 
            WBC_.w_rf_xy = 0; 
            WBC_.w_rf_z = 0;
            WBC_.mu = .3;

            // Manually prescribe which feet are in contact (BL,BR,FL,FR)
            feet_in_contact = {true,true,true,false};

            // Task for Body center of mass to be 10cm high // range .02
            static Task CoM_Position_Task;
            CoM_Position_Task.type    = BODY_POS;
            CoM_Position_Task.body_id = "bottom_PCB";
            CoM_Position_Task.task_weight = 100; // 10
            CoM_Position_Task.active_targets = {true, true, true};   
            CoM_Position_Task.pos_target << 0, 0, 0.08;
            CoM_Position_Task.Kp << 0,0,100; // zeros indicate we want the acceleration to be zero 
            CoM_Position_Task.Kd << 0,0,10; 
            CoM_Position_Task.x_ddot_ff << 0, 0, 0;

            // Task for Body center of mass to be flat // .001
            static Task CoM_Orientation_Task;
            CoM_Orientation_Task.type    = BODY_ORI;
            CoM_Orientation_Task.body_id = "bottom_PCB";
            CoM_Orientation_Task.task_weight = 100; // 5000
            CoM_Orientation_Task.quat_target = Eigen::Quaternion<double>::Identity();
            CoM_Orientation_Task.active_targets = {true, true, true};
            float com_ori_Kp = 1000;
            float com_ori_Kd = 100;
            CoM_Orientation_Task.Kp << com_ori_Kp, com_ori_Kp, com_ori_Kp; 
            CoM_Orientation_Task.Kd << com_ori_Kd, com_ori_Kd, com_ori_Kd; 
            CoM_Orientation_Task.x_ddot_ff << 0, 0, 0;

            static Task JointPositionTask; 
            JointPositionTask.type = JOINT_POS;
            JointPositionTask.body_id = "joint";
            JointPositionTask.task_weight = 0; // 1
            JointPositionTask.joint_target <<   0.0,  M_PI_4,  M_PI_2, 
                                                0.0, -M_PI_4, -M_PI_2,
                                                0.0,  M_PI_4,  M_PI_2,
                                                0.0, -M_PI_4, -M_PI_2;
            JointPositionTask.active_targets = {true, false, false, true, false, false, true, false, false, true, false, false};
            JointPositionTask.Kp << 0,0,0; //600 : The three gains are applied to each leg
            JointPositionTask.Kd << 0,0,0; //25 : When tuning remember: tau = M*q_ddot + ... and q_dotdot = Kp (theta-theta_d) + Kd(omega)

            Eigen::Vector3d foot_pos_Kp = {100, 100, 0}; 
            Eigen::Vector3d foot_pos_Kd = {20, 20, 0}; 
            float foot_pos_w  = 1; // .01

            // Keep the back left foot in place
            static Task BLFootTask;
            BLFootTask.type = BODY_POS;
            BLFootTask.body_id = "back_left_foot";
            BLFootTask.task_weight = foot_pos_w;
            BLFootTask.active_targets = {true, true, true};  
            BLFootTask.pos_target << -0.11, 0.165, -0.08;
            BLFootTask.Kp = foot_pos_Kp;
            BLFootTask.Kd = foot_pos_Kd;

            // Keep the back right foot in place
            static Task BRFootTask;
            BRFootTask.type = BODY_POS;
            BRFootTask.body_id = "back_right_foot";
            BRFootTask.task_weight = foot_pos_w;
            BRFootTask.active_targets =  {true, true, true}; 
            BRFootTask.pos_target << -0.11, -0.165, -0.08;
            BRFootTask.Kp = foot_pos_Kp;
            BRFootTask.Kd = foot_pos_Kd;

            // Keep the front left foot in place
            static Task FLFootTask;
            FLFootTask.type = BODY_POS;
            FLFootTask.body_id = "front_left_foot";
            FLFootTask.task_weight = foot_pos_w;
            FLFootTask.active_targets = {true, true, true}; 
            FLFootTask.pos_target << 0.08, 0.045, -0.08; // .075
            FLFootTask.Kp = foot_pos_Kp;
            FLFootTask.Kd = foot_pos_Kd;

            // Keep the front right foot in place
            static Task FRFootTask;
            FRFootTask.type = BODY_POS;
            FRFootTask.body_id = "front_right_foot";
            FRFootTask.task_weight = 0.01;
            FRFootTask.active_targets = {true, true, true};  
            FRFootTask.pos_target << 0.08, -0.075, -0.04; 
            FRFootTask.x_ddot_ff << 0.0, 0.0, 0.0;
            FRFootTask.Kp << 2000, 2000, 2000;
            FRFootTask.Kd << 20, 20, 20;

            WBC_.addTask("COM_POSITION", &CoM_Position_Task);
            WBC_.addTask("COM_ORIENTATION", &CoM_Orientation_Task);
            WBC_.addTask("JOINT_ANGLES", &JointPositionTask);

            // Foot position tasks to enforce zero acceleration of contacting feet
            WBC_.addTask("BACK_LEFT_FOOT_POS", &BLFootTask);
            WBC_.addTask("BACK_RIGHT_FOOT_POS", &BRFootTask);
            WBC_.addTask("FRONT_LEFT_FOOT_POS", &FLFootTask);
            WBC_.addTask("FRONT_RIGHT_FOOT_POS", &FRFootTask);
            break;
        }
        case GoalName::GETUP:
        {
            // =========================================================================================
            // -----------------------------        GOAL: GET UP        --------------------------------
            // =========================================================================================
            current_goal_ = GoalName::GETUP;
            init_angles = {0.0,  M_PI_4,  M_PI_2, // Sets initial simulation configuration
                           0.0, -M_PI_4, -M_PI_2,
                           0.0,  M_PI_4,  M_PI_2,
                           0.0, -M_PI_4, -M_PI_2}; 
            time_init_ms = 3e3; 
            time_fall_ms = 5e3;

            // WBC parameters
            WBC_.lambda_q = 0.001; 
            WBC_.rf_desired = VectorNd::Zero(12);
            WBC_.lambda_rf_z = 0; // Minimize impacts
            WBC_.lambda_rf_xy = 0; 
            WBC_.w_rf_xy = 0; // Tracking weight
            WBC_.w_rf_z = 0;
            WBC_.mu = .1;

            // Manually prescribe which feet are in contact (BL,BR,FL,FR)
            feet_in_contact = {true,true,true,true};

            // Task for Body center of mass to be 10cm high // range .02
            static Task CoM_Position_Task;
            CoM_Position_Task.type    = BODY_POS;
            CoM_Position_Task.body_id = "bottom_PCB";
            CoM_Position_Task.task_weight = 100; // 10
            CoM_Position_Task.active_targets = {true, true, true};   
            CoM_Position_Task.pos_target << 0, 0, 0.08;
            CoM_Position_Task.Kp << 20,20,100; // zeros for Kp and Kd indicate we want the acceleration to be zero (20,5), (100,10)
            CoM_Position_Task.Kd << 5,5,10;
            CoM_Position_Task.x_ddot_ff << 0, 0, 0;

            // Task for Body center of mass to be flat // .001
            static Task CoM_Orientation_Task;
            CoM_Orientation_Task.type    = BODY_ORI;
            CoM_Orientation_Task.body_id = "bottom_PCB";
            CoM_Orientation_Task.task_weight = 100; // 100
            CoM_Orientation_Task.quat_target = Eigen::Quaternion<double>::Identity();
            CoM_Orientation_Task.active_targets = {true, true, true};
            float com_ori_Kp = 1000;
            float com_ori_Kd = 100;
            CoM_Orientation_Task.Kp << com_ori_Kp, com_ori_Kp, com_ori_Kp; 
            CoM_Orientation_Task.Kd << com_ori_Kd, com_ori_Kd, com_ori_Kd; 
            CoM_Orientation_Task.x_ddot_ff << 0, 0, 0;

            static Task JointPositionTask; 
            JointPositionTask.type = JOINT_POS;
            JointPositionTask.body_id = "joint";
            JointPositionTask.task_weight = 0; // 1
            JointPositionTask.joint_target <<   0.0,  M_PI_4,  M_PI_2, 
                                                0.0, -M_PI_4, -M_PI_2,
                                                0.0,  M_PI_4,  M_PI_2,
                                                0.0, -M_PI_4, -M_PI_2;
            JointPositionTask.active_targets = {true, false, false, true, false, false, true, false, false, true, false, false};
            JointPositionTask.Kp << 100,0,0; //600 : The three gains are applied to each leg
            JointPositionTask.Kd << 10,0,0; //25 : When tuning remember: tau = M*q_ddot + ... and q_dotdot = Kp (theta-theta_d) + Kd(omega)

            Eigen::Vector3d foot_pos_Kp = {0, 0, 0}; // zeros indicate desired acceleration = zero 
            Eigen::Vector3d foot_pos_Kd = {0, 0, 0};   // zeros indicate desired acceleration = zero 
            float foot_pos_w  = 100;

            // Enforce zero acceleration of contacting feet
            static Task BLFootTask;
            BLFootTask.type = BODY_POS;
            BLFootTask.body_id = "back_left_foot";
            BLFootTask.task_weight = foot_pos_w;
            BLFootTask.active_targets = {true, true, true};  
            BLFootTask.Kp = foot_pos_Kp;
            BLFootTask.Kd = foot_pos_Kd;

            static Task BRFootTask;
            BRFootTask.type = BODY_POS;
            BRFootTask.body_id = "back_right_foot";
            BRFootTask.task_weight = foot_pos_w;
            BRFootTask.active_targets =  {true, true, true}; 
            BRFootTask.Kp = foot_pos_Kp;
            BRFootTask.Kd = foot_pos_Kd;

            static Task FLFootTask;
            FLFootTask.type = BODY_POS;
            FLFootTask.body_id = "front_left_foot";
            FLFootTask.task_weight = foot_pos_w;
            FLFootTask.active_targets = {true, true, true}; 
            FLFootTask.Kp = foot_pos_Kp;
            FLFootTask.Kd = foot_pos_Kd;

            static Task FRFootTask;
            FRFootTask.type = BODY_POS;
            FRFootTask.body_id = "front_right_foot";
            FRFootTask.task_weight = foot_pos_w;
            FRFootTask.active_targets = {true, true, true};  
            FRFootTask.Kp << foot_pos_Kp;
            FRFootTask.Kd << foot_pos_Kd;

            WBC_.addTask("COM_POSITION", &CoM_Position_Task);
            WBC_.addTask("COM_ORIENTATION", &CoM_Orientation_Task);
            WBC_.addTask("JOINT_ANGLES", &JointPositionTask);

            // Foot position tasks
            WBC_.addTask("BACK_LEFT_FOOT_POS", &BLFootTask);
            WBC_.addTask("BACK_RIGHT_FOOT_POS", &BRFootTask);
            WBC_.addTask("FRONT_LEFT_FOOT_POS", &FLFootTask);
            WBC_.addTask("FRONT_RIGHT_FOOT_POS", &FRFootTask);

            break;
        }
    }
}

void TaskMaster::updateGoalTasks(PupperWBC& WBC_, const double& time_ms)
{
    switch (this->current_goal_){
        case GoalName::THREE_LEG_STANCE:
        {
            // Oscillate floating foot position
            float target_z = -0.04 + 0.04*sin(3.0*time_ms/1000.0); // 3 Hz (.33 sec)
            float target_y = -0.075 + 0.04*cos(3.0*time_ms/1000.0);
            WBC_.getTask("FRONT_RIGHT_FOOT_POSITION")->pos_target << 0.08, target_y, target_z; 
            break;
        }
        case GoalName::GETUP:
        {
            // Update the desired COM_Position as the average of the support polygon edges (contacts)
            static Vector3_t support_avg_lp; // IIR low pass filtered support average
            double lp_coeff = 0.81; // 200 hz cutoff with Ts = 1000 Hz
            Vector3_t support_avg = Vector3_t::Zero(3); // Instantaneous support average
            
            // Compute average of support polygon edges
            support_avg += WBC_.getRelativeBodyLocation("back_left_lower_link", WBC_.body_contact_point_left);
            support_avg += WBC_.getRelativeBodyLocation("back_right_lower_link", WBC_.body_contact_point_right);
            support_avg += WBC_.getRelativeBodyLocation("front_left_lower_link", WBC_.body_contact_point_left);
            support_avg += WBC_.getRelativeBodyLocation("front_right_lower_link", WBC_.body_contact_point_right);

            // Low pass to remove derivative impulses due to moving contacts
            support_avg_lp = lp_coeff*support_avg_lp + (1-lp_coeff)*support_avg;
            
            WBC_.getTask("COM_POSITION")->pos_target(0) = support_avg_lp(0);
            WBC_.getTask("COM_POSITION")->pos_target(1) = support_avg_lp(1); 
            break;
        }
        case GoalName::STAND:
        {
            // // Oscillate COM height task
            // float target_height = 0.12 + 0.02*sin(0.5*simtime_); // 0.5 Hz (2 sec)
            // WBC_.getTask("COM_POSITION")->pos_target.z() = target_height; 

            // // Oscillate left/right tilt
            // float target_roll = M_PI/12 * sin(0.5 * simtime_); // 0.5 HZ (2 sec)
            // WBC_.getTask("COM_ORIENTATION")->quat_target = Eigen::AngleAxisd(target_roll, Eigen::Vector3d::UnitX());
            break;
        }
    }    
}