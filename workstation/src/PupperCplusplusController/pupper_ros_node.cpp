#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "workstation/PupperModel.h"
#include "workstation/PupperWBC.hpp"
#include "workstation/WBCTaskMaster.hpp"
#include "workstation/StateEstimation.hpp"

using std::array;

// Global variables
namespace {
    VectorNd joint_positions_(12);
    VectorNd joint_velocities_(12);
    Eigen::Vector3d body_pos_(3);
    Eigen::Quaterniond robot_quaternion_;
    Eigen::Quaterniond initial_quaterion_;
    bool joint_init = false;
    bool pose_init  = false;
    bool shutdown = false;
    bool updated = false; // has a new message been received?
    double time_last_callback = 0.0; 
}

// This is called every time we receive a message from the Python node
void pupperStateCallBack(const sensor_msgs::JointStateConstPtr &msg){
    if (not joint_init){
        ROS_INFO("Joint states received");
        joint_init = true;
        std::cout << "initial position: ";
        for (int i=0; i<msg->position.size();i++){
            std::cout << msg->position[i] << " ";
        }
        std::cout << std::endl;
    }
    // auto a = msg->header.seq; // message count
    // Record the robot data
    std::copy(msg->position.begin(), msg->position.end(), joint_positions_.data());
    std::copy(msg->velocity.begin(), msg->velocity.end(), joint_velocities_.data());
    // std::cout << "message number: " << msg->position[11] << std::endl;
    // std::cout << "dt callback (ms): " << (ros::Time::now().toSec() - time_last_callback)*1000.0f << std:: endl;
    // time_last_callback = ros::Time::now().toSec();
    // Ugly but simple way of shutting down this node from the python node
    if (msg->position[0] == 0.012345){
        shutdown = true;
    }
    updated = true;
}

// Receive the robot pose from the onboard IMU
void pupperPoseCallBack(const geometry_msgs::PoseConstPtr &msg){ 
    // Record the robot pose
    robot_quaternion_.x() = msg->orientation.x;
    robot_quaternion_.y() = msg->orientation.y;
    robot_quaternion_.z() = msg->orientation.z;
    robot_quaternion_.w() = msg->orientation.w;
    // If this is the first time, record the quaternion
    if (not pose_init){
        pose_init = true;
        initial_quaterion_ = robot_quaternion_;
        ROS_INFO("Initial quaternion received: [%.2f, (%.2f, %.2f, %.2f)]", 
        initial_quaterion_.w(), initial_quaterion_.x(), initial_quaterion_.y(), initial_quaterion_.z());
    }
    // updated = true;
}

int main(int argc, char** argv){
    // Initialize ROS node
    ros::init(argc, argv, "pupper_control_node");
    ros::NodeHandle nh;

    // Create publisher and subscriber to communicate with pupper
    ros::Subscriber RobotStateSubscriber = nh.subscribe("pupper_state", 1, &pupperStateCallBack, ros::TransportHints().tcpNoDelay());
    ros::Subscriber RobotPoseSubscriber  = nh.subscribe("pupper_pose", 1, &pupperPoseCallBack, ros::TransportHints().tcpNoDelay());
    ros::Publisher  RobotCommandPub      = nh.advertise<std_msgs::Float64MultiArray>("pupper_commands", 1, false);

    // Create the Whole Body Controller
    PupperWBC Pup;
    Pup.Load(*createPupperModel());

    // Setup tasks and goals
    TaskMaster taskmaster_; // Sets tasks and updates tasks for a specific goal
    std::array<bool, 4> contacts; // Foot contacts with order: BL, BR, FL, FR

    // Choose goal
    GoalName goal = GoalName::GETUP;
    taskmaster_.setGoal(goal, Pup, contacts);

    // Run controller at 500 Hz
    ros::Rate rate(500);

    // Zero the globals
    body_pos_.setZero();
    joint_positions_.setZero();
    joint_velocities_.setZero();
    robot_quaternion_ = initial_quaterion_;
    Eigen::Vector3d robot_pos = Eigen::Vector3d::Zero(3);

    // Current time
    double time_now = ros::Time::now().toSec();
    double time_last = time_now;

    // Create the ROS message that we will be sending back to the Python node
    // top 12 are torque commands, bottom 12 are desired joint accelerations
    std_msgs::Float64MultiArray command_msg;
    command_msg.data.resize(24);

    // Wait for messages
    ROS_INFO("Waiting for initial message...");
    while(not pose_init or not joint_init and nh.ok()){
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    ROS_INFO("Starting IHWBC Algorithm");

    // Main loop
    while(nh.ok()){
        time_now = ros::Time::now().toSec();

        // Look for new ROS messages
        ros::spinOnce();

        if (updated){
            auto time_main_loop_start = ros::Time::now().toSec();
            // Offset the quaternion from our initial state
            Eigen::Quaterniond correct_quat = initial_quaterion_ * robot_quaternion_.conjugate();
            // ROS_INFO("Corrected quaternion: [%.2f, (%.2f, %.2f, %.2f)]", 
            // correct_quat.w(), correct_quat.x(), correct_quat.y(), correct_quat.z());

            // Print RPY values
            VectorNd error3d;
            error3d = correct_quat.vec() * correct_quat.w()/abs(correct_quat.w());
            std::cout << "error3d: " << 1000.0 * error3d.transpose() << std::endl;

            // Update the robot state
            Pup.updateController(joint_positions_, joint_velocities_, correct_quat, contacts, time_now);

            // Update robot height
            robot_pos[2] = estimateHeight(Pup, contacts);
            std::cout << "\n \n \n" << "Estimated height: " << robot_pos[2] <<  std::endl;

            // Update the tasks states
            Pup.updateBodyPosTask("COM_POSITION", robot_pos);
            Pup.updateBodyOriTask("COM_ORIENTATION", correct_quat);
            Pup.updateJointTask("JOINT_ANGLES", Pup.getJointPositions().segment(6,12));
            Pup.updateBodyPosTask("BACK_LEFT_FOOT_POS",   Pup.getRelativeBodyLocation("back_left_foot"));
            Pup.updateBodyPosTask("BACK_RIGHT_FOOT_POS",  Pup.getRelativeBodyLocation("back_right_foot"));
            Pup.updateBodyPosTask("FRONT_LEFT_FOOT_POS",  Pup.getRelativeBodyLocation("front_left_foot"));
            Pup.updateBodyPosTask("FRONT_RIGHT_FOOT_POS", Pup.getRelativeBodyLocation("front_right_foot"));

            // Update goal tasks
            taskmaster_.updateGoalTasks(Pup, time_now);

            // Run the IHWBC
            array<float,12> tau;
            try{
                tau = Pup.calculateOutputTorque(); // !!!!!!!!!!!!!!!!!!!!
            }
            catch(std::runtime_error& e){
                std::cout << e.what() << std::endl;
            }
            array<float,12> q_ddot_des = Pup.getOptimalAccel();
            
            // Print task information
            Pup.printDiag(); 

            // Shutdown if python node asks
            if (shutdown == true){
                ros::shutdown();
            }

            // Send commands
            std::copy(tau.begin(), tau.end(), command_msg.data.data());
            std::copy(q_ddot_des.begin(), q_ddot_des.end(), command_msg.data.data() + tau.size());
            RobotCommandPub.publish(command_msg);
            updated = false;
            std::cout << "dt (ms): " << (time_now-time_last) * 1000.0f << std::endl;
            time_last = time_now;
            std::cout << "message number: " << joint_positions_[11] << std::endl;
            std::cout << "traverse MAIN LOOP (ms): " << (ros::Time::now().toSec()-time_main_loop_start) * 1000.0f << std::endl;
        }
    rate.sleep();
    }
}