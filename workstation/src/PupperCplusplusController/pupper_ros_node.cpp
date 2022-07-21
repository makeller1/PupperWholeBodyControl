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
    uint32_t msg_seq_; // message number
    bool joint_init = false;
    bool pose_init  = false;
    bool shutdown = false;
    bool updated = false; // has a new message been received?
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
    msg_seq_ = msg->header.seq;
    // Record the robot data
    std::copy(msg->position.begin(), msg->position.end(), joint_positions_.data());
    std::copy(msg->velocity.begin(), msg->velocity.end(), joint_velocities_.data());
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
    updated = true;
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

    // Run controller at 1000 Hz
    ros::Rate rate(1000);

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
            static int k = 0;
            k = k + 1;
            auto time_main_loop_start = ros::Time::now().toSec();
            // Zero the orientation to the initial state
            Eigen::Quaterniond correct_quat = initial_quaterion_.conjugate() * robot_quaternion_;

            // Print orientation error
            // VectorNd error3d;
            // error3d = correct_quat.vec() * correct_quat.w()/abs(correct_quat.w());
            // std::cout << "error3d: " << 100.0 * error3d.transpose() << std::endl;

            // Update the robot state
            Eigen::Vector3d ang_velocity; // estimate angular velocity
            Pup.updateController(joint_positions_, joint_velocities_, correct_quat, ang_velocity, contacts, time_now);

            // Update robot COM pos
            Eigen::Vector2d lateral_pos_est;
            lateral_pos_est = estimateLateralPos(Pup, correct_quat);
            robot_pos(0) = lateral_pos_est(0);
            robot_pos(1) = lateral_pos_est(1);
            robot_pos(2) = estimateHeight(Pup, contacts);

            // Update the tasks states
            Pup.updateBodyPosTask("COM_HEIGHT", robot_pos);
            Pup.updateBodyPosTask("COM_LATERAL_POS", robot_pos);
            Pup.updateBodyOriTask("COM_ORIENTATION", correct_quat);
            Pup.updateJointTask("JOINT_ANGLES", Pup.getJointPositions().segment(6,12));
            Pup.updateBodyPosTask("BACK_LEFT_FOOT_POS",   Pup.calcBodyPosInBaseCoordinates("back_left_foot"));
            Pup.updateBodyPosTask("BACK_RIGHT_FOOT_POS",  Pup.calcBodyPosInBaseCoordinates("back_right_foot"));
            Pup.updateBodyPosTask("FRONT_LEFT_FOOT_POS",  Pup.calcBodyPosInBaseCoordinates("front_left_foot"));
            Pup.updateBodyPosTask("FRONT_RIGHT_FOOT_POS", Pup.calcBodyPosInBaseCoordinates("front_right_foot"));

            // Update goal tasks
            taskmaster_.updateGoalTasks(Pup, time_now);

            // Run the IHWBC
            array<float,12> tau;
            try{
                tau = Pup.calculateOutputTorque(); 
            }
            catch(std::runtime_error& e){
                std::cout << e.what() << std::endl;
                Pup.np_logger.saveData(); // save test log
                throw(std::runtime_error(e.what()));
                // TODO: Engage motor braking
            }
            array<float,12> q_ddot_des = Pup.getOptimalAccel();
            
            // Log task data to numpy logger
            Pup.printDiag(); 

            // Neglect first solve (setup time and first solution can be significant > 10ms)
            // Todo: do something better here to allow WBC time to initialize
            if (k==1){
                tau.fill(0.0);
            }

            // Send commands
            std::copy(tau.begin(), tau.end(), command_msg.data.data());
            std::copy(q_ddot_des.begin(), q_ddot_des.end(), command_msg.data.data() + tau.size());

            RobotCommandPub.publish(command_msg);
            updated = false;
            std::cout << "dt since last run (ms): " << (time_now-time_last) * 1000.0f << std::endl;
            time_last = time_now;
            // Log time required to run wbc
            Pup.np_logger.logScalars({"wbc_ms"},{(ros::Time::now().toSec()-time_main_loop_start) * 1000.0f});

            // Shutdown if python node asks
            if (shutdown == true){
                Pup.np_logger.saveData("out_hardware"); // save test log
                ros::shutdown();
            }
        }
    rate.sleep();
    }
}