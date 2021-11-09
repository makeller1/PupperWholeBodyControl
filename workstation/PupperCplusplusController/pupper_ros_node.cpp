#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "ase389/PupperModel.h"
#include "ase389/PupperWBC.hpp"

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
}

// This is called every time we receive a message from the Python node
void pupperStateCallBack(const sensor_msgs::JointStateConstPtr &msg){
    if (not joint_init){
        ROS_INFO("Joint states received");
        joint_init = true;
    }
    
    // Record the robot data
    std::copy(msg->position.begin(), msg->position.end(), joint_positions_.data());
    std::copy(msg->velocity.begin(), msg->velocity.end(), joint_velocities_.data());
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
}

int main(int argc, char** argv){
    // Initialize ROS node
    ros::init(argc, argv, "pupper_control_node");
    ros::NodeHandle nh;

    // Create publisher and subscriber to communicate with pupper
    ros::Subscriber RobotStateSubscriber = nh.subscribe("pupper_state", 1, &pupperStateCallBack);
    ros::Subscriber RobotPoseSubscriber  = nh.subscribe("pupper_pose", 1, &pupperPoseCallBack);
    ros::Publisher  RobotCommandPub      = nh.advertise<std_msgs::Float64MultiArray>("pupper_commands", 1, false);

    // Create the Whole Body Controller
    PupperWBC Pup;
    Pup.Load(*createPupperModel());

    // ============================  TASKS ============================= //

    // Task for Body center of mass to be 10cm high // range .02
    static Task CoM_Position_Task;
    CoM_Position_Task.type    = BODY_POS;
    CoM_Position_Task.body_id = "bottom_PCB";
    CoM_Position_Task.task_weight = 5; // 3(3) too low // 10 // 1
    CoM_Position_Task.active_targets = {false, false, true};    // only account for z-position
    CoM_Position_Task.pos_target << 0, 0, 0.15;
    CoM_Position_Task.Kp =  250;//60;// 40 // 25 // 16//  12 // 8 // 6// 4// 3//1000;
    CoM_Position_Task.Kd = 100;//20;// 20//   7 // 5  //  1 // .5

    // Task for Body center of mass to be flat // .001
    static Task CoM_Orientation_Task;
    CoM_Orientation_Task.type    = BODY_ORI;
    CoM_Orientation_Task.body_id = "bottom_PCB";
    CoM_Orientation_Task.task_weight = 200; // 45 // 15 // 10;
    CoM_Orientation_Task.quat_target = Eigen::Quaternion<double>::Identity();
    CoM_Orientation_Task.Kp = 4;//15;// 7 // 3 //1000;
    CoM_Orientation_Task.Kd = 1;//7; // 1.5 // .5

    // Task to keep the hips level
    static Task JointPositionTask; // .01
    JointPositionTask.type = JOINT_POS;
    JointPositionTask.task_weight = 1; //0.1;
    JointPositionTask.joint_target.resize(12);
    // JointPositionTask.joint_target << 0, M_PI_4, M_PI_2, 0, -M_PI_4, -M_PI_2, 0, M_PI_4, M_PI_2, 0, -M_PI_4, -M_PI_2;
    // JointPositionTask.active_targets = std::vector<bool>(12, true);
    JointPositionTask.joint_target = VectorNd::Zero(12);
    JointPositionTask.active_targets = {true, false, false, true, false, false, true, false, false, true, false, false}; // Hips only
    JointPositionTask.Kp = 3;
    JointPositionTask.Kd = .5;

    // Weights are shared between all four feet
    float foot_pos_Kp = 0; //1
    float foot_pos_Kd = 0; //0
    float foot_pos_w  = 5; // 40 maybe too high

    // Keep the front left foot in place
    static Task FLFootTask;
    FLFootTask.type = BODY_POS;
    FLFootTask.body_id = "front_left_foot";
    FLFootTask.task_weight = foot_pos_w;
    FLFootTask.active_targets = {true, true, false};  // We'll let the COM task take care of height
    FLFootTask.pos_target << 0.08, 0.075, -0.1;
    FLFootTask.Kp = foot_pos_Kp;
    FLFootTask.Kd = foot_pos_Kd;

    // Keep the front right foot in place
    static Task FRFootTask;
    FRFootTask.type = BODY_POS;
    FRFootTask.body_id = "front_right_foot";
    FRFootTask.task_weight = foot_pos_w;
    FRFootTask.active_targets = {true, true, false};  // We'll let the COM task take care of height
    FRFootTask.pos_target << 0.08, -0.065, -0.1;
    FRFootTask.Kp = foot_pos_Kp;
    FRFootTask.Kd = foot_pos_Kd;

    // Keep the back left foot in place
    static Task BLFootTask;
    BLFootTask.type = BODY_POS;
    BLFootTask.body_id = "back_left_foot";
    BLFootTask.task_weight = foot_pos_w;
    BLFootTask.active_targets = {true, true, false};  // We'll let the COM task take care of height
    BLFootTask.pos_target << -0.11, 0.075, -0.1;
    BLFootTask.Kp = foot_pos_Kp;
    BLFootTask.Kd = foot_pos_Kd;

    // Keep the back right foot in place
    static Task BRFootTask;
    BRFootTask.type = BODY_POS;
    BRFootTask.body_id = "back_right_foot";
    BRFootTask.task_weight = foot_pos_w;
    BRFootTask.active_targets = {true, true, false};  // We'll let the COM task take care of height
    BRFootTask.pos_target << -0.11, -0.065, -0.1;
    BRFootTask.Kp = foot_pos_Kp;
    BRFootTask.Kd = foot_pos_Kd;

    // Control the body
    Pup.addTask("COM_POSITION", &CoM_Position_Task);
    Pup.addTask("COM_ORIENTATION", &CoM_Orientation_Task);
    Pup.addTask("JOINT_ANGLES", &JointPositionTask);

    // Foot position tasks for standstill
    Pup.addTask("FRONT_LEFT_FOOT_POSITION", &FLFootTask);
    Pup.addTask("FRONT_RIGHT_FOOT_POSITION", &FRFootTask);
    Pup.addTask("BACK_LEFT_FOOT_POSITION", &BLFootTask);
    Pup.addTask("BACK_RIGHT_FOOT_POSITION", &BRFootTask);

    // ================================================================= //

    // Run controller at 100 Hz
    ros::Rate rate(100);  

    // Zero the globals
    body_pos_.setZero();
    joint_positions_.setZero();
    joint_velocities_.setZero();
    robot_quaternion_.setIdentity();
    Eigen::Vector3d robot_pos = Eigen::Vector3d::Zero(3);

    // Foot contacts
    std::array<bool, 4> contacts = {true, true, true, true};

    // Create the ROS message that we will be sending back to the Python node
    std_msgs::Float64MultiArray command_msg;
    command_msg.data.resize(12);

    // Wait for messages
    ROS_INFO("Waiting for initial message...");
    while(not pose_init or not joint_init and nh.ok()){
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    ROS_INFO("Starting IHWBC Algorithm");

    // Main loop
    while(nh.ok()){
        // Look for new ROS messages
        ros::spinOnce();

        // Offset the quaternion from our initial state
        Eigen::Quaterniond correct_quat = robot_quaternion_ * initial_quaterion_.conjugate();
        // ROS_INFO("Corrected quaternion: [%.2f, (%.2f, %.2f, %.2f)]", 
        // correct_quat.w(), correct_quat.x(), correct_quat.y(), correct_quat.z());

        // Calculate the robot height using forward kinematics
        robot_pos.z() = Pup.calcPupperHeight();
        std::cout << "height: " << robot_pos.z() << std::endl;
        
        // Update the robot state
        Pup.updateController(joint_positions_, joint_velocities_, body_pos_, correct_quat, contacts);

        // Update the tasks states
        Pup.updateBodyPosTask("COM_POSITION", robot_pos);
        Pup.updateBodyOriTask("COM_ORIENTATION", correct_quat);
        Pup.updateJointTask("JOINT_ANGLES", Pup.getJointPositions().segment(6,12));
        Pup.updateBodyPosTask("BACK_LEFT_FOOT_POSITION",   Pup.getRelativeBodyLocation("back_left_foot"));
        Pup.updateBodyPosTask("BACK_RIGHT_FOOT_POSITION",  Pup.getRelativeBodyLocation("back_right_foot"));
        Pup.updateBodyPosTask("FRONT_LEFT_FOOT_POSITION",  Pup.getRelativeBodyLocation("front_left_foot"));
        Pup.updateBodyPosTask("FRONT_RIGHT_FOOT_POSITION", Pup.getRelativeBodyLocation("front_right_foot"));
        // Run the IHWBC
        array<float,12> tau = Pup.calculateOutputTorque();

        // Send commands
        std::copy(tau.begin(), tau.end(), command_msg.data.data());
        RobotCommandPub.publish(command_msg);

        rate.sleep();
    }
}

