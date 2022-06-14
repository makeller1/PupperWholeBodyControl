#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"

using std::array;

// Global variables
namespace {
    bool joint_init = false;
    bool pose_init  = false;
    bool shutdown = false;
    bool updated = false; // has a new message been received?
    double time_last_callback = 0.0; 
    double time_init = 0.0;
    double message_number = 0.0;
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
    auto a = msg->header.seq; // message count
    message_number = msg->position[11];
    std::cout << "message number: " << msg->position[11] << std::endl;
    // std::cout << "dt callback (ms): " << (ros::Time::now().toSec() - time_last_callback)*1000.0f << std:: endl;
    std::cout << "Time (ms): " << (ros::Time::now().toSec() - time_init)*1000.0f << std::endl;
    time_last_callback = ros::Time::now().toSec();
    // Ugly but simple way of shutting down this node from the python node
    if (msg->position[0] == 0.012345){
        shutdown = true;
    }
    updated = true;
}

// Receive the robot pose from the onboard IMU
void pupperPoseCallBack(const geometry_msgs::PoseConstPtr &msg){ 
    // Record the robot pose
    auto x = msg->orientation.x;
    auto y = msg->orientation.y;
    auto z = msg->orientation.z;
    auto w = msg->orientation.w;
    // If this is the first time, record the quaternion
    if (not pose_init){
        pose_init = true;
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

    // Run controller at 500 Hz
    ros::Rate rate(500);

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
        ros::Duration(0.001).sleep();
    }
    ROS_INFO("Starting IHWBC Algorithm");
    time_init = ros::Time::now().toSec();

    // Main loop
    while(nh.ok()){
        time_now = ros::Time::now().toSec();

        // Look for new ROS messages
        ros::spinOnce();

        if (updated){
            auto time_main_loop_start = ros::Time::now().toSec();
            // Shutdown if python node asks
            if (shutdown == true){
                ros::shutdown();
            }
            RobotCommandPub.publish(command_msg);
            updated = false;
            std::cout << "dt MAIN LOOP (ms): " << (time_now-time_last) * 1000.0f << std::endl;
            time_last = time_now;
            std::cout << "MAIN LOOP ############################## Number: " << message_number << std::endl;
            auto time_main_loop_end = ros::Time::now().toSec();
            std::cout << "traverse MAIN LOOP (ms): " << (time_main_loop_end-time_main_loop_start) * 1000.0f << std::endl;
        }
    rate.sleep();
    }
}