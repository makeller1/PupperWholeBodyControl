#include <Arduino.h>

#include "ros.h"
#include "workstation/pupper_reply.h"
#include "workstation/pupper_command.h"

// <===== Create ROS global variables =====>
ros::NodeHandle nh;

// Data to be sent back to the workstation
workstation::pupper_reply reply_msg;
ros::Publisher reply_pub("pupper_feedback", &reply_msg);

// Data being received from the workstation
workstation::pupper_command workstation_command;
void pupperCommandCallback(const workstation::pupper_command& msg){
    workstation_command = msg;
}
ros::Subscriber<workstation::pupper_command> command_sub("pupper_commands", &pupperCommandCallback);

// Just a dummy function for illustration purposes
void controlPupper(float* joint_torques){
    float sum = 0;
    for (int i = 0; i < 12; i++){
        sum += joint_torques[i];
    }
}

void Setup(){
    // Initialize the teensy as a ROS node
    nh.initNode();

    // Subscribe to the "pupper_commands" topic
    nh.subscribe(command_sub);

    // Activate the publisher to send data
    nh.advertise(reply_pub);
}

void Loop(){
    // Check for new ROS messages
    nh.spinOnce();

    // Use the command message to control the pupper
    controlPupper(workstation_command.torques);

    // Send the feedback
    reply_msg.joint_pos[0] = 2;
    reply_msg.joint_vel[5] = -1000;
    reply_msg.joint_trq[2] = 1.784;
    reply_pub.publish(&reply_msg);

    // Some other debug info we'd like to see
    nh.loginfo("This will show up on the PC terminal window");

    delay(10);
}