#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("pupper_coms_node", anonymous=True)

    # Create a class to communicate with the pupper
    PupComm = whisperer()

    # Create a robot joint state message of size 12
    state_msg = JointState()
    state_msg.name     = [""] * 12
    state_msg.position = [0]  * 12
    state_msg.velocity = [0]  * 12
    state_msg.effort   = [0]  * 12

    # Set the names of the joints
    for name, index in PupComm.joint_name_map.items():
        state_msg.name[index] = name

    # Create a list to hold the joint commands to send to the pupper
    pupper_commands = [0] * 12

    # Define the message callback
    def commandCallback(msg):
        pupper_commands = list(msg.data)
    
    # Create the subscriber and publisher
    state_pub   = rospy.Publisher("pupper_state", JointState, queue_size=1)
    command_sub = rospy.Subscriber("pupper_commands", Float64MultiArray, commandCallback, queue_size=10)

    # Run at 100 Hz
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():

        # Get the robot state and send the command here
        PupComm.sendCommand(pupper_commands)
        PupComm.getDataFromRobotOrSomethingLikeThatIDK()

        # Put into the correct form
        for i in range(12):
            [joint_pos, joint_vel, joint_cur] = PupComm.get_joint_state(name)
            state_msg.position[i] = joint_pos
            state_msg.velocity[i] = joint_vel
            state_msg.effort[i]   = joint_curr

        # Send the robot state to the C++ node
        state_pub.publish(state_msg)

        # Sleep to maintain the desired frequency
        rate.sleep()

