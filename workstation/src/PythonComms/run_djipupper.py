#!/usr/bin/env python
################ FOR ROS #######################
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
################################################

import numpy as np
from KeyboardController.JoystickInterface import JoystickInterface
from SerialInterface import HardwareInterface
from SerialInterface.IndividualConfig import SERIAL_PORT 
from pupper_whisperer import whisperer 

class Data:
    def __init__(self):
        # Create a list to hold the joint commands to send to the pupper
        self.commands = [0.0]*12


def main():
    """Main program"""
    data = Data()

    # Create a robot joint state message of size 12
    state_msg = JointState()
    state_msg.name     = [""] * 12
    state_msg.position = [0]  * 12
    state_msg.velocity = [0]  * 12
    state_msg.effort   = [0]  * 12

    pose_msg = Pose()

    # Create pupper whisperer which will provide I/O comms through gazebo node
    PupComm = whisperer()

    # Set the names of the joints
    for name, index in PupComm.joint_name_map.items():
        state_msg.name[index] = name

    # Define the message callback
    def commandCallback(msg):
        data.commands = list(msg.data)
    
    # Create the ROS subscriber and publisher
    state_pub   = rospy.Publisher("pupper_state", JointState, queue_size=1)
    pose_pub    = rospy.Publisher("pupper_pose", Pose, queue_size=1)
    command_sub = rospy.Subscriber("pupper_commands", Float64MultiArray, commandCallback, queue_size=1)

    # Run at 1000 Hz
    rate = rospy.Rate(1000)

    # Setup interfaces
    hardware_interface = HardwareInterface.HardwareInterface(port=SERIAL_PORT)
    joystick_interface = JoystickInterface() #Keyboard interface

    # Put Pupper into torque control mode
    hardware_interface.set_trq_mode()

    # The zero position is set with pupper laying down with elbows back. 
    input("Press enter to ZERO MOTORS") # - mathew    

    # Zero motors
    hardware_interface.zero_motors() 
    print("Zeroing Done")
    hardware_interface.set_max_current(1.0) # - mathew
    
    current_commands = [0]*12
    try:
        while not rospy.is_shutdown():

            command = joystick_interface.get_command()
            
            PupComm.print_states()

            # Read data from pupper
            PupComm.store_robot_states(hardware_interface.get_robot_states())

            # Put into the correct form
            for i, name in enumerate(PupComm.joint_name_map.keys()):
                (joint_pos, joint_vel, joint_cur) = PupComm.get_joint_state(name)
                state_msg.position[i] = joint_pos
                state_msg.velocity[i] = joint_vel
                state_msg.effort[i]   = joint_cur

            # Send the robot state to the C++ node
            state_pub.publish(state_msg)

            # Get Orientation
            quaternion = PupComm.get_pupper_orientation()

            pose_msg.position.x = 0 # Replace with function
            pose_msg.position.y = 0 
            pose_msg.position.z = 0
            pose_msg.orientation.w = quaternion[0] # Replace with function
            pose_msg.orientation.x = quaternion[1]
            pose_msg.orientation.y = quaternion[2]
            pose_msg.orientation.z = quaternion[3]  

            #Send Orientation to the C++ node
            pose_pub.publish(pose_msg)

            # Read torque command from ROS
            WBC_commands_reordered = PupComm.reorder_torques(data.commands)

            print(WBC_commands_reordered)

            # -------------------------------------------
            # -------- Send torques to pupper ---------- 
            # ------------------------------------------- 

            # Scaling factors found in C610.cpp in Stanford's code
            for i in range(12):
                # Convert torque to current
                torque_cmd = WBC_commands_reordered[i]
                vel        = PupComm.robot_states_["vel"][i]
                current_commands[i] = 1/0.308 * (torque_cmd + 0.0673 * np.sign(vel) + .00277 * vel) 
                # Over-rise current 
                current_commands[i] = 0

            hardware_interface.set_torque(current_commands) # FR, FL, BR, BL
            
            # Sleep to maintain the desired frequency
            rate.sleep()

            # Check if the pupper has faulted
            if PupComm.check_errors():
                print("Quitting program")
                break

            # Break loop when "q" is pressed
            if command.activate_event == 1:
                hardware_interface.set_torque([0]*12)
                print("Stopping motors.")
                break

    except KeyboardInterrupt:
        hardware_interface.set_torque([0]*12) # Zero torques when quit

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("pupper_coms_node", anonymous=True)
    main() 