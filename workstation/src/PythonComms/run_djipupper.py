# TODO: Fix the torque to current transformation. 
#         - The velocity and torque are not aligning correctly so the driving mode doesn't change
#           from forward to backward.
#       

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
from utilities import trq_to_current

class Data:
    def __init__(self):
        # Create a list to hold the torque commands to send to the pupper
        self.trq_commands = [0.0]*12
        self.dir_commands = [False]*12

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
        commands = list(msg.data)
        data.trq_commands = commands[:12]
        data.dir_commands = list( (np.array(commands[12:]) > 0) - 0.5)

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
    input("Press enter to ZERO MOTORS")     

    # Zero motors
    hardware_interface.zero_motors() 
    print("Zeroing Done")
    hardware_interface.set_max_current(0.5) # Saturation (not fault)
    
    current_commands = [0]*12
    try:
        while not rospy.is_shutdown():

            command = joystick_interface.get_command()
            
            # #Print states
            # PupComm.print_states(0)
            # PupComm.print_states(1)

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
            
            # Read torque and accel direction command from ROS
            WBC_trq_commands = data.trq_commands
            WBC_dir_commands = data.dir_commands

            # Tranform to Teensy's frame
            WBC_trq_reordered = PupComm.reorder_commands(WBC_trq_commands)
            WBC_dir_reordered = PupComm.reorder_commands(WBC_dir_commands)
            # -------------------------------------------
            # -------- Send torques to pupper ---------- 
            # ------------------------------------------- 
            for i in range(12):
                # Convert torque to current
                torque_cmd = WBC_trq_reordered[i] #Desired torque
                q_ddot_des = WBC_dir_reordered[i] #Direction of desired acceleration

                vel = PupComm.robot_states_["vel"][i]
                current_commands[i] = trq_to_current(torque_cmd, q_ddot_des, vel)
            
            # print("Reordered commands:")
            # print("Desired currents: "+' '.join('{:.2f}'.format(f) for f in current_commands))
            # print("Desired torques : "+' '.join('{:.2f}'.format(f) for f in WBC_trq_reordered))
            # print("Desired dir.: ", WBC_dir_reordered)

            hardware_interface.set_torque(current_commands) # misnomer until trq to current transformation is done on Teensy
            
            # Sleep to maintain the desired frequency
            rate.sleep()

            # Check if the pupper has faulted
            PupComm.check_errors() # Note: breaking here should be avoided since it causes a delay in the Teensy which affects fault handling

            # Break loop when "q" is pressed
            if command.activate_event == 1:
                hardware_interface.set_torque([0]*12)
                print("Stopping motors.")
                break

    except KeyboardInterrupt:
        hardware_interface.set_torque([0]*12) # Zero torques when quit
        print("Stopping motors.")
    finally:
        hardware_interface.set_torque([0]*12) # Zero torques when quit
        print("Stopping motors.")

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("pupper_coms_node", anonymous=True)
    main() 