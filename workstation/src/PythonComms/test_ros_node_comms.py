# This code used to test the ros node for topic message delays, drops, etc.
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
from scipy.spatial.transform import Rotation# For debugging
import time 

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
    state_msg.position = [0]  * 12 # [.06539, 1.19682, 2.71176, -.06539, -1.19682, -2.71176, .06539, 1.19682, 2.71176, -.06539, -1.19682, -2.71176]
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

    # Run at 500 Hz
    rate = rospy.Rate(500)

    # Setup interfaces
    hardware_interface = HardwareInterface.HardwareInterface(port=SERIAL_PORT)
    joystick_interface = JoystickInterface() #Keyboard interface

    # Put Pupper into torque control mode
    hardware_interface.set_trq_mode()

    # Flush first few messages of serial to avoid false fault signal
    for i in range(5):
        PupComm.store_robot_states(hardware_interface.get_robot_states())
        time.sleep(.002)

    # Zero motors
    hardware_interface.zero_motors() 
    MAX_CURRENT = 0.5
    hardware_interface.set_max_current(MAX_CURRENT) # Saturation (not fault)
    k = 0
    current_commands = [0]*12
    time.sleep(3.0)
    init_time = time.time() # seconds
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
                state_msg.position[i] = 0
                state_msg.velocity[i] = 0
                state_msg.effort[i]   = 0

            state_msg.position[-1] = k
            print("k = ",k, " q: {:.3f}".format(state_msg.position[-1]))
            # Send the robot state to the C++ node
            print("time (ms):", (time.time()-init_time)*1000.0)
            state_pub.publish(state_msg)

            pose_msg.position.x = 0 
            pose_msg.position.y = 0 
            pose_msg.position.z = 0
            pose_msg.orientation.w = 1 
            pose_msg.orientation.x = 0
            pose_msg.orientation.y = 0
            pose_msg.orientation.z = 0  

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
                # Convert torque to current in the Teensy frame
                torque_cmd = WBC_trq_reordered[i] #Desired torque
                q_ddot_des = WBC_dir_reordered[i] #Direction of desired acceleration
                vel = PupComm.robot_states_["vel"][i] #Already in teensy frame
                current_commands[i] = trq_to_current(torque_cmd, q_ddot_des, vel)

            if np.any(np.array(current_commands) > MAX_CURRENT):
                # print("Motors saturating")
                donothing = 1
            
            # Check if the pupper has faulted
            if PupComm.check_errors() == True: # or k > 100: # Note: breaking here should be avoided since it causes a delay in the Teensy which affects fault handling
                print("Joint positions: "+' '.join('{:.2f}'.format(f) for f in PupComm.robot_states_["pos"]))
                print("Joint velocities: "+' '.join('{:.2f}'.format(f) for f in PupComm.robot_states_["vel"]))
                # Ugly but simple way of shutting down the C++ node from this node
                state_msg.position[0] = 0.012345
                state_pub.publish(state_msg)
                rospy.signal_shutdown("Pupper faulted.")

            k += 1
            # Break loop when "q" is pressed
            if command.activate_event == 1:
                hardware_interface.set_torque([0]*12)
                print("Stopping motors.")
                state_msg.position[0] = 0.012345
                state_pub.publish(state_msg)
                rospy.signal_shutdown("Manual stop.")

            # Sleep to maintain the desired frequency
            rate.sleep()

    except KeyboardInterrupt:
        hardware_interface.set_torque([0]*12) # Zero torques when quit
        state_msg.position[0] = 0.012345
        state_pub.publish(state_msg)
        rospy.signal_shutdown("Manual stop.")
        print("Stopping motors.")
    finally:
        hardware_interface.set_torque([0]*12) # Zero torques when quit
        rospy.signal_shutdown("Manual stop.")
        print("Stopping motors.")

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("pupper_coms_node", anonymous=True, disable_signals=True)
    main() 