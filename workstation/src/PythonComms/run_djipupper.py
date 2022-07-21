# TODO: Fix the torque to current transformation. 
#         - The velocity and torque are not aligning correctly so the driving mode doesn't change
#           from forward to backward.
#       

#!/usr/bin/env python
################ FOR ROS #######################
from cmath import pi
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


def quaternion_relative(Q0,Q1):
    """
    Multiplies two quaternions Q0.conjugate() * Q1
    where Q0 is the initial quat and Q1 is the current quat
    Assumes unitary quaternions.

    Input
    :param Q0: A 4 element array containing the first quaternion (q01,q11,q21,q31) 
    :param Q1: A 4 element array containing the second quaternion (q02,q12,q22,q32) 

    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33) 

    """
    # Extract the values from Q0
    w0 = Q0[0]
    x0 = -Q0[1]
    y0 = -Q0[2]
    z0 = -Q0[3]

    # Extract the values from Q1
    w1 = Q1[0]
    x1 = Q1[1]
    y1 = Q1[2]
    z1 = Q1[3]

    # Computer the product of the two quaternions, term by term
    Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    # Create a 4 element array containing the final quaternion
    final_quaternion = [Q0Q1_w, Q0Q1_x, Q0Q1_y, Q0Q1_z]

    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32) 
    return final_quaternion

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
    command_sub = rospy.Subscriber("pupper_commands", Float64MultiArray, commandCallback, queue_size=1, tcp_nodelay=True)

    # Run at 1000 Hz
    rate = rospy.Rate(1000)

    # Setup interfaces
    hardware_interface = HardwareInterface.HardwareInterface(port=SERIAL_PORT)
    joystick_interface = JoystickInterface() #Keyboard interface

    # Put Pupper into torque control mode
    hardware_interface.set_trq_mode()

    # Flush first few messages of serial to avoid false fault signal
    for i in range(5):
        PupComm.store_robot_states(hardware_interface.get_robot_states())
        time.sleep(.002)

    # Get initial orientation
    quaternion_init = PupComm.get_pupper_orientation()

    # The zero position is set with pupper laying down with elbows back. 
    input("Press enter to ZERO MOTORS")     

    # Zero motors
    hardware_interface.zero_motors() 
    print("Zeroing Done")
    MAX_CURRENT = 8.0
    hardware_interface.set_max_current(MAX_CURRENT) # Saturation (not fault)
    current_commands = [0]*12
    time.sleep(3.0)
    k=0
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
                # Convert torque to current in the Teensy frame
                torque_cmd = WBC_trq_reordered[i] #Desired torque
                q_ddot_des = WBC_dir_reordered[i] #Direction of desired acceleration
                vel = PupComm.robot_states_["vel"][i] #Already in teensy frame
                current_commands[i] = trq_to_current(torque_cmd, q_ddot_des, vel)

            # #error3d is used in WBC to control orientation (x, y, z)
            # # quat_rel = quaternion_relative(quaternion_init,quaternion) #relative quaternion
            # if k%200==0:
            #     quat_rel = quaternion_relative(quaternion,quaternion_init) #relative quaternion
            #     error3d = 1000.0 * np.array([quat_rel[1],quat_rel[2],quat_rel[3]])*np.sign(quat_rel[0])
            #     print("Error 3d: {:.2f} {:.2f} {:.2f}".format(error3d[0], error3d[1], error3d[2]))
            #     r = Rotation.from_quat([quat_rel[1],quat_rel[2],quat_rel[3],quat_rel[0]])
            #     # print(Rotation.as_dcm(r))
            #     rot_vec = r.as_rotvec()
            #     print("Rot vec: {:.2f} {:.2f} {:.2f}".format(rot_vec[0],rot_vec[1],rot_vec[2]))

            if np.any(np.array(current_commands) > MAX_CURRENT):
                print("Motors saturating")
                # print("Desired currents: "+' '.join('{:.2f}'.format(f) for f in current_commands))
                
            # current_commands = [0]*12
            hardware_interface.set_torque(current_commands) # misnomer until trq to current transformation is done on Teensy
            
            # Check if the pupper has faulted
            if PupComm.check_errors() == True: # Note: breaking here should be avoided since it causes a delay in the Teensy which affects fault handling
                print("Joint positions: "+' '.join('{:.2f}'.format(f) for f in PupComm.robot_states_["pos"]))
                print("Joint velocities: "+' '.join('{:.2f}'.format(f) for f in PupComm.robot_states_["vel"]))
                # Ugly but simple way of shutting down the C++ node from this node
                state_msg.position[0] = 0.012345
                state_pub.publish(state_msg)
                rospy.signal_shutdown("Pupper faulted.")

            # Break loop when "q" is pressed
            if command.activate_event == 1:
                hardware_interface.set_torque([0]*12)
                print("Stopping motors.")
                state_msg.position[0] = 0.012345
                state_pub.publish(state_msg)
                rospy.signal_shutdown("Manual stop.")

            # Sleep to maintain the desired frequency
            rate.sleep()
            k = k+1

    except KeyboardInterrupt:
        hardware_interface.set_torque([0]*12) # Zero torques when quit
        state_msg.position[0] = 0.012345
        state_pub.publish(state_msg)
        rospy.signal_shutdown("Manual stop.")
        print("Stopping motors.")
    finally:
        hardware_interface.set_torque([0]*12) # Zero torques when quit
        print("Stopping motors.")

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("pupper_coms_node", anonymous=True, disable_signals=True)
    main() 