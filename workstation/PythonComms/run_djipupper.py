#!/usr/bin/env python
################ FOR ROS #######################
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
################################################

import time
from src.Controller import Controller
from src.JoystickInterface import JoystickInterface
from src.State import State
from djipupper import HardwareInterface
from djipupper.IndividualConfig import SERIAL_PORT  # make the configs more consistent
from djipupper.Config import Configuration
from djipupper.Kinematics import four_legs_inverse_kinematics
from pupper_whisperer import whisperer # mathew

import argparse

import datetime
import os
import msgpack

DIRECTORY = os.path.dirname(os.path.realpath(__file__)) + "/logs/"
FILE_DESCRIPTOR = "log_file"

class Data:
    def __init__(self):
        # Create a list to hold the joint commands to send to the pupper
        self.commands = [0.0]*12


def main(FLAGS):
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
    
    # Create the subscriber and publisher
    state_pub   = rospy.Publisher("pupper_state", JointState, queue_size=1)
    pose_pub    = rospy.Publisher("pupper_pose", Pose, queue_size=1)
    command_sub = rospy.Subscriber("pupper_commands", Float64MultiArray, commandCallback, queue_size=1)

    # Run at 1000 Hz
    rate = rospy.Rate(1000)

    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface.HardwareInterface(port=SERIAL_PORT)

    # Create controller and user input handles
    controller = Controller(config, four_legs_inverse_kinematics)
    state = State(height=config.default_z_ref)
    print("Creating joystick listener...", end="")
    joystick_interface = JoystickInterface(config)
    print("Done.")

    summarize_config(config)

    if FLAGS.log:
        #today_string = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        filename = os.path.join(DIRECTORY, FILE_DESCRIPTOR + ".csv")
        log_file = open(filename, "w")
        hardware_interface.write_logfile_header(log_file)

    if FLAGS.zero:
        # hardware_interface.set_joint_space_parameters(0, 4.0, 4.0)
        hardware_interface.set_joint_space_parameters(
            0, 4.0, 0.2
        )  # mathew (changed max current to .2)
        hardware_interface.set_actuator_postions(np.zeros((3, 4)))
        input(
            "Do you REALLY want to calibrate? Press enter to continue or ctrl-c to quit."
        )
        print("Zeroing motors...", end="")
        hardware_interface.zero_motors()
        hardware_interface.set_max_current_from_file()
        print("Done.")

    # The zero position is set with pupper laying down with elbows back. 
    # Follow this procedure: 1. Lay pupper flat
    #                        2. Rotate hip to raise leg
    #                        3. Rotate elbow to raise foot off ground
    #                        4. Rotate hip to lower leg until end of motor touches ground
    #                        5. Rotate elbow until foot touches ground (the motor should 
    #                           be rubbing against ground if done correctly)
    # If done correctly, repeatability is < 1 degree
    input("Press enter to ZERO MOTORS") # - mathew    

    hardware_interface.zero_motors() 
    print("Zeroing Done")
    hardware_interface.set_max_current_from_file() # - mathew
    
# //////////////////////////// PD CONTROL ////////////////////////////////////////
    input("Press enter to stand. MAKE SURE PUPPER IS LAYING DOWN ELBOWS BACK.")
    print("Press q to start WBC")
    
    hardware_interface.set_joint_space_parameters(0, 0, 7.0) #Set max current
    PD_Kp = .6 # Gains on position error
    PD_Kd = .1 # Gains on velocity error
    PD_joint_pos = [0] * 12
    PD_joint_vel = [0] * 12
    PD_torque = [0] * 12
    PD_des_pos  = [0, np.pi/4, np.pi/2,
                   0,-np.pi/4,-np.pi/2,
                   0, np.pi/4, np.pi/2,
                   0,-np.pi/4,-np.pi/2]

    while state.activation == 0:
        # print(time.time())
        command = joystick_interface.get_command(state)
        #Get joint states
        PupComm.store_robot_states(hardware_interface.get_robot_states())
        # Put into the correct form for us
        for i, name in enumerate(PupComm.joint_name_map.keys()):
            (joint_pos, joint_vel, _) = PupComm.get_joint_state(name)
            PD_joint_pos[i] = joint_pos
            PD_joint_vel[i] = joint_vel
        # Calculate PD torque
        for i in range(12):
            if (i%3 == 0):
                PD_torque[i] = PD_Kp * (PD_des_pos[i] -  PD_joint_pos[i]) + PD_Kd * (-PD_joint_vel[i])
                print("Joint Velocity ", i, ": ", PD_joint_vel[i])
                print("Pos error ", i, ": ", (PD_des_pos[i] -  PD_joint_pos[i]))
            else:
                PD_torque[i] = 0
        
        print(   "PD torque[0]: {:+.3f}".format(PD_torque[0]),
                " PD torque[3]: {:+.3f}".format(PD_torque[3]),
                " PD torque[6]: {:+.3f}".format(PD_torque[6]),
                " PD torque[9]: {:+.3f}".format(PD_torque[9]))
        PD_torque_reordered = PupComm.reorder_torques(PD_torque)
        hardware_interface.set_torque(PD_torque_reordered)

        # Break loop when "q" is pressed
        if command.activate_event == 1:
            print("WBC STARTED. Press q to stop motors.")
            break
    # ///////////////////////////////////////////////////////////////////////////

    # stand_position = np.array(  [[      0,       0,      0,       0],
    #                              [np.pi/4,-np.pi/4,np.pi/4,-np.pi/4],
    #                              [np.pi/2,-np.pi/2,np.pi/2,-np.pi/2]])
    # hardware_interface.set_actuator_postions(stand_position)
    # command = joystick_interface.get_command(state)
    # controller.run(state, command)
    
    # hardware_interface.set_max_current_from_file() # Uses HardwareConfig.py MAX_CURRENT
    hardware_interface.set_joint_space_parameters(0, 0, 7.0) #Set max current #5.0
    state.activation = 1 # Start automatically
    try:
        while not rospy.is_shutdown():
            if state.activation == 0:
                time.sleep(0.02)
                joystick_interface.set_color(config.ps4_deactivated_color)
                command = joystick_interface.get_command(state)
                if command.activate_event == 1:
                    print("Robot activated.")
                    joystick_interface.set_color(config.ps4_color)
                    hardware_interface.serial_handle.reset_input_buffer()
                    hardware_interface.activate()
                    state.activation = 1
                    continue
            elif state.activation == 1:
                command = joystick_interface.get_command(state)
                # controller.run(state, command)
                
                #Printing states in function below
                # if(True):
                #     # --------------------- Make sure we're reading info from pupper ---------------------------
                #     print("front right hip : {:+.5f}".format(PupComm.get_joint_state("front_right_hip")[0]), "  "
                #           "front right shoulder : {:+.5f}".format(PupComm.get_joint_state("front_right_shoulder")[0]), "  "
                #           "front right elbow : {:+.5f}".format(PupComm.get_joint_state("front_right_elbow")[0]))

                #     print("back left hip : {:+.5f}".format(PupComm.get_joint_state("back_left_hip")[0]), "  "
                #           "back left shoulder : {:+.5f}".format(PupComm.get_joint_state("back_left_shoulder")[0]), "  "
                #           "back left elbow : {:+.5f}".format(PupComm.get_joint_state("back_left_elbow")[0]))

                #     print("back right hip : {:+.5f}".format(PupComm.get_joint_state("back_right_hip")[0]), "  "
                #           "back right shoulder : {:+.5f}".format(PupComm.get_joint_state("back_right_shoulder")[0]), "  "
                #           "back right elbow : {:+.5f}".format(PupComm.get_joint_state("back_right_elbow")[0]))
                    
                #     print("front left hip : {:+.5f}".format(PupComm.get_joint_state("front_left_hip")[0]), "  "
                #           "front left shoulder : {:+.5f}".format(PupComm.get_joint_state("front_left_shoulder")[0]), "  "
                #           "front left elbow : {:+.5f}".format(PupComm.get_joint_state("front_left_elbow")[0]))
                #     # ------------------------------------------- mathew
                #     dummy = 1
                
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

                # print("Quaternion: ", quaternion)    

                #Send Orientation to the C++ node
                pose_pub.publish(pose_msg)

                # Read torque command from ROS
                WBC_commands_reordered = PupComm.reorder_torques(data.commands)

                # -------------------------------------------
                # -------- Send torques to pupper ---------- 
                # ------------------------------------------- 
                # WBC_commands_reordered is a list with order: 
                # [FR hip, FR shoulder, FR knee,
                #  FL hip, FL shoulder, FL knee,
                #  BR hip, BR shoulder, BR knee,
                #  BL hip, BL shoulder, BL knee]

                # For testing, set desired torque:
                # manual_torques = [0] * 12
                # manual_torques[2] = 1 #make sure limits are obeyed
                # WBC_commands_reordered = PupComm.reorder_torques(manual_torques)

                # Scaling factors found in C610.cpp in Stanford's code
                # print("looped")
                for i in range(12):
                    # Convert torque to current
                    torque_cmd = WBC_commands_reordered[i]
                    vel        = PupComm.robot_states_["vel"][i]
                    # if vel * torque_cmd <= 0:
                    WBC_commands_reordered[i] = 1/0.308 * (torque_cmd + 0.0673 * np.sign(vel) + .00277 * vel) 
                    # else:
                    # WBC_commands_reordered[i] = 1/0.179 * (torque_cmd + 0.0136 * np.sign(vel) + 0.00494 * vel)
                        # print("Motor ", i, " B")
                    # print("Curent ", i, ": {:+.3f}".format(WBC_commands_reordered[i]))
                    # if vel * torque_cmd > 0:
                    #     print("B")
                #Set_torque actually sets currents
                hardware_interface.set_torque(WBC_commands_reordered) # FR, FL, BR, BL
                
                # ------------------------------------------- 

                ## Commented below - 
                # hardware_interface.set_cartesian_positions(
                #     state.final_foot_locations
                # )

                if FLAGS.log:
                    any_data = hardware_interface.log_incoming_data(log_file)
                    if any_data:
                        print(any_data["ts"])

                # if command.activate_event == 1:
                #     print("Deactivating Robot")
                #     print("Waiting for L1 to activate robot.")
                #     hardware_interface.deactivate()
                #     state.activation = 0
                #     continue
                
                # Sleep to maintain the desired frequency
                rate.sleep()

                # Break loop when "q" is pressed
                if command.activate_event == 1:
                    hardware_interface.set_torque([0]*12)
                    print("Stopping motors.")
                    break

    except KeyboardInterrupt:
        if FLAGS.log:
            print("Closing log file")
            log_file.close()


def summarize_config(config):
    # Print summary of configuration to console for tuning purposes
    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("default height: ", config.default_z_ref)
    print("x shift: ", config.x_shift)


if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("pupper_coms_node", anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument("--zero", help="zero the motors", action="store_true")
    parser.add_argument("--log", help="log pupper data to file", action="store_true")
    FLAGS = parser.parse_args()
    main(FLAGS)


                