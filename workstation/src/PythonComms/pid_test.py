# This code is used to investigate the PID control of a single motor. 

import numpy as np

import time
from KeyboardController.JoystickInterface import JoystickInterface
from SerialInterface import HardwareInterface
from SerialInterface.IndividualConfig import SERIAL_PORT  # make the configs more consistent
from pupper_whisperer import whisperer # mathew

def main():
    """Main program"""

    # Create pupper whisperer which will provide I/O comms through gazebo node
    PupComm = whisperer()

    # Setup interfaces
    hardware_interface = HardwareInterface.HardwareInterface(port=SERIAL_PORT)
    joystick_interface = JoystickInterface() #Keyboard interface

    # Put Pupper into torque control mode (and out of error mode if it is)
    hardware_interface.set_trq_mode()

    # Zero motors
    hardware_interface.zero_motors() 
    print("Zeroing Done")
    hardware_interface.set_max_current(1.0) # - mathew

    motor_torque = 0
    
    # maintain a control frequency 
    dt = .001 # 1000hz
    now = time.time() 

    # Moving average for current print out
    k = 0
    H = 1000 # Window size (1000 = 1s)
    curr_window = np.zeros([H,1])
    vel_window = np.zeros([H,1])
    
    # Controller parameters
    w_desired = 0 #rad/s (output shaft)
    Kp = .6 # 0.6
    Ki = 1.0 # 1.0 work well for w_desired <= 6
    e_I = 0

    manual_torques = [0] * 12
    try:
        while True:
            command = joystick_interface.get_command()
            if time.time() - now >= dt:
                k = k + 1
                # print("Loop time in ms: ", (time.time() - now)*1000)
                # -------------------------------------------
                # -------- Retrieve states from motor ------- 
                # ------------------------------------------- 
                
                # Read data from pupper
                PupComm.store_robot_states(hardware_interface.get_robot_states())
                print(hardware_interface.get_robot_states())

                # Put into the correct form
                (joint_pos, joint_vel, joint_cur) = PupComm.get_joint_state("front_right_hip")

                # Store values into sliding window
                curr_window[k%H,0] = joint_cur
                vel_window[k%H,0] = joint_vel

                # Calculate statistics 
                avg_curr = np.sum(curr_window)/H
                avg_vel = np.sum(vel_window)/H
                std_vel = np.std(vel_window)

                print("------------------------------------")
                print("Velocity: {:+.5f}".format(joint_vel))
                print("Velocity mean: {:+.5f}".format(avg_vel))
                print("Velocity stdev: {:+.5f}".format(std_vel))
                print("Current avg: {:+.5f}".format(avg_curr))
                print("Current inst: {:+.5f}".format(avg_curr))
                print("Desired Current: ", motor_torque)
                print("e_I: ",e_I)
                
                # -------------------------------------------
                # -------- PD Control ----------------------- 
                # ------------------------------------------- 
                e = w_desired - joint_vel #error
                e_I = e_I + e*dt #integral error
                motor_torque = Kp*(e) + Ki*(e_I)

                #Compensate static friction
                # if abs(joint_vel) < .1:
                #     motor_torque = motor_torque + .3
                
                #Limit current
                if abs(motor_torque) > 3:
                    print("###################### MOTOR CURRENT SATURATED at 3.0 amps") 
                    motor_torque = 3 * np.sign(motor_torque)

                #End if velocity is too large
                if abs(joint_vel) > 40:
                    hardware_interface.set_torque([0]*12) # Zero torques
                    print("Velocity: ", joint_vel)
                    print("MOTOR VELOCITY > 40 rad/s") 
                    print("Previous Kp: ", Kp, " Ki: ", Ki)
                    Kp = float(input("Input Kp: "))
                    Ki = float(input("Input Ki: "))
                    e_I = 0

                # -------------------------------------------
                # -------- Send current to motor ------------ 
                # ------------------------------------------- 

                # Set desired current:
                manual_torques[9] = motor_torque #make sure limits are obeyed

                WBC_commands_reordered = PupComm.reorder_commands(manual_torques)

                #Set_torque actually sets currents
                hardware_interface.set_torque(WBC_commands_reordered)

                now = time.time() 

            # Break loop when "q" is pressed
            if command.activate_event == 1:
                hardware_interface.set_torque([0]*12) # Zero torques temporarily
                print("Previous Kp: ", Kp, " Ki: ", Ki)
                Kp = float(input("Input Kp: "))
                Ki = float(input("Input Ki: "))
                e_I = 0

            # Set w_desired when "e" is pressed
            if command.trot_event == 1:
                hardware_interface.set_torque([0]*12) # Zero torques temporarily
                w_desired = float(input("Input desired vel [rad/s]: "))

    except KeyboardInterrupt:
        hardware_interface.set_torque([0]*12) # Zero torques when quit
        print("Stopping motors.")

def summarize_config(config):
    # Print summary of configuration to console for tuning purposes
    print("Done.")

if __name__ == "__main__":
    main()