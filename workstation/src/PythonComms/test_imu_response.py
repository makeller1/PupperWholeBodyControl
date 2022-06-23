# This code is used to check the calibration of the magnetometer and gyroscope of the IMU.

from cmath import pi
import time
from SerialInterface import HardwareInterface
from SerialInterface.IndividualConfig import SERIAL_PORT  # make the configs more consistent
from pupper_whisperer import whisperer # mathew
from scipy.spatial.transform import Rotation 
import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt
from utilities import trq_to_current

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

def main():
    """Main program"""

    # Create pupper whisperer which will provide I/O comms through gazebo node
    PupComm = whisperer()

    # Setup interfaces
    hardware_interface = HardwareInterface.HardwareInterface(port=SERIAL_PORT)

    # Set max current for joint regulation during calibration
    hardware_interface.set_max_current(3.0) # Saturation (not fault)

    # Put Pupper into torque control mode (and out of error mode if it is)
    hardware_interface.set_trq_mode()

    # Flush first few messages of serial to receive initial orientation measurement
    for i in range(5):
        PupComm.store_robot_states(hardware_interface.get_robot_states())
        time.sleep(.002)

    # Get initial orientation
    quaternion_init = PupComm.get_pupper_orientation()

    # Zero motors
    hardware_interface.zero_motors() 
    print("Zeroing Done")

    time.sleep(0.5)

    # Test settings
    theta_max = 40/2
    freq_hz = 2.0
    dt = .001 # 1000hz
    k = 0 # loop counter
    n = 1000

    # PID settings 0.02, 0.01
    Kp = 0.006
    Kd = 0.0 
    Ki = 0
    alpha = 0.87 # Low pass filter coefficient for derivative of error

    # PID variables
    e_i = 0.0 
    e_d = 0.0
    current_commands = [0.0]*12
    pos_des = 0.0
    vel_des = 0.0
    e_prev = 0.0
    now = time.time()
    t_init = time.time()

    # Dataframe for recording results of test
    results = pd.DataFrame(columns=["time (ms)","encoder (deg)","imu (deg)"])
    try:
        while k < n:#2500:
            if time.time() - now >= dt:
                now = time.time()

                # Check if the pupper has faulted
                PupComm.check_errors(False)

                # Read data from pupper
                PupComm.store_robot_states(hardware_interface.get_robot_states())

                # Retrieve encoder data
                joint_pos = -PupComm.robot_states_['pos'][0]*180/pi # (deg)
                joint_vel = -PupComm.robot_states_['vel'][0]*180/pi # (deg/s)

                # Read IMU and compute angle
                quaternion = PupComm.get_pupper_orientation()
                quat_rel = quaternion_relative(quaternion_init,quaternion) #relative quaternion
                r = Rotation.from_quat([quat_rel[1],quat_rel[2],quat_rel[3],quat_rel[0]])
                euler = r.as_euler('xyz', degrees=True)
                angle = euler[2]

                if k%20 == 0:
                    # print("Desired torques : "+' '.join('{:.2f}'.format(f) for f in WBC_trq_reordered))
                    print("deg: ", angle)
                    print("pos_des: {:.2f}  pos_meas: {:.2f}".format(pos_des, joint_pos))
                    print("vel_des: {:.2f}  vel_meas: {:.2f}".format(vel_des, joint_vel))

                #Store values
                t_ms = (time.time() - t_init)*1000
                results.loc[k] = [t_ms, joint_pos, angle]
                
                #Calculate PID control input
                pos_des = theta_max*np.cos(t_ms / 1000.0 * 2*pi * freq_hz) # deg
                vel_des = -(2*pi * freq_hz)*theta_max*np.sin(t_ms / 1000.0 * 2*pi * freq_hz) # deg/s
                
                e = vel_des - joint_vel
                e_d = (alpha)*e_d -(1-alpha)*(e-e_prev)
                e_i = e_i + e*dt
                torque = -(Kp*e + Kd*e_d + Ki*e_i)

                current_commands[0] = trq_to_current(torque, np.sign(torque), np.sign(torque)*abs(joint_vel)/180.0*pi)
                hardware_interface.set_torque(current_commands)
                if current_commands[0] >= 10:
                    print("SATURATED")

                e_prev = e 
                k = k+1

        hardware_interface.set_torque([0]*12)

        plt.plot(results["time (ms)"],results["encoder (deg)"])
        plt.plot(results["time (ms)"],results["imu (deg)"])
        plt.grid()
        plt.xlabel("Time (ms)")
        plt.ylabel("Angle (deg)")
        plt.legend(["Encoder","IMU"])
        plt.show()
        # plt.savefig('foo.png')

    except KeyboardInterrupt:
        hardware_interface.set_torque([0]*12) # Zero torques when quit
        print("Stopping motors.")
        

if __name__ == "__main__":
    main()