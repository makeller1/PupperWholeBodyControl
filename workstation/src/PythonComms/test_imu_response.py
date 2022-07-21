'''This code is used to evaluate the performance of the AHRS by comparing the IMU 
   estimated angle with the measured encoder angle while the IMU is fixed to the motor shaft.
'''
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

# def main():
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
    while PupComm.store_robot_states(hardware_interface.get_robot_states(), False) is False:
        continue
# Get initial orientation
quaternion_init = PupComm.get_pupper_orientation()
quaternion_last = PupComm.get_pupper_orientation()

# Zero motors
hardware_interface.zero_motors() 
print("Zeroing Done")

# Test settings
theta_max = 40/2
freq_hz = 1.0
dt = .001 # 1000hz
k = 0 # loop counter
n = 1000
tf = 10 # seconds

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
results = pd.DataFrame(columns=["time (ms)","encoder (deg)", "velocity (rad/s)","imu_z (deg)","imu_y (deg)","imu_x (deg)"], dtype = float)
df_omega = pd.DataFrame(columns=["time (ms)","w0","w1","w2", "n0","n1","n2"], dtype = float)
N = int(tf*2000)
seq = np.zeros(N)
try:
    while time.time() - t_init < tf:
        
        # Read data from pupper
        new_measurements = PupComm.store_robot_states(hardware_interface.get_robot_states())
        
        if new_measurements: # and time.time() - now >= dt:
            # Retrieve message number
            seq[k] = PupComm.robot_states_['seq']
            # Retrieve encoder data
            joint_pos = -PupComm.robot_states_['pos'][0]*180/pi # (deg)
            joint_vel = -PupComm.robot_states_['vel'][0]*180/pi # (deg/s)
            
            # Read IMU and compute angle
            quat_dot = PupComm.robot_states_['quatdot']
            quaternion = PupComm.get_pupper_orientation()
            
            #Compute angular velocity from quaternion derivative
            ang_vel = 2*quaternion_relative(quaternion,quat_dot)[1:]
            #Compute angular velocity numerically 
            dt = time.time() - now
            quat_diff = np.array(quaternion_relative(quaternion_last,quaternion))
            ang_vel_num = 2*np.sign(quat_diff[0])*quat_diff[1:]/dt
            now = time.time()
            quaternion_last = quaternion
            
            #Compute euler angles
            quat_rel = quaternion_relative(quaternion_init,quaternion) #relative quaternion
            r = Rotation.from_quat([quat_rel[1],quat_rel[2],quat_rel[3],quat_rel[0]])
            euler = r.as_euler('xyz', degrees=True)
            angle_z = euler[2]
            angle_y = euler[1]
            angle_x = euler[0]
            
            #Store values
            t_ms = (time.time() - t_init)*1000
            results.loc[k] = [t_ms, joint_pos, joint_vel/180*pi, angle_z, angle_y, angle_x]
            df_omega.loc[k] = [t_ms, ang_vel[0], ang_vel[1], ang_vel[2], ang_vel_num[0], ang_vel_num[1], ang_vel_num[2]]
            
            #Calculate PID control input
            # pos_des = theta_max*np.cos(t_ms / 1000.0 * 2*pi * freq_hz) # deg
            vel_des = -(2*pi * freq_hz)*theta_max*np.sin(t_ms / 1000.0 * 2*pi * freq_hz) # deg/s
            
            e = vel_des - joint_vel
            e_d = (alpha)*e_d -(1-alpha)*(e-e_prev)
            e_i = e_i + e*dt
            torque = -(Kp*e + Kd*e_d + Ki*e_i)

            current_commands[0] = trq_to_current(torque, np.sign(torque), np.sign(torque)*abs(joint_vel)/180.0*pi)
            # current_commands = [pi/100]*12
            hardware_interface.set_torque(current_commands)

            e_prev = e 
            k = k+1

    hardware_interface.set_torque([0]*12)
    hardware_interface.serial_handle.close()
    
    # Remove unused memory
    seq = np.delete(seq,np.arange(k,N,1))

    # Normalize to first value
    seq = seq - seq[0]
    
    plt.figure(5)
    plt.scatter(np.arange(0,seq.size-1,1),np.diff(seq)-1)
    plt.title("Delta sequence")
    plt.ylabel("Messages lost")
    
    plt.figure(1)
    ax1 = plt.subplot(3,1,1)
    plt.plot(results["time (ms)"],results["imu_x (deg)"])
    plt.grid()
    plt.xlabel("Time (ms)")
    plt.ylabel("Angle (deg)")
    plt.legend(["Encoder x"])
    
    plt.subplot(3,1,2,sharex=ax1)
    plt.plot(results["time (ms)"],results["imu_y (deg)"])
    plt.grid()
    plt.xlabel("Time (ms)")
    plt.ylabel("Angle (deg)")
    plt.legend(["Encoder y"])
    
    plt.subplot(3,1,3,sharex=ax1)
    plt.plot(results["time (ms)"],results["encoder (deg)"])
    plt.plot(results["time (ms)"],results["imu_z (deg)"])
    plt.grid()
    plt.xlabel("Time (ms)")
    plt.ylabel("Angle (deg)")
    plt.legend(["Encoder","IMU"])
    
    #Angular velocity
    plt.figure(2)
    ax1 = plt.subplot(3,1,1)
    plt.grid()
    plt.title("Angular velocity")
    plt.plot(df_omega["time (ms)"],df_omega["w0"])
    plt.plot(df_omega["time (ms)"],df_omega["n0"])
    plt.ylabel("omega_x (rad/s)")
    
    plt.subplot(3,1,2,sharex=ax1)
    plt.grid()
    plt.plot(df_omega["time (ms)"],df_omega["w1"])
    plt.plot(df_omega["time (ms)"],df_omega["n1"])
    plt.ylabel("omega_y (rad/s)")
    
    plt.subplot(3,1,3,sharex=ax1)
    plt.grid()
    plt.plot(df_omega["time (ms)"],df_omega["w2"])
    plt.plot(df_omega["time (ms)"],df_omega["n2"])
    plt.plot(results["time (ms)"],results["velocity (rad/s)"])
    plt.legend(["madgwick","numerical","encoder"])
    plt.xlabel("Time (ms)")
    plt.ylabel("omega_z (rad/s)")
    # #Characterize noise
    # plt.figure(3)
    # ax1 = plt.subplot(3,1,1)
    # plt.plot(results["time (ms)"],results["imu_z (deg)"])
    # plt.subplot(3,1,2,sharex=ax1)
    # plt.plot(results["time (ms)"],results["imu_y (deg)"])
    # plt.subplot(3,1,3,sharex=ax1)
    # plt.plot(results["time (ms)"],results["imu_x (deg)"])
    
except:
    hardware_interface.set_torque([0]*12) # Zero torques when quit
    hardware_interface.serial_handle.close()
    print("Stopping motors.")
    

# if __name__ == "__main__":
#     main()