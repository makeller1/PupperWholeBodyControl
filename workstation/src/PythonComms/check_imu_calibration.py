# This code is used to check the calibration of the magnetometer and gyroscope of the IMU.

from cmath import pi
import time
from SerialInterface import HardwareInterface
from SerialInterface.IndividualConfig import SERIAL_PORT  # make the configs more consistent
from pupper_whisperer import whisperer # mathew
from scipy.spatial.transform import Rotation 
import numpy as np

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

    for i in range(5):
        while PupComm.store_robot_states(hardware_interface.get_robot_states(), False) is False:
            continue

    # Get initial orientation
    quaternion_init = PupComm.get_pupper_orientation()

    # Zero motors
    hardware_interface.zero_motors() 
    print("Zeroing Done")

    k = 0 # loop counter
    try:
        while True:
            # Read data from pupper
            new_measurements = PupComm.store_robot_states(hardware_interface.get_robot_states(), False)
            
            if new_measurements:

                # Check if the pupper has faulted
                PupComm.check_errors(False)

                # Print Roll-Pitch-Yaw
                quaternion = PupComm.get_pupper_orientation()
                if k%200 == 0:
                    quat_rel = quaternion_relative(quaternion_init,quaternion) #relative quaternion
                    r = Rotation.from_quat([quat_rel[1],quat_rel[2],quat_rel[3],quat_rel[0]])
                    rv = r.as_rotvec()
                    rv_angle = np.linalg.norm(rv)
                    print("deg: ", rv_angle*180/pi*-np.sign(rv[0]))
                    print("Quaternion: {:.2f} {:.2f} {:.2f} {:.2f}".format(quat_rel[0], quat_rel[1], quat_rel[2], quat_rel[3]))
                    #error3d is used in WBC to determine angular accelerations (x, y, z)
                    error3d = 1000.0 * np.array([quat_rel[1],quat_rel[2],quat_rel[3]])*np.sign(quat_rel[0])
                    print("Error 3d: {:.2f} {:.2f} {:.2f}".format(error3d[0], error3d[1], error3d[2]))
                    euler = r.as_euler('xyz', degrees=True)
                    print("Roll {:.2f}, Pitch: {:.2f}, Yaw: {:.2f}".format(euler[0],euler[1],euler[2]))
                k = k+1

    except KeyboardInterrupt:
        hardware_interface.set_torque([0]*12) # Zero torques when quit
        print("Stopping motors.")

if __name__ == "__main__":
    main()