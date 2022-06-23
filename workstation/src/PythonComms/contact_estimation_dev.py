# This code is used to check the calibration of the magnetometer and gyroscope of the IMU.

from cmath import pi
import time
from SerialInterface import HardwareInterface
from SerialInterface.IndividualConfig import SERIAL_PORT  # make the configs more consistent
from pupper_whisperer import whisperer # mathew
import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt
from utilities import trq_to_current

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

    # Zero motors
    hardware_interface.zero_motors() 
    print("Zeroing Done")

    time.sleep(0.5)

    # Test settings
    cur_des = 0.5 #A
    dt = .001 # 1000hz

    current_commands = [0.0]*12
    now = time.time()
    k = 0
    try:
        while True:#2500:
            if time.time() - now >= dt:
                now = time.time()

                # Check if the pupper has faulted
                PupComm.check_errors(True)

                # Read data from pupper
                PupComm.store_robot_states(hardware_interface.get_robot_states())

                # Retrieve drive data
                cur_meas = -PupComm.robot_states_['cur'][0] #(A)

                if k%20 == 0:
                    # print("Desired torques : "+' '.join('{:.2f}'.format(f) for f in WBC_trq_reordered))
                    print("cur_des: {:.2f}  cur_meas: {:.2f}".format(cur_des, cur_meas))

                current_commands[0] = -cur_des
                hardware_interface.set_torque(current_commands)
                k += 1

    except KeyboardInterrupt:
        hardware_interface.set_torque([0]*12) # Zero torques when quit
        print("Stopping motors.")

if __name__ == "__main__":
    main()