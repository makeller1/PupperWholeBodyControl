# This code is used to investigate the PID control of a single motor. 

import numpy as np
import pandas as pd

import time
from SerialInterface import HardwareInterface
from SerialInterface.IndividualConfig import SERIAL_PORT  # make the configs more consistent
from pupper_whisperer import whisperer # mathew

def main():
    """Main program"""

    N = 95003 # Total samples used in calibration (90 seconds worth)

    # Create pupper whisperer which will provide I/O comms through gazebo node
    PupComm = whisperer()

    # Setup interfaces
    hardware_interface = HardwareInterface.HardwareInterface(port=SERIAL_PORT)

    # Put Pupper into torque control mode (and out of error mode if it is)
    hardware_interface.set_trq_mode()

    # Zero motors
    hardware_interface.zero_motors() 
    print("Zeroing Done")
    
    # maintain control frequency 
    dt = .001 # 1000hz
    now = time.time()
    i = 0
    try:
        while i < 86928+1001: 
            if time.time() - now >= dt:

                hardware_interface.run_calibration()

                now = time.time()

                # Check if the pupper has faulted
                PupComm.check_errors()

                i = i + 1
                
    except KeyboardInterrupt:
        hardware_interface.set_torque([0]*6) # Zero torques when quit
        print("Stopping motors.")

if __name__ == "__main__":
    main()