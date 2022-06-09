# This code is used to calibrate the magnetometer and gyroscope of the IMU.
# Procedure:
#  1. Start with pupper at rest. Gyroscope bias will be determined now. (5 seconds)
#  2. After medium beep, trace the surface of a sphere by rotating the pupper. Magnetometer additive bias will be determined now. (45 seconds)
#  3. After short beep, repeat step 2. Magnetometer multiplicative bias will be determined now. (45 seconds)
#  4. After a long beep, calibration is complete. 
# Note: Teensy must be restarted to run Calibration more than once 

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

    # Set max current for joint regulation during calibration
    hardware_interface.set_max_current(3.0) # Saturation (not fault)

    # Put Pupper into torque control mode (and out of error mode if it is)
    hardware_interface.set_trq_mode()

    # Zero motors
    hardware_interface.zero_motors() 
    print("Zeroing Done")
    
    # maintain communication frequency 
    dt = .001 # 1000hz
    now = time.time()
    i = 0
    try:
        while i < 86928+1001: # Number of samples required for calibration
            if time.time() - now >= dt:

                hardware_interface.run_calibration()

                now = time.time()

                # Check if the pupper has faulted
                PupComm.check_errors()

                i = i + 1
                
    except KeyboardInterrupt:
        hardware_interface.set_torque([0]*12) # Zero torques when quit
        print("Stopping motors.")

if __name__ == "__main__":
    main()