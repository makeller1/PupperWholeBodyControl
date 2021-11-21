'''
This code is used to convert torque to current. 
Inputs: trq_desired - Desired torque (Nm)
        q_ddot_des - Desired acceleration direction (1 or -1)
        joint_vel - Angular velocity of the motor (rad/s)
        Note: All values must be in the same reference system

Output: i_cmd - current to be commanded to the C610 motor controllers (A)

Steps for accurate torque control
1. Determine desired output torque
2. Determine back or forward driving
3. Convert desired output torque to current with torque constant according to step 2
4. Add compensating current i_c = f_k(w) to current from step 3
5. If w near 0, i_c = i_b*sgn(q_dot_dot), to compensate for static friction, where i_b is the breakaway current

Note: The desired velocity is required for compensating static friction in the correct direction. We can use the desired
      acceleration q_dot_dot from the WBC solution to determine the direction of desired velocity at rest.
'''

import numpy as np

def trq_to_current(trq_desired,q_ddot_des,joint_vel):
    # Identified motor parameters
    Kt_pos = .1834 # Torque constant forward driving (Nm/A)
    Kt_neg = .3030 # Torque constant backdriving (Nm/A)
    i_s = 0.26 # Static friction current compensation (A) 
    i_damping = .04 # Amount current is reduced for stability (A)
                    # (currently set to std(i_k) or 1 std dev of compensation current) 

    # -------------------------------------------
    # ------- -DETERMINE DESIRED CURRENT --------
    # ------------------------------------------- 
    if np.abs(joint_vel) > .1:
        #backdriving (-1) or forward driving (1):
        drive_mode = np.sign(joint_vel)*np.sign(trq_desired)
    else:
        #Special case when motor is at rest, we use desired acceleration direction
        drive_mode = np.sign(q_ddot_des)*np.sign(trq_desired)

    #Convert torque to current
    if drive_mode >= 0: 
        #Forward driving
        i_trq = trq_desired / Kt_pos 
    else:
        #Backdriving
        i_trq = trq_desired / Kt_neg

    if abs(joint_vel) < .1: 
        # Compensate static friction
        # Discontinuous
        i_c = i_s * np.sign(q_ddot_des) #Compensation current

        # #Continuous (experimental)
        # x0 = 0
        # x1 = .2
        # y0 = .287
        # y1 = .12788
        # m0 = (y1-y0)/(x1-x0)
        # b = y0
        # i_s = (m0*joint_vel + b) * np.sign(q_ddot_des)
    else:
        # Compensate kinetic friction
        i_c = kin_friction(joint_vel, i_damping, drive_mode) #Compensation current

    # Calculate total current
    i_cmd = i_trq + i_c # Current command (A)
    return i_cmd

def kin_friction(w, i_damping, drive_mode):
    # Piece-wise linear approximation of kinetic friction as a function of velocity
    # Inputs: w: joint velocity (rad/s)
    #         i_damping: amount compensation current is reduced for stability (A)
    # Outputs: i_k: the current necessary to compensate kinetic friction (A)
    # Note: The term i_damping reduces compensation to prevent instability i.e. velocity blowing up
    #       (i_damping = stddev(i_k), 1 std dev of compensation current seems to work well)

    dir = np.sign(w)
    w = np.abs(w) 

    m0 = .0294
    m1 = .0136
    m2 = .0102
    b =  .1220
    w1 = 5
    w2 = 19
    w3 = 30

    if w <= w1:
        i_k = m0*w + b
    elif w <= w2:
        i_k = m0*w1 + m1*(w-w1) + b
    elif w > w2:
        i_k = m0*w1 + m1*(w2-w1) + m2*(w-w2) + b

    # Add damping term at higher velocities to prevent chance of velocity blowing up (since kinetic friction is stochastic)
    if w >= .1:
        i_k = i_k - i_damping
    # Reduce compensation in backdrive mode (determined experimentally)
    if w >= .1 and drive_mode == -1:
        i_k = i_k - .02

    return i_k * dir