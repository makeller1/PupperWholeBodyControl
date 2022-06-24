# -*- coding: utf-8 -*-
"""
Reads logs from the WBC which include the following variables

time_ms // time when WBC was called
solve_ms // time required to solve
tau_i // optimal torques (BL, BR, FL, FR)
Fr_i // optimal reaction forces

Fr_gazebo_br_i // Gazebo contact forces (bl, br, fl, fr)

Task cost
Task optimal accel
Task desired accel
Task measured pos
Task target pos

For the following tasks:
    
"COM_HEIGHT"
"COM_LATERAL_POS"
"COM_ORIENTATION"
"JOINT_ANGLES"

"BACK_LEFT_FOOT_POS"
"BACK_RIGHT_FOOT_POS"
"FRONT_LEFT_FOOT_POS"
"FRONT_RIGHT_FOOT_POS"
"""


def diagnoseTiming(df): 
    plt.figure() 
    ax1 = plt.subplot(3,1,1)
    plt.plot(t,df["update_ms"])
    plt.plot(t,df["update_ms"]+df["solve_ms"])
    plt.plot(t,df["run_ms"],linestyle = '--')
    plt.legend(['osqp update','osqp update + solve','osqp run time'],loc='upper left')
    plt.xlabel('time (ms)')
    plt.ylabel('dt (ms)')
    plt.grid()
    
    plt.subplot(3,1,2, sharex=ax1)
    plt.plot(t,df["wbc_ms"])
    plt.plot(t,df["ucj_ms"])
    plt.plot(t,df["crb_ms"])
    plt.plot(t,df["nle_ms"])
    plt.plot(t,df["fqp_ms"])
    plt.plot(t,df["sqp_ms"])
    plt.plot(t,df["pdg_ms"])
    plt.legend(['elapsed time of wbc','UpdateContactJacobian','CompositeRigidBodyAlgorithm','NonlinearEffect',
                'formQP','solveQP','printDiag'])
    plt.xlabel('time (ms)')
    plt.ylabel('dt (ms)')
    plt.grid()
    
    plt.subplot(3,1,3, sharex=ax1)
    plt.plot(t,df["log_ms"])
    plt.legend(['elapsed time spent logging'])
    plt.xlabel('time (ms)')
    plt.ylabel('dt (ms)')
    plt.grid()
    
    print("Mean run times:")
    print("wbc_ms: ",np.mean(df["wbc_ms"]))
    print("ucj_ms: ",np.mean(df["ucj_ms"]))
    print("crb_ms: ",np.mean(df["crb_ms"]))
    print("nle_ms: ",np.mean(df["nle_ms"]))
    print("fqp_ms: ",np.mean(df["fqp_ms"]))
    print("sqp_ms: ",np.mean(df["sqp_ms"]))
    print("pdg_ms: ",np.mean(df["pdg_ms"]))
    print("log_ms: ",np.mean(df["log_ms"]))
    
def diagnoseContacts(df):
    # Plot back right foot diagnostics
    plt.figure()
    ax1 = plt.subplot(3,1,1)
    plt.title("BR foot")
    plt.plot(t,df["BACK_RIGHT_FOOT_POS_acc_opt_1"])
    plt.plot(t,df["BACK_RIGHT_FOOT_POS_acc_des_1"])
    plt.legend(['opt accel x'])
    plt.grid()
    
    plt.subplot(3,1,2, sharex=ax1)
    plt.plot(t,df["Fr_4"])
    plt.plot(t,df["Fr_6"]*0.3,color='orange')
    plt.plot(t,df["Fr_6"]*-0.3,color='orange')
    plt.legend(['BR foot r.f. x','BR foot r.f. z'])
    plt.grid()
    
    plt.subplot(3,1,3, sharex=ax1)
    plt.plot(t,df["BACK_RIGHT_FOOT_POS_pos_measured_1"])
    plt.legend(['pos measured x'])
    plt.grid()
    
    # Plot front left foot diagnostics
    plt.figure()
    ax1 = plt.subplot(3,1,1)
    plt.title("FL foot")
    plt.plot(t,df["FRONT_LEFT_FOOT_POS_acc_opt_1"])
    plt.plot(t,df["FRONT_LEFT_FOOT_POS_acc_des_1"])
    plt.grid()
    plt.legend(['opt accel x'])
    
    plt.subplot(3,1,2, sharex=ax1)
    plt.plot(t,df["Fr_7"])
    plt.plot(t,df["Fr_9"]*0.3,color='orange')
    plt.plot(t,df["Fr_9"]*-0.3,color='orange')
    plt.legend(['FL foot r.f. x','FL foot r.f. z'])
    plt.grid()
    
    plt.subplot(3,1,3, sharex=ax1)
    plt.plot(t,df["FRONT_LEFT_FOOT_POS_pos_measured_1"])
    plt.legend(['pos measured x'])
    plt.grid()
    
    # Plot front right foot diagnostics
    plt.figure()
    ax1 = plt.subplot(3,1,1)
    plt.title("FR foot")
    plt.plot(t,df["FRONT_RIGHT_FOOT_POS_acc_opt_1"])
    plt.plot(t,df["FRONT_RIGHT_FOOT_POS_acc_des_1"])
    plt.legend(['opt accel x'])
    plt.grid()
    
    plt.subplot(3,1,2, sharex=ax1)
    plt.plot(t,df["Fr_10"])
    plt.plot(t,df["Fr_12"]*0.3,color='orange')
    plt.plot(t,df["Fr_12"]*-0.3,color='orange')
    plt.legend(['FR foot r.f. x','FR foot r.f. z'])
    plt.grid()
    
    plt.subplot(3,1,3, sharex=ax1)
    plt.plot(t,df["FRONT_RIGHT_FOOT_POS_pos_measured_1"])
    plt.legend(['pos measured x'])
    plt.grid()
    
def xComPosTask(df):
    plt.figure()
    ax1 = plt.subplot(2,1,1)
    plt.title('COM lateral position X')
    plt.plot(t,df["COM_LATERAL_POS_acc_opt_1"])
    plt.plot(t,df["COM_LATERAL_POS_acc_des_1"])
    plt.legend(['optimal acc','desired acc'])
    plt.grid()

    plt.subplot(2,1,2, sharex = ax1)
    plt.plot(t,df["COM_LATERAL_POS_pos_target_1"])
    plt.plot(t,df["COM_LATERAL_POS_pos_measured_1"])
    plt.legend(['pos target','pos measured'])
    plt.xlabel('time (ms)')
    plt.grid()
    
    
def simReactionForce(df):
    # Plot gazebo reaction forces against wbc optimal reaction forces 
    
    #Shift gazebo measurements back one index
    df["Fr_gazebo_bl_1"][:-1] = df["Fr_gazebo_bl_1"][1:]
    df["Fr_gazebo_bl_2"][:-1] = df["Fr_gazebo_bl_2"][1:]
    df["Fr_gazebo_bl_3"][:-1] = df["Fr_gazebo_bl_3"][1:]
    df["Fr_gazebo_br_1"][:-1] = df["Fr_gazebo_br_1"][1:]
    df["Fr_gazebo_br_2"][:-1] = df["Fr_gazebo_br_2"][1:]
    df["Fr_gazebo_br_3"][:-1] = df["Fr_gazebo_br_3"][1:]
    df["Fr_gazebo_fl_1"][:-1] = df["Fr_gazebo_fl_1"][1:]
    df["Fr_gazebo_fl_2"][:-1] = df["Fr_gazebo_fl_2"][1:]
    df["Fr_gazebo_fl_3"][:-1] = df["Fr_gazebo_fl_3"][1:]
    df["Fr_gazebo_fr_1"][:-1] = df["Fr_gazebo_fr_1"][1:]
    df["Fr_gazebo_fr_2"][:-1] = df["Fr_gazebo_fr_2"][1:]
    df["Fr_gazebo_fr_3"][:-1] = df["Fr_gazebo_fr_3"][1:]
    
    plt.figure()
    ax1 = plt.subplot(4,1,1)
    plt.title('Back left reaction forces')
    plt.plot(t,df["Fr_gazebo_bl_1"], color='blue',linestyle='--')
    plt.plot(t,df["Fr_1"], color='blue')
    plt.plot(t,df["Fr_gazebo_bl_2"], color='green',linestyle='--')
    plt.plot(t,df["Fr_2"], color='green')
    plt.plot(t,df["Fr_gazebo_bl_3"], color='orange',linestyle='--')
    plt.plot(t,df["Fr_3"], color='orange')
    plt.grid()
    plt.legend(['x gazebo','x wbc','y gazebo','y wbc','z gazebo','z wbc'])
    
    plt.subplot(4,1,2, sharex = ax1)
    plt.title('Back right reaction forces')
    plt.plot(t,df["Fr_gazebo_br_1"], color='blue',linestyle='--')
    plt.plot(t,df["Fr_4"], color='blue')
    plt.plot(t,df["Fr_gazebo_br_2"], color='green',linestyle='--')
    plt.plot(t,df["Fr_5"], color='green')
    plt.plot(t,df["Fr_gazebo_br_3"], color='orange',linestyle='--')
    plt.plot(t,df["Fr_6"], color='orange')
    plt.grid()
    plt.legend(['x gazebo','x wbc','y gazebo','y wbc','z gazebo','z wbc'])
    
    plt.subplot(4,1,3, sharex = ax1)
    plt.title('Front left reaction forces')
    plt.plot(t,df["Fr_gazebo_fl_1"], color='blue',linestyle='--')
    plt.plot(t,df["Fr_7"], color='blue')
    plt.plot(t,df["Fr_gazebo_fl_2"], color='green',linestyle='--')
    plt.plot(t,df["Fr_8"], color='green')
    plt.plot(t,df["Fr_gazebo_fl_3"], color='orange',linestyle='--')
    plt.plot(t,df["Fr_9"], color='orange')
    plt.grid()
    plt.legend(['x gazebo','x wbc','y gazebo','y wbc','z gazebo','z wbc'])
    
    plt.subplot(4,1,4, sharex = ax1)
    plt.title('Front right reaction forces')
    plt.plot(t,df["Fr_gazebo_fr_1"], color='blue',linestyle='--')
    plt.plot(t,df["Fr_10"], color='blue')
    plt.plot(t,df["Fr_gazebo_fr_2"], color='green',linestyle='--')
    plt.plot(t,df["Fr_11"], color='green')
    plt.plot(t,df["Fr_gazebo_fr_3"], color='orange',linestyle='--')
    plt.plot(t,df["Fr_12"], color='orange')
    plt.grid()
    plt.legend(['x gazebo','x wbc','y gazebo','y wbc','z gazebo','z wbc'])
    
def simReactionForceError(df):
    # Plot error between Gazebo and wbc reaction forces
    
    #Shift gazebo measurements back one index
    df["Fr_gazebo_bl_1"][:-1] = df["Fr_gazebo_bl_1"][1:]
    df["Fr_gazebo_bl_2"][:-1] = df["Fr_gazebo_bl_2"][1:]
    df["Fr_gazebo_bl_3"][:-1] = df["Fr_gazebo_bl_3"][1:]
    df["Fr_gazebo_br_1"][:-1] = df["Fr_gazebo_br_1"][1:]
    df["Fr_gazebo_br_2"][:-1] = df["Fr_gazebo_br_2"][1:]
    df["Fr_gazebo_br_3"][:-1] = df["Fr_gazebo_br_3"][1:]
    df["Fr_gazebo_fl_1"][:-1] = df["Fr_gazebo_fl_1"][1:]
    df["Fr_gazebo_fl_2"][:-1] = df["Fr_gazebo_fl_2"][1:]
    df["Fr_gazebo_fl_3"][:-1] = df["Fr_gazebo_fl_3"][1:]
    df["Fr_gazebo_fr_1"][:-1] = df["Fr_gazebo_fr_1"][1:]
    df["Fr_gazebo_fr_2"][:-1] = df["Fr_gazebo_fr_2"][1:]
    df["Fr_gazebo_fr_3"][:-1] = df["Fr_gazebo_fr_3"][1:]
    
    plt.figure()
    ax1 = plt.subplot(4,1,1)
    plt.title('Back left reaction forcce error')
    plt.plot(t,np.abs(df["Fr_1"] - df["Fr_gazebo_bl_1"]), color='blue')
    plt.plot(t,np.abs(df['Fr_2'] - df["Fr_gazebo_bl_2"]), color='green')
    plt.plot(t,np.abs(df['Fr_3'] - df["Fr_gazebo_bl_3"]), color='orange')
    plt.grid()
    plt.legend(['x error','y error', 'z error'])
    
    plt.subplot(4,1,2, sharex = ax1)
    plt.title('Back right reaction forcce error')
    plt.plot(t,np.abs(df["Fr_4"] - df["Fr_gazebo_br_1"]), color='blue')
    plt.plot(t,np.abs(df['Fr_5'] - df["Fr_gazebo_br_2"]), color='green')
    plt.plot(t,np.abs(df['Fr_6'] - df["Fr_gazebo_br_3"]), color='orange')
    plt.grid()
    plt.legend(['x error','y error', 'z error'])
    
    plt.subplot(4,1,3, sharex = ax1)
    plt.title('Front left reaction forcce error')
    plt.plot(t,np.abs(df["Fr_7"] - df["Fr_gazebo_fl_1"]), color='blue')
    plt.plot(t,np.abs(df['Fr_8'] - df["Fr_gazebo_fl_2"]), color='green')
    plt.plot(t,np.abs(df['Fr_9'] - df["Fr_gazebo_fl_3"]), color='orange')
    plt.grid()
    plt.legend(['x error','y error', 'z error'])
    
    plt.subplot(4,1,4, sharex = ax1)
    plt.title('Front right reaction forcce error')
    plt.plot(t,np.abs(df["Fr_10"] - df["Fr_gazebo_fr_1"]), color='blue')
    plt.plot(t,np.abs(df['Fr_11'] - df["Fr_gazebo_fr_2"]), color='green')
    plt.plot(t,np.abs(df['Fr_12'] - df["Fr_gazebo_fr_3"]), color='orange')
    plt.grid()
    plt.legend(['x error','y error', 'z error'])
    
def diagnoseOptimalTorques(df):
    plt.figure()
    ax1 = plt.subplot(4,1,1)
    plt.title('Back left leg')
    plt.plot(t,df["tau_1"])
    plt.plot(t,df["tau_2"])
    plt.plot(t,df["tau_3"])
    plt.xlabel('time (ms)')
    plt.ylabel('Torque (Nm)')
    plt.legend(['hip','shoulder','knee'])
    plt.grid()

    plt.subplot(4,1,2, sharex = ax1)
    plt.title('Back right leg')
    plt.plot(t,df["tau_4"])
    plt.plot(t,df["tau_5"])
    plt.plot(t,df["tau_6"])
    plt.xlabel('time (ms)')
    plt.ylabel('Torque (Nm)')
    plt.legend(['hip','shoulder','knee'])
    plt.grid()

    plt.subplot(4,1,3, sharex = ax1)
    plt.title('Front left leg')
    plt.plot(t,df["tau_7"])
    plt.plot(t,df["tau_8"])
    plt.plot(t,df["tau_9"])
    plt.xlabel('time (ms)')
    plt.ylabel('Torque (Nm)')
    plt.legend(['hip','shoulder','knee'])
    plt.grid()

    plt.subplot(4,1,4, sharex = ax1)
    plt.title('Front right leg')
    plt.plot(t,df["tau_10"])
    plt.plot(t,df["tau_11"])
    plt.plot(t,df["tau_12"])
    plt.xlabel('time (ms)')
    plt.ylabel('Torque (Nm)')
    plt.legend(['hip','shoulder','knee'])
    plt.grid()


    
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


file_name = 'out'
# file_name = 'out_hardware_rest'
# file_name = 'out_without_j_dot'
# file_name = 'out_with_j_dot'
# file_name = 'out_300hz_jdot'
# file_name = 'out_300hz'
# file_name = 'out_no_LP'
# file_name = 'out_good'

df = pd.DataFrame()
with np.load(file_name+'.npz') as data:
    for name in list(data.keys()):
        df[name] = np.array(data[name],dtype='float')
        

df = df.copy()
t = df["time_ms"]
t = t-t[0]
dt = np.append(np.diff(t),1)

plt.close('all')

'''Diagnose timing'''
diagnoseTiming(df)

'''Plot solve codes'''
plt.figure()
plt.plot(t,df["solve_code"])

'''Diagnose contact tasks'''
# diagnoseContacts(df)

'''Lateral position tracking '''
xComPosTask(df)

'''Angular velocities'''
# plt.figure()
# plt.plot(t,df["ang_vel_2"])

# plt.figure()
# plt.plot(t)

'''Compare gazebo reaction forces to wbc'''
# simReactionForce(df)

'''Plot reaction force error between wbc and gazebo'''
# simReactionForceError(df)

'''Optimal motor torques'''
diagnoseOptimalTorques(df)

'''Time between wbc calls'''
plt.figure()
plt.plot(t,dt)


'''Diagnose ori task'''
plt.figure()
ax1 = plt.subplot(3,1,1)
plt.title("X ori")
plt.plot(t,df["ori_d_1"])
plt.plot(t,df["ori_p_1"])
plt.legend(['d term', 'p term'])
plt.grid()

plt.subplot(3,1,2, sharex=ax1)
plt.title("Y ori")
plt.plot(t,df["ori_d_2"])
plt.plot(t,df["ori_p_2"])
plt.legend(['d term', 'p term'])
plt.grid()

plt.subplot(3,1,3, sharex=ax1)
plt.title("Z ori")
plt.plot(t,df["ori_d_3"])
plt.plot(t,df["ori_p_3"])
plt.legend(['d term', 'p term'])
plt.grid()


'''j_dot diagnostics'''
# # # Plot jdot terms for back left leg z acceleration
# linestyle = '-'

# plt.figure()
# ax1 = plt.subplot(2,1,1)
# plt.plot(t,df["j_dot_bl_z_4"],color='blue',linestyle=linestyle)
# plt.plot(t,df["j_dot_bl_z_5"],color='orange',linestyle=linestyle)
# plt.plot(t,df["j_dot_bl_z_6"],color='red',linestyle=linestyle)
# if linestyle == '-':
#     plt.legend(['j(w_x)','j(w_y)','j(w_z)'])
# plt.xlabel('time (ms)')
# plt.title("BL foot j_dot z terms")

# plt.subplot(2,1,2,sharex = ax1)
# plt.plot(t,df["j_dot_bl_z_7"],color='blue',linestyle=linestyle)
# plt.plot(t,df["j_dot_bl_z_8"],color='orange',linestyle=linestyle)
# plt.plot(t,df["j_dot_bl_z_9"],color='red',linestyle=linestyle)
# plt.xlabel('time (ms)')
# if linestyle == '-':
#     plt.legend(['j(d hip)','j(d shoulder)','j(d elbow)'])


