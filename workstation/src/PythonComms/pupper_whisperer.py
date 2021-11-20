"""
Class used to communicate with pupper through run_djipupper.py
It transforms the order and sign of current/torque lists to be compatible between 
WBC and low-level motor control

Note the WBC uses the order
    "back_left_hip":         0,
    "back_left_shoulder":    1, 
    "back_left_elbow":       2,
    "back_right_hip":        3,
    "back_right_shoulder":   4,
    "back_right_elbow":      5,
    "front_left_hip":        6,
    "front_left_shoulder":   7,
    "front_left_elbow":      8,
    "front_right_hip":       9,
    "front_right_shoulder": 10,
    "front_right_elbow":    11

The teensy uses the order
    [FR hip, FR shoulder, FR knee,
     FL hip, FL shoulder, FL knee,
     BR hip, BR shoulder, BR knee,
     BL hip, BL shoulder, BL knee]

"""
class whisperer:
    def __init__(self):

        # robot_states is a dictionary with the following keys:
        # ts : current time 1x1
        # vel : velocity 12x1 (array)
        # pos : position 12x1 (array)
        # cur : current 12x1 (array)
        # pref : reference pos 12x1 (array)
        # lcur : last command 12x1 (array)
        # quat : robot orientation wxyz (array)
        self.robot_states_ = {'pos': [0] * 12, 'vel': [0] * 12, 'cur': [0] * 12, 'quat' : [1,0,0,0]}

        # The zero position map is the joint position offset when the pupper is zeroed 
        # laying down with elbows back.                     
        self.zero_pos_map = {"back_left_hip":          .06539,
                            "back_left_shoulder":     1.19682, 
                            "back_left_elbow":        2.71176, 
                            "back_right_hip":         -.06539,  
                            "back_right_shoulder":   -1.19682, 
                            "back_right_elbow":      -2.71176, 
                            "front_left_hip":          .06539, 
                            "front_left_shoulder":    1.19682, 
                            "front_left_elbow":       2.71176, 
                            "front_right_hip":        -.06539, 
                            "front_right_shoulder":  -1.19682,
                            "front_right_elbow":     -2.71176
                            }
        self.joint_name_map = {"back_left_hip":      9, # -
                            "back_left_shoulder":    10, # + 
                            "back_left_elbow":       11, # -
                            "back_right_hip":        6, # - 
                            "back_right_shoulder":   7, # -
                            "back_right_elbow":      8, # +
                            "front_left_hip":        3, # -
                            "front_left_shoulder":   4, # +
                            "front_left_elbow":      5, # -
                            "front_right_hip":       0, # -
                            "front_right_shoulder":  1, # -
                            "front_right_elbow":     2  # +
                            }
        self.joint_sign_map = {"back_left_hip":      -1, # -
                            "back_left_shoulder":     1, # + 
                            "back_left_elbow":       -1, # -
                            "back_right_hip":        -1, # - 
                            "back_right_shoulder":   -1, # -
                            "back_right_elbow":       1, # +
                            "front_left_hip":        -1, # -
                            "front_left_shoulder":    1, # +
                            "front_left_elbow":      -1, # -
                            "front_right_hip":       -1, # -
                            "front_right_shoulder":  -1, # -
                            "front_right_elbow":      1  # +
                            }
        self.torque_commands_reordered = [0] * 12

    def get_joint_state(self, joint_name):
        i = self.joint_name_map[joint_name]
        sign = self.joint_sign_map[joint_name]
        zero_offset = self.zero_pos_map[joint_name] # We currently implement the offset on our side
        joint_pos = self.robot_states_['pos'][i] * sign + zero_offset
        joint_vel = self.robot_states_['vel'][i] * sign
        joint_cur = self.robot_states_['cur'][i] 
        return [joint_pos, joint_vel, joint_cur]

    def get_pupper_orientation(self):
        if 'quat' in self.robot_states_:
            quaternion = self.robot_states_['quat']
            return quaternion
        else:
            print("Quaternion could not be read")
            return [1,0,0,0]

    def store_robot_states(self, robot_states):
        if robot_states is not None:
            self.robot_states_ = robot_states
        else:
            print("Robot states is None")

    def reorder_torques(self, torque_commands):
        i = 0
        for name, index in self.joint_name_map.items():
            self.torque_commands_reordered[index] = torque_commands[i] * self.joint_sign_map[name]
            i = i + 1
            #Debugging:
            # print("NAME: ", name)
            # print("INDEX: ", index)
            # print("VALUE: ", torque_commands[i])
            # sign = self.joint_sign_map[name]
            # print("SIGN: ", sign)
        return self.torque_commands_reordered

    def check_errors(self):
        if "err" in self.robot_states_:
            print("Pupper in kError mode (fault limits violated)")
            return True
        else:
            return False

    def print_states(self):
        # --------------------- Make sure we're reading info from pupper ---------------------------
        print("front right hip : {:+.5f}".format(self.get_joint_state("front_right_hip")[0]), "  "
                "front right shoulder : {:+.5f}".format(self.get_joint_state("front_right_shoulder")[0]), "  "
                "front right elbow : {:+.5f}".format(self.get_joint_state("front_right_elbow")[0]))

        print("back left hip : {:+.5f}".format(self.get_joint_state("back_left_hip")[0]), "  "
                "back left shoulder : {:+.5f}".format(self.get_joint_state("back_left_shoulder")[0]), "  "
                "back left elbow : {:+.5f}".format(self.get_joint_state("back_left_elbow")[0]))

        print("back right hip : {:+.5f}".format(self.get_joint_state("back_right_hip")[0]), "  "
                "back right shoulder : {:+.5f}".format(self.get_joint_state("back_right_shoulder")[0]), "  "
                "back right elbow : {:+.5f}".format(self.get_joint_state("back_right_elbow")[0]))
        
        print("front left hip : {:+.5f}".format(self.get_joint_state("front_left_hip")[0]), "  "
                "front left shoulder : {:+.5f}".format(self.get_joint_state("front_left_shoulder")[0]), "  "
                "front left elbow : {:+.5f}".format(self.get_joint_state("front_left_elbow")[0]))