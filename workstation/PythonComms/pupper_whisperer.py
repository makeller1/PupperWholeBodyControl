class whisperer:
    def __init__(self):
        # Class used to communicate with pupper through run_djipupper.py

        # robot_states is a dictionary with the following keys:
        # ts : current time 1x1
        # vel : velocity 12x1 (array)
        # pos : position 12x1 (array)
        # cur : current 12x1 (array)
        # pref : reference pos 12x1 (array)
        # lcur : last command 12x1 (array)
        # quat : robot orientation wxyz (array)
        self.robot_states_ = {'pos': [0] * 12, 'vel': [0] * 12, 'cur': [0] * 12, 'quat' : [1,0,0,0]}
        # WBC Order
        # self.joint_name_map = {"back_left_hip":         0,
        #                        "back_left_shoulder":    1, 
        #                        "back_left_elbow":       2,
        #                        "back_right_hip":        3,
        #                        "back_right_shoulder":   4,
        #                        "back_right_elbow":      5,
        #                        "front_left_hip":        6,
        #                        "front_left_shoulder":   7,
        #                        "front_left_elbow":      8,
        #                        "front_right_hip":       9,
        #                        "front_right_shoulder": 10,
        #                        "front_right_elbow":    11
        #                        }

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
        zero_offset = self.zero_pos_map[joint_name] # If we want to do the offset on our side
        joint_pos = self.robot_states_['pos'][i] * sign + zero_offset #Now done on teensy
        joint_vel = self.robot_states_['vel'][i] * sign
        joint_cur = self.robot_states_['cur'][i] 
        return [joint_pos, joint_vel, joint_cur]

    def get_pupper_orientation(self):
        quaternion = self.robot_states_['quat']
        return quaternion

    def store_robot_states(self, robot_states):
        if robot_states is not None:
            self.robot_states_ = robot_states

    def reorder_torques(self, torque_commands):
        i = 0
        for name, index in self.joint_name_map.items():
            # print("NAME: ", name)
            # print("INDEX: ", index)
            # print("VALUE: ", torque_commands[i])
            # sign = self.joint_sign_map[name]
            # print("SIGN: ", sign)
            self.torque_commands_reordered[index] = torque_commands[i] * self.joint_sign_map[name]
            i = i + 1
        return self.torque_commands_reordered