#ifndef PUPPER_TASKMASTER_HH_
#define PUPPER_TASKMASTER_HH_

#include <array>
#include <vector>
#include <string>
#include "workstation/PupperWBC.hpp"

enum struct GoalName{
    THREE_LEG_STANCE,
    GETUP,
    STAND
};

class TaskMaster{
public:
    TaskMaster(); // Constructor
    void setGoal(GoalName goal, PupperWBC& WBC_, std::array<bool,4>& feet_in_contact);
    void updateGoalTasks(PupperWBC& WBC_, const double& time_ms);
    double time_init_ms; // time when joint config control ends (ms)
    double time_fall_ms; // time when fall (zero joint torque) ends (ms)
    std::vector<float> init_angles;
private:
    GoalName current_goal_;
};

#endif