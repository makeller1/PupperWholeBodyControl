#ifndef PUPPER_CONTROLLER_HH_
#define PUPPER_CONTROLLER_HH_

#include <vector>
#include <array>
#include <string>
#include <cstring>
#include <unordered_map>

#include "osqp/osqp.h"
#include "rbdl/rbdl.h"
#include "eigen3/Eigen/Dense"
#include "workstation/WBCTask.hpp"

#define ROBOT_NUM_JOINTS 12
#define NUM_JOINTS 18   // degrees of freedom to describe velocities for 12 motors plus 6 for floating base joint (3 trans 3 rot)
#define NUM_Q 19        // degrees of freedom to describe angles for 12 motors plus 3 xyz plus 4 quaternion, refer to: https://rbdl.github.io/df/dbe/joint_description.html

// To make the code a little more readable
typedef RigidBodyDynamics::Math::MatrixNd Matrix;
typedef RigidBodyDynamics::Math::VectorNd VectorNd;
typedef RigidBodyDynamics::Math::Quaternion Quat;

class PupperWBC{
public:
    // Constructor
    PupperWBC();

    // Take in the current robot data
    void updateController(const VectorNd& joint_angles, 
                          const VectorNd& joint_velocities,
                          const Eigen::Vector3d& body_position,
                          const Eigen::Quaterniond& body_quaternion,
                          const std::array<bool, 4>& feet_in_contact,
                          const double time_s);

    // Add a task to the robot's task list
    void addTask(std::string name, Task* task);
    Task* getTask(std::string name);

    // Update a task with its current measurement
    void updateJointTask(  std::string name, VectorNd state);
    void updateBodyPosTask(std::string name, Eigen::Vector3d state);
    void updateBodyOriTask(std::string name, Eigen::Quaternion<double> state);

    // Get the position of a point on a body relative to the root body
    VectorNd getRelativeBodyLocation(std::string body_name, VectorNd offset = VectorNd::Zero(3));

    // Getter for the robot joint angles
    VectorNd getJointPositions();

    // Get the calculated height (m from ground to origin of bottom pcb)
    double getCalculatedHeight();

    // Load the Pupper model from a URDF
    void Load(std::string filename);
    void Load(RigidBodyDynamics::Model& model);

    // Get the torque command fulfilling the current tasks
    std::array<float, 12> calculateOutputTorque();
    // Get the desired joint (motor) accelerations fulfilling the current tasks
    std::array<float, 12> getDesiredAccel();

private:
    // The Pupper model for RBDL
    RigidBodyDynamics::Model Pupper_;
    // Pupper constraints struct for RBDL
    RigidBodyDynamics::ConstraintSet pup_constraints_;

    // Retrieve the body COM Jacobian
    Matrix getBodyJacobian_(std::string body_id, const Eigen::Vector3d &offset = Eigen::Vector3d::Zero());

    // Retrieve the task Jacobian for specific task
    Matrix getTaskJacobian_(std::string task_name);
    Matrix getTaskJacobian_(unsigned priority);

    // Initialize RBDL constraints 
    void initConstraintSets_();

    // Retrieve the contact Jacobian for the active contacts
    void updateContactJacobian_(bool update_kinematics = true);

    // Calculate robot height using forward kinematics
    double calcRobotHeight_();

    // Store the robot state
    // Note: base is the coordinate frame fixed to the center of bottom PCB and oriented with global frame
    VectorNd joint_angles_;     // joint angles in rad. Describes 12 motors plus floating base of 3 xyz and 4 quaternion, ...
                                // ORDER: t_x, t_y, t_z, quat_1, quat_2, quat_3, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12, quat_4
                                //        where t is translation of the base, w is quaternion orientation of the base
    VectorNd joint_velocities_; // joint velocities in rad/s. Describes 12 motors plus floating base of xyz_dot and 3 angular velocities 
                                // ORDER: v_x, v_y, v_z, w_x, w_y, w_z, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12
    VectorNd robot_position_;    // robot base position in meters  
    double robot_height_;        // distance from floor to robot base in base coordinates in m
    VectorNd robot_velocity_;    // robot base velocity in m/s
    Quat   robot_orientation_;   // robot orientation from IMU (Quaternion)
    Matrix Jc_;                  // contact Jacobian. Jc_.transpose() maps reaction forces to forces/torques on the joints (including floating joint).
    Matrix massMat_;             // mass matrix
    VectorNd b_g_;               // coriolis plus gravity
    std::array<bool, 4> feet_in_contact_;  // Feet in contact (boolean array: [back left, back right, front left, front right])

    // Control torques in Nm
    VectorNd control_torques_;
    // Desired joint (motor) accelerations in m/s^2
    VectorNd control_qddot_;

    // Body ids of lower links 
    uint back_left_lower_link_id_;
    uint back_right_lower_link_id_;
    uint front_left_lower_link_id_;
    uint front_right_lower_link_id_;

    // Map of robot tasks organized by name
    std::vector<Task*> robot_tasks_;
    std::unordered_map<std::string, unsigned> task_indices_;

    // Location of center of foot in lower link coordinates (center of hemisphere face)
    const RigidBodyDynamics::Math::Vector3d body_contact_point_left_ =  RigidBodyDynamics::Math::Vector3d(0.0, -.11, 0.0095);
    const RigidBodyDynamics::Math::Vector3d body_contact_point_right_ = RigidBodyDynamics::Math::Vector3d(0.0, -.11, -0.0095);

    // OSQP Solver
    std::unique_ptr<OSQPSettings> QP_settings_;
    std::unique_ptr<OSQPData> QP_data_;
    void convertEigenToCSC_(const Matrix &P, std::vector<c_float> &P_x, std::vector<c_int> &P_p, std::vector<c_int> &P_i, bool triup = false);
    void formQP(Matrix &P, VectorNd &q, Matrix &A, VectorNd &l, VectorNd &u);
    VectorNd solveQP(int n, int m, Matrix  &P, c_float *q, Matrix  &A, c_float *lb, c_float *ub);

    // Used for numerical derivative of task (x_dot)
    double now(); // returns time in seconds 
    VectorNd taskDerivative_(const Task *T);
    double time_now_; // s current time
    double t_prev_; // s time at previous update
};

#endif
