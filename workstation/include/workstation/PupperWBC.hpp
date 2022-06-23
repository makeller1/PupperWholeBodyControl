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
#include "workstation/NumpyLogger.hpp" // for logging test data to numpy

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
                          const Eigen::Quaterniond& body_quaternion,
                          const Eigen::Vector3d& ang_velocity,
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

    // Load the Pupper model from a URDF
    void Load(std::string filename);
    void Load(RigidBodyDynamics::Model& model);

    // Get the torque command fulfilling the current tasks
    std::array<float, 12> calculateOutputTorque();
    // Get the desired joint (motor) accelerations fulfilling the current tasks
    std::array<float, 12> getOptimalAccel();

    // Location of center of foot in lower link coordinates (center of hemisphere face)
    const RigidBodyDynamics::Math::Vector3d body_contact_point_left =  RigidBodyDynamics::Math::Vector3d(0.0, -.11, 0.0095);
    const RigidBodyDynamics::Math::Vector3d body_contact_point_right = RigidBodyDynamics::Math::Vector3d(0.0, -.11, -0.0095);

    // For tuning and debugging
    void printDiag();

    // WBC parameters
    double lambda_q; // Penalizes high joint accelerations (q_ddot^2)
    VectorNd rf_desired = VectorNd::Zero(12); // Desired reaction forces
    double lambda_rf_z; // Normal reaction force penalty (minimize impacts)
    double lambda_rf_xy; // Tangential reaction force penalty (minimize slipping - prevent lateral force contribution to ori task)
    double w_rf_xy; // Tangential reaction force tracking penalty (follow desired reaction force)
    double w_rf_z; // Normal reaction force tracking penalty (follow desired reaction force)
    double mu; // Coefficient of friction 

    // For logging with numpy
    NumpyLogger np_logger;

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

    // Retrieve the active target selection matrix for specific task
    Matrix getTaskU_(unsigned priority);

    // Initialize RBDL constraints 
    void initConstraintSets_();

    // Retrieve the contact Jacobian for the active contacts
    void calcContactJacobian_();

    // Retrieve the contact Jacobian for the active contacts
    void updateContactJacobian_(bool update_kinematics = true);

    // Store the robot state
    // Note: base is the coordinate frame fixed to the center of bottom PCB and oriented with global frame
    VectorNd joint_angles_;     // joint angles in rad. Describes 12 motors plus floating base of 3 xyz and 4 quaternion, ...
                                // ORDER: t_x, t_y, t_z, quat_1, quat_2, quat_3, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12, quat_4
                                //        where t is translation of the base, quat is quaternion orientation of the base
    VectorNd joint_velocities_; // joint velocities in rad/s. Describes 12 motors plus floating base of xyz_dot and 3 angular velocities 
                                // ORDER: v_x, v_y, v_z, w_x, w_y, w_z, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12
    VectorNd robot_velocity_;    // robot base velocity in m/s
    Quat   robot_orientation_;   // robot orientation from IMU (Quaternion)
    Matrix Jc_;                  // contact Jacobian. Jc_.transpose() maps reaction forces to forces/torques on the joints (including floating joint).
    Matrix massMat_;             // mass matrix
    VectorNd b_g_;               // coriolis plus gravity
    std::array<bool, 4> feet_in_contact_;  // Feet in contact (boolean array: [back left, back right, front left, front right])

    // Control torques in Nm
    VectorNd control_torques_;
    // Optimal joint (motor) accelerations in rad/s^2 (solution of QP)
    VectorNd control_qddot_;

    // Body ids of lower links 
    uint back_left_lower_link_id_;
    uint back_right_lower_link_id_;
    uint front_left_lower_link_id_;
    uint front_right_lower_link_id_;

    // Map of robot tasks organized by name
    std::vector<Task*> robot_tasks_;
    std::unordered_map<std::string, unsigned int> task_indices_;

    // OSQP Solver
    std::unique_ptr<OSQPSettings> QP_settings_;
    std::unique_ptr<OSQPData> QP_data_;
    OSQPWorkspace* work_;
    void setupOSQP(int n, int m, Matrix &P, c_float  *q, Matrix &A, c_float  *lb, c_float  *ub);
    void convertEigenToCSC_(const Matrix &P, std::vector<c_float> &P_x, std::vector<c_int> &P_p, std::vector<c_int> &P_i, bool triup = false);
    void convertEigenToCSCSparsePat_(const Matrix &P, std::vector<c_float> &P_x, std::vector<c_int> &P_p, std::vector<c_int> &P_i, bool triup = false);
    void formQP(Matrix &P, VectorNd &q, Matrix &A, VectorNd &l, VectorNd &u);
    VectorNd solveQP(int n, int m, Matrix  &P, c_float *q, Matrix  &A, c_float *lb, c_float *ub);

    // Used for numerical derivative of task (x_dot)
    // returns time in seconds 
    double now();
    VectorNd taskDerivative_(const Task *T);
    double time_now_; // s current time
    double t_prev_; // s time at previous update

    // Map feet body names to indices
    const std::unordered_map<std::string, unsigned int> feet_indices = {{"back_left_foot",0},
                                                                        {"back_right_foot",1},
                                                                        {"front_left_foot",2},
                                                                        {"front_right_foot",3}};
    // QP variables to avoid repeated allocation
    // Objective function terms
    Matrix P = Matrix::Zero(NUM_JOINTS + 12, NUM_JOINTS + 12);
    VectorNd q = VectorNd::Zero(NUM_JOINTS + 12);
    
    // Constraint terms
    Matrix A = Matrix::Zero(38, NUM_JOINTS + 12); // 18 for torque limit, 16 for cone, 4 for normal reaction
    VectorNd lower_bounds = VectorNd::Zero(38);
    VectorNd upper_bounds = VectorNd::Zero(38);

    // Store optimal WBC solution to return previous value if OSQP fails
    VectorNd optimal_solution_;

    // For tuning and debugging
    VectorNd optimal_qddot_; // Optimal generalized accelerations
    VectorNd optimal_rf_; // Optimal reaction forces

    // Sparsity structure of QP matrices TODO: Make these into arrays of non-zero elements to save space
    std::array<std::array<bool, 30>, 30> P_sparsity_ = 
    {{{1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
    {1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {1,1,1,1,1,1,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {1,1,1,1,1,1,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {1,1,1,1,1,1,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
    {1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
    {1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1}}};

    std::array<std::array<bool, 38>, 38> A_sparsity_ =
    {{{1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,0,0},
    {0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,0,1,0,0,1,0,0,1,0},
    {0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,1,0,0,1,0,0,1,0,0,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0},
    {1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0},
    {1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0},
    {1,1,1,1,1,1,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0},
    {1,1,1,1,1,1,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0},
    {1,1,1,1,1,1,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0},
    {1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0},
    {1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0},
    {1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0},
    {1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1},
    {1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1,1,1},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1}}};
};

#endif
