#include <chrono>
#include <memory>
#include <iostream>
#include <iomanip> // for setting cout precision
#include "workstation/PupperWBC.hpp"
#include "rbdl/addons/urdfreader/urdfreader.h"

// TODO: 
/*
    - Move x_ddot_desired calculation to its own function
    - Define other WBC parameters with task master
    - Saturate x_ddot_desired
*/

using std::vector;
using std::array;
using std::string;
using std::cout;
using std::endl;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

#define PRINT_CLEAN(M) cout << #M << ":" << endl; printClean(M);

namespace {
    Eigen::IOFormat f(3);

    // Computes the rotation from q1 to q2 (analogous to q2-q1) as a non-normalized axis angle.
    // The axis angle is described in the same frame as q1.
    VectorNd quatDiff(Eigen::Quaternion<double> q2, Eigen::Quaternion<double> q1){
        Eigen::Quaternion<double> error = q1.conjugate() * q2;
        // Get the 3d axis angle of the rotation
        VectorNd error3d = VectorNd::Zero(3);
        error3d = error.vec() * error.w()/abs(error.w());
        return error3d;
    }

    inline void printClean(Eigen::MatrixXd &M){
        for (int i = 0; i < M.rows(); i++){
            for (int j = 0; j < M.cols(); j++){
                if (abs(M(i,j)) < 1e-4) M(i,j) = 0;
            }
        }

        cout << M << endl;
    }

    // start timer (ms)
    double tic(int mode=0) {
        static std::chrono::_V2::system_clock::time_point t_start;
        
        if (mode==0)
            t_start = std::chrono::high_resolution_clock::now();
        else {
            auto t_end = std::chrono::high_resolution_clock::now();
            double t_delta = (t_end-t_start).count()*1E-6;
            return t_delta;
        }
        return 0.0;
    }
    // return elapsed time (ms)
    double toc() { return tic(1); }

}

// Constructor
PupperWBC::PupperWBC(){
    // Correctly size the robot state vectors
    joint_angles_     = VectorNd::Zero(NUM_Q);
    joint_velocities_ = VectorNd::Zero(NUM_JOINTS);
    control_torques_  = VectorNd::Zero(NUM_JOINTS);
    control_qddot_ = VectorNd::Zero(ROBOT_NUM_JOINTS);
    feet_in_contact_ = {true, true, true, true};

    // Default WBC parameters
    lambda_q = 0.001; 
    rf_desired = VectorNd::Zero(12);
    lambda_rf_z = 0; 
    lambda_rf_xy = 0; 
    w_rf_xy = 0; 
    w_rf_z = 0; 
    mu = .3;

    // Create solver structs
    QP_settings_ = std::make_unique<OSQPSettings>();
    QP_data_     = std::make_unique<OSQPData>();

    // Default to identity quaternion
    robot_orientation_.x() = 0;
    robot_orientation_.y() = 0;
    robot_orientation_.z() = 0;
    robot_orientation_.w() = 1;

    // Robot dynamics matrices
    Jc_      = MatrixNd::Zero(12, NUM_JOINTS);
    massMat_ = MatrixNd::Zero(NUM_JOINTS,NUM_JOINTS);
    b_g_     = VectorNd::Zero(NUM_JOINTS);

    // Used for numerical derivatives
    t_prev_ = now();
}

// Update the controller with the current state of the robot
void PupperWBC::updateController(const VectorNd& joint_angles, 
                                 const VectorNd& joint_velocities,
                                 const Eigen::Quaterniond& body_quaternion,
                                 const Eigen::Vector3d& ang_velocity,
                                 const array<bool, 4>& feet_in_contact,
                                 const double time_s){
    // Copy over the joint states
    for (int i = 0; i < ROBOT_NUM_JOINTS; i++){
        joint_angles_[i+6]     = joint_angles[i];
        joint_velocities_[i+6] = joint_velocities[i];
    }

    // Copy over the body angular velocity states
    for (int i = 0; i < 3; i++){
        joint_velocities_[i+3] = ang_velocity[i];
    }

    // Record the body orientation quaternion
    robot_orientation_.w() = body_quaternion.w();
    robot_orientation_.x() = body_quaternion.x();
    robot_orientation_.y() = body_quaternion.y();
    robot_orientation_.z() = body_quaternion.z();

    Pupper_.SetQuaternion(Pupper_.GetBodyId("bottom_PCB"), robot_orientation_, joint_angles_);

    // Copy which feet are in contact
    feet_in_contact_ = feet_in_contact;

    // Record current time
    t_prev_ = time_now_;
    time_now_ = time_s;

    // Update the problem matrices
    massMat_.setZero(); // Required!
    Jc_.setZero(); // Just to be safe; not clear if this is required 
    b_g_.setZero(); // Just to be safe; not clear if this is required 

    tic();
    // Log time required to run this line
    updateContactJacobian_(true);
    np_logger.logScalars({"ucj_ms"},{toc()});

    tic();
    // Log time required to run this line
    CompositeRigidBodyAlgorithm(Pupper_, joint_angles_, massMat_, false);
    np_logger.logScalars({"crb_ms"},{toc()});

    tic();
    // Log time required to run this line
    // if simulating in zero gravity, do not calculate nonlinear effects 
    NonlinearEffects(Pupper_, joint_angles_, joint_velocities_, b_g_);
    np_logger.logScalars({"nle_ms"},{toc()});

    if (b_g_[2]==0.0){
        cout << "WARNING: NONLINEAR EFFECTS DISABLED" << endl;
    }

    // Log states
    np_logger.logScalars({"time_ms"},{time_s*1000.0f});
    np_logger.logScalars({"quat_w","quat_x","quat_y","quat_z"},{body_quaternion.w(),body_quaternion.x(),body_quaternion.y(),body_quaternion.z()});
    np_logger.logVectorXd("joint_ang",joint_angles);
    np_logger.logVectorXd("joint_vel",joint_velocities);
    np_logger.logVectorXd("ang_vel",ang_velocity);
}

// Add a task to the IHWBC controller
void PupperWBC::addTask(string name, Task* T){
    T->task_name = name;
    task_indices_[name] = robot_tasks_.size();
    robot_tasks_.push_back(T);

    switch(T->type){
        case REACTION_FORCE:
            // Fall through to body pos case
        case BODY_POS:
            if (Pupper_.GetBodyId(T->body_id.c_str()) == -1){
                string message = "Task name " + T->body_id + " did not match any known body";
                throw(std::runtime_error(message));
            }
            T->pos_measured.fill(0);
            T->dpos_measured.fill(0);
            T->active_targets.resize(3, false);
            break;

        case BODY_ORI:
            if (Pupper_.GetBodyId(T->body_id.c_str()) == -1){
                string message = "Task name " + T->body_id + " did not match any known body";
                throw(std::runtime_error(message));
            }
            T->quat_measured = Eigen::Quaterniond::Identity();
            T->last_quat_measured = Eigen::Quaterniond::Identity();
            break;

        case JOINT_POS:
            assert(T->joint_target.size() == ROBOT_NUM_JOINTS);
            T->joint_measured = VectorNd::Zero(12,1);
            T->djoint_measured = VectorNd::Zero(12,1);
            T->djoint_measured = VectorNd::Zero(12,1);
            T->last_joint_measured = T->joint_measured;
            T->djoint_target = VectorNd::Zero(12,1);
            T->active_targets.resize(12, false);
            break;
    }
}

// Update the measured state of the task focus
void PupperWBC::updateJointTask(std::string name, VectorNd state){
    if (not task_indices_.count(name)) {
        cout << "\e[1;33m" << "Task " << name << " does not exist" << "\e[0m" << endl;
        return;
    }
    Task* T = getTask(name);
    assert(T->type == JOINT_POS);
    T->last_joint_measured = T->joint_measured;
    T->joint_measured = joint_angles_.segment(6,12); // ORDER: t_x, t_y, t_z, quat_1, quat_2, quat_3, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12, quat_4
    T->djoint_measured = joint_velocities_.segment(6,12); // ORDER: v_x, v_y, v_z, w_x, w_y, w_z, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12
}

void PupperWBC::updateBodyPosTask(std::string name, Eigen::Vector3d state){
    if (not task_indices_.count(name)) {
        cout << "\e[1;33m" << "Task " << name << " does not exist" << "\e[0m" << endl;
        return;
    }
    Task* T = getTask(name);
    assert(T->type == BODY_POS || T->type == REACTION_FORCE);
    T->last_pos_measured = T->pos_measured;
    T->pos_measured = state;
}

void PupperWBC::updateBodyOriTask(std::string name, Eigen::Quaternion<double> state){
    if (not task_indices_.count(name)) {
        cout << "\e[1;33m" << "Task " << name << " does not exist" << "\e[0m" << endl;
        return;
    }
    Task* T = getTask(name);
    assert(T->type == BODY_ORI);
    T->last_quat_measured = T->quat_measured;
    T->quat_measured = state;
}

Task* PupperWBC::getTask(string name){
    if (not task_indices_.count(name)) {
        string message = "\e[1;33m Trying to get Task " + name + " which does not exist \e[0m";
        throw(std::runtime_error(message));
    }
    return robot_tasks_[task_indices_[name]];
}

VectorNd PupperWBC::taskDerivative_(const Task *T){
    VectorNd deriv;
    float dt = now() - t_prev_;

    switch (T->type){
        case REACTION_FORCE:
            // Fall through to body pos case

        case BODY_POS:
        {
            deriv.resize(3);
            deriv = (T->pos_measured - T->last_pos_measured)/dt; //TODO: Replace with j*q_dot
            break;
        }
        case BODY_ORI:
            deriv.resize(3);
            // for (int i = 0; i < 3; i++){
            //     deriv(i) = joint_velocities_(i+3); // measured angular velocity
            // }
            deriv = 2.0*quatDiff(T->quat_measured, T->last_quat_measured)/dt; // Numerical derivative
            break;

        case JOINT_POS:
            deriv.resize(ROBOT_NUM_JOINTS);
            deriv = T->djoint_measured;
            break;

    }

    return deriv;
}

VectorNd PupperWBC::getRelativeBodyLocation(std::string body_name, VectorNd offset){
    // Returns position in base (root) coordinates
    int id = Pupper_.GetBodyId(body_name.c_str());
    VectorNd pos = CalcBodyToBaseCoordinates(Pupper_, joint_angles_, id, offset, false);
    return pos;
}

// Load the model
void PupperWBC::Load(std::string urdf_file_string){
    // Get the model information from the URDF
    RigidBodyDynamics::Addons::URDFReadFromString(urdf_file_string.c_str(), &Pupper_, true, true);

    // Summarize model characteristics
    printf("Loaded model with %d DOFs\n", Pupper_.dof_count);
    printf("*\tQ Count:   \t%d\n", Pupper_.q_size);
    printf("*\tQdot Count:\t%d\n", Pupper_.qdot_size);
    printf("*\tBody Count:\t%zd\n", Pupper_.mBodies.size());
    printf("*\tJoint Count:\t%zd\n", Pupper_.mJoints.size());
    printf("*\tBody Names:\n");
    for (int i = 0; i < Pupper_.mBodies.size(); i++)
        cout<< "*\t|----" << i << "\t=> " << Pupper_.GetBodyName(i) << endl;

    // Set the robot state
    Pupper_.SetQuaternion(Pupper_.GetBodyId("bottom_PCB"), robot_orientation_, joint_angles_);
    initConstraintSets_();
}

void PupperWBC::Load(Model& m){
    Pupper_ = m;

    Pupper_.SetQuaternion(Pupper_.GetBodyId("bottom_PCB"), robot_orientation_, joint_angles_);
    initConstraintSets_();

 
    //Print model diagnostics
    // Eigen::Vector3d offset = Eigen::Vector3d::Zero();
    // cout << "\nAll bodies:" << endl;
    // for (int i = 0; i < Pupper_.mBodies.size(); i++){
    //     cout << i << ") Mass of " << Pupper_.GetBodyName(i) << ": " << Pupper_.mBodies[i].mMass << "kg\n";
        
    //     VectorNd body_coord = CalcBodyToBaseCoordinates(Pupper_, joint_angles_, i, offset, false);
    //     cout << "  location in base coord.: " << body_coord.transpose() << endl;
    // }
    Vector3d com_location;
    double model_mass;
    RigidBodyDynamics::Utils::CalcCenterOfMass(Pupper_,joint_angles_,joint_velocities_,NULL, model_mass, com_location);
    cout << "Total mass from RBDL: " << model_mass << endl;
    cout << "Center of mass location: " << com_location.transpose() << endl;
}


array<float, 12> PupperWBC::calculateOutputTorque(){
    // Solve for q_ddot and reaction forces
    // cout << "FORMING QP ---------------------------------------------" << endl;
    P.setZero();
    q.setZero();
    A.setZero();
    lower_bounds.setZero();
    upper_bounds.setZero();
    tic();
    // Log time required to run this line
    formQP(P, q, A, lower_bounds, upper_bounds);
    np_logger.logScalars({"fqp_ms"},{toc()});
    
    tic();
    // Log time required to run this line
    optimal_solution_ = solveQP(A.cols(), A.rows(), P, q.data(), A, lower_bounds.data(), upper_bounds.data());
    np_logger.logScalars({"sqp_ms"},{toc()});

    VectorNd q_ddot = optimal_solution_.head(NUM_JOINTS);
    VectorNd F_r    = optimal_solution_.tail(12);

    // cout << "Solution: -----------------" << endl;
    // for (int i = 0; i < optimal_solution.size(); i++){
    //     cout<< optimal_solution[i] << endl;
    // }
    // cout << " --------------------------" << endl;

    // Solve for the command torques
    VectorNd tau = (massMat_*q_ddot + b_g_ - Jc_.transpose()*F_r).tail(ROBOT_NUM_JOINTS);

    // Store optimal motor accelerations
    control_qddot_ = q_ddot.tail(ROBOT_NUM_JOINTS);

    // Store optimal solution for diagnostics and tuning
    optimal_qddot_ = q_ddot;
    optimal_rf_ = F_r;

    VectorNd Fr = optimal_solution_.tail(12); // reaction forces
    // VectorNd tau_gen = VectorNd::Zero(18); // generalized torques
    // tau_gen.tail(12) = tau;
    
    // cout << "Joint angles for test: " << joint_angles_.transpose().format(f) << endl;
    // cout << "Joint velocities for test: " << joint_velocities_.transpose().format(f) << endl;
    // cout << "torques for test: " << tau_gen.transpose().format(f) << endl;

    // VectorNd QDDOT = VectorNd::Zero(18);
    // ForwardDynamicsConstraintsDirect(Pupper_,joint_angles_,joint_velocities_,tau_gen,pup_constraints_,QDDOT);
    // cout << "Qdotdot OSQP: -------------------------" << endl;
    // cout << optimal_solution.head(18).transpose().format(f) << endl;
    //cout << "Qdotdot RBDL: \n" << QDDOT.transpose().format(f) << endl;
    // cout << " --------------------------------------" << endl;

    // std::cout << std::setprecision(3) << std::fixed;
    // cout << "Commanded Torques: -----------------\n";
    // cout << tau.transpose() << "\n";
    // cout << "Reaction Forces OSQP: --------------\n";
    // cout << Fr.transpose().format(f) << endl;
    np_logger.logVectorXd("joint_q_ddot",control_qddot_);
    np_logger.logVectorXd("tau",tau);
    np_logger.logVectorXd("Fr",Fr);

    // cout << "Jc' (floating base): " << endl;
    // cout << (Jc_.transpose()).topRows(6) << endl;
    // cout << "Jc': " << endl;
    // cout << (Jc_.transpose()) << endl;
    // cout << "Jc'*Fr: ---------" << endl;
    // cout << (Jc_.transpose()*F_r).transpose() << endl;
    // cout << "b_g_: ---------" << endl;
    // std::cout << std::setprecision(4) << std::fixed;
    // cout << b_g_.transpose() << endl; 
    
    // cout << "q_ddot: -----------" << endl;
    // cout << q_ddot.transpose() << endl;
    // cout << "M*q_ddot: ---------" << endl;
    // cout << (massMat_*q_ddot).transpose() << endl;
    // cout << "M: ---------" << endl;
    // cout << massMat_ << endl;

    // std::cout << std::setprecision(3) << std::fixed;
    //cout << "Reaction forces RBDL: \n " << pup_constraints_.force.transpose().format(f) << endl;
    //cout << " --------------------------------------" << endl;

    // //-------------------------------------------------------------------------------//
    // //-------------------------------------------------------------------------------//
    // Check constraints:
    // double mu = 1; 
    // VectorNd Ax = (A*optimal_solution);
    // First leg x
    // cout << "Cone Constraint Test --------------------------------" << endl;
    //cout << "Fr_x_1: " << abs(Fr(0)) << " <= " << mu*Fr(2) << endl;
    //cout << "Fr_y_1: " << abs(Fr(1)) << " <= " << mu*Fr(2) << endl;

    // cout << "----------------------------------------------------" << endl;
    //cout << "A size: " << A.rows() << "x" << A.cols() << endl;
    //cout << "x size: " << optimal_solution.rows() << endl;
    //cout << "Lower Bounds for F_r: \n" << upper_bounds.tail(20).transpose().format(f)<<endl;
    //cout << "A*x for F_r = " << Ax.tail(20).transpose().format(f) << endl; 

    // //-------------------------------------------------------------------------------//
    // //-------------------------------------------------------------------------------//

    array<float, 12> output;
    std::copy(tau.data(), tau.data() + tau.size(), output.data());
    return output;
}

array<float, 12> PupperWBC::getOptimalAccel(){
    array<float, 12> output;
    std::copy(control_qddot_.data(), control_qddot_.data() + control_qddot_.size(), output.data());
    return output;
}

void PupperWBC::initConstraintSets_(){
    // Perform initialization of RBDL contact constraints
    // Contact normals are defined in the global frame
    // Order is as follows (Rows of the contact Jacobian):
    // Back left foot
    //      X,Y,Z constraints (3 rows)
    // Back right foot
    //      X,Y,Z constraints (3 rows)
    // Front left foot
    //      X,Y,Z constraints (3 rows)
    // Front right foot 
    //      X,Y,Z constraints (3 rows)
    back_left_lower_link_id_ = Pupper_.GetBodyId("back_left_lower_link");
    back_right_lower_link_id_ = Pupper_.GetBodyId("back_right_lower_link");
    front_left_lower_link_id_ = Pupper_.GetBodyId("front_left_lower_link");
    front_right_lower_link_id_ = Pupper_.GetBodyId("front_right_lower_link");

    // Contact normal direction in global coordinates
    const Math::Vector3d world_x(1.0, 0.0, 0.0); 
    const Math::Vector3d world_y(0.0, 1.0, 0.0); 
    const Math::Vector3d world_z(0.0, 0.0, 1.0); 

    // Back left foot 
    pup_constraints_.AddContactConstraint(back_left_lower_link_id_, body_contact_point_left, world_x, "back_left_contact_x");
    pup_constraints_.AddContactConstraint(back_left_lower_link_id_, body_contact_point_left, world_y, "back_left_contact_y");
    pup_constraints_.AddContactConstraint(back_left_lower_link_id_, body_contact_point_left, world_z, "back_left_contact_z");
    // Back right foot 
    pup_constraints_.AddContactConstraint(back_right_lower_link_id_, body_contact_point_right, world_x, "back_right_contact_x");
    pup_constraints_.AddContactConstraint(back_right_lower_link_id_, body_contact_point_right, world_y, "back_right_contact_y");
    pup_constraints_.AddContactConstraint(back_right_lower_link_id_, body_contact_point_right, world_z, "back_right_contact_z");
    // Front left foot 
    pup_constraints_.AddContactConstraint(front_left_lower_link_id_, body_contact_point_left, world_x, "front_left_contact_x");
    pup_constraints_.AddContactConstraint(front_left_lower_link_id_, body_contact_point_left, world_y, "front_left_contact_y");
    pup_constraints_.AddContactConstraint(front_left_lower_link_id_, body_contact_point_left, world_z, "front_left_contact_z");
    // Front right foot 
    pup_constraints_.AddContactConstraint(front_right_lower_link_id_, body_contact_point_right, world_x, "front_right_contact_x");
    pup_constraints_.AddContactConstraint(front_right_lower_link_id_, body_contact_point_right, world_y, "front_right_contact_y");
    pup_constraints_.AddContactConstraint(front_right_lower_link_id_, body_contact_point_right, world_z, "front_right_contact_z");

    pup_constraints_.Bind(Pupper_);
}

// Retrieve body Jacobian J by ID such that [ang vel; lin vel] = J*q_dot where the 6D vector is described in the base frame.
MatrixNd PupperWBC::getBodyJacobian_(string body_id, const Eigen::Vector3d& offset) {
    // calcPointJacobian6D returns J such that [w;v] = J*q_dot where [w;v] is described in the base frame
    // Create output
    MatrixNd J = MatrixNd::Zero(6, NUM_JOINTS);

    // Fill the Jacobian matrix
    // cout << body_id << " " << endl;
    // CalcBodySpatialJacobian(Pupper_, joint_angles_, Pupper_.GetBodyId(body_id.c_str()), J, true);
    // PRINT_CLEAN(J);
    
    // calcPointJacobian6D returns J such that [w;v] = J*q_dot where [w;v] is described in the base frame
    CalcPointJacobian6D(Pupper_, joint_angles_, Pupper_.GetBodyId(body_id.c_str()), offset, J, false);
    // PRINT_CLEAN(J);
    return J;
}

MatrixNd PupperWBC::getTaskU_(unsigned priority){
    Task* T = robot_tasks_[priority];

    // Create selection matrix to remove rows we don't care about
    int n = T->active_targets.size();
    int num_active = 0;
    for (int i = 0; i<n; i++){
        num_active += T->active_targets[i];
    }
    MatrixNd U  = MatrixNd::Zero(num_active,n);
    int row = 0;
    for (int i = 0; i < n; i++){
        if (T->active_targets[i]){
            U(row, i) = T->active_targets[i];
            row += 1;
        }
    }

    return U;
}

// Get the task Jacobian (derivative of a task with respect to generalized joints)
MatrixNd PupperWBC::getTaskJacobian_(unsigned priority){
    Task* T = robot_tasks_[priority];
    MatrixNd Jt;

    // Create selection matrix to remove rows we don't care about
    MatrixNd U = getTaskU_(priority);

    switch(T->type){
        // Reaction Force Task
        case REACTION_FORCE:
            //fall through to Body Position Task 

        // Body Position Task
        case BODY_POS:
        {
            MatrixNd Jb = getBodyJacobian_(T->body_id, T->offset);
            Jt = U * Jb.bottomRows(3);
            break;
        }

        // Body Orientation Task
        case BODY_ORI:
        {
            MatrixNd Jb = getBodyJacobian_(T->body_id, T->offset);
            // In general we care about all 4 parts of a quaternion, so 
            // U is identity in this case
            Jt = U * Jb.topRows(3);
            break;
        }

        // Joint position task
        case JOINT_POS:
        {
            Jt = MatrixNd::Zero(ROBOT_NUM_JOINTS, NUM_JOINTS);

            // For joint position Jacobians the value is either 1 or zero
            for (int i = 0; i < T->active_targets.size(); i++){
                Jt(i, i+6) = (T->active_targets[i] ? 1 : 0);
            }
            Jt = U * Jt;
            break;
        }

        default:
            throw(std::runtime_error("Unrecognized WBC Task format"));
    }

    return Jt;
}

// Overloaded version of getTaskJacobian_
MatrixNd PupperWBC::getTaskJacobian_(std::string task_name){
    unsigned index = task_indices_.at(task_name);
    return getTaskJacobian_(index);
}

// Getter for the joint angles
VectorNd PupperWBC::getJointPositions(){
    return joint_angles_;
}

void PupperWBC::updateContactJacobian_(bool update_kinematics){
    // update_kinematics defaults to true.
    // Note: Update_kinematics must be true if an RBDL function has not been called previously and the joint angles have changed.
    // Method below simply calculates the jacobian of the feet
    CalcConstraintsJacobian(Pupper_, joint_angles_, pup_constraints_, Jc_, update_kinematics);
    // Zero-out the floating feet; They do not impart any forces.
    for (int i=0; i<4; i++){
        if (!feet_in_contact_[i]){
            cout << "Foot " << i << " is floating." << endl;
            Jc_.row(i*3).setZero();
            Jc_.row(i*3 + 1).setZero();
            Jc_.row(i*3 + 2).setZero();
        }
    }
    
    // cout << "Jc_': \n" << Jc_.transpose().format(f) << endl;
}

void PupperWBC::printDiag(){
    tic();
    // only weight>0
    // only active targets 
    for (int i = 0; i < robot_tasks_.size(); i++ ){
        Task* T = robot_tasks_[i];
        
        if (T->task_weight == 0){
            continue;
        }
        MatrixNd j = getTaskJacobian_(i);
        VectorNd x_ddot_desired = T->x_ddot_desired;
        
        // cout << "###################################" << endl;
        // cout << "Task " << T->task_name << "\n";
        // cout << "Weight: " << T->task_weight << "\n";

        if (T->type != REACTION_FORCE){
            double cost = T->task_weight*(j*optimal_qddot_ - T->x_ddot_desired).transpose()*(j*optimal_qddot_ - T->x_ddot_desired);
            auto acc_opt = j*optimal_qddot_; // optimal acceleration
            auto acc_des = T->x_ddot_desired; // target acceleration
            // cout << "Cost: " << cost << "\n";
            // cout << "Accel. Optimal: " << acc_opt.transpose() << "\n";
            // cout << "Accel. Desired: " << acc_des.transpose() << "\n";
            // cout << "Pos. Meas: " << T->pos_measured.transpose() << "\n";
            // cout << "Pos. Target: " << T->pos_target.transpose() << endl;

            // log data in numpy format
            np_logger.logScalars({T->task_name+"_weight"},{T->task_weight});
            np_logger.logScalars({T->task_name+"_cost"},{cost});
            np_logger.logVectorXd(T->task_name+"_acc_opt",acc_opt);
            np_logger.logVectorXd(T->task_name+"_acc_des",T->x_ddot_desired);
            np_logger.logVectorXd(T->task_name+"_pos_measured",T->pos_measured);
            np_logger.logVectorXd(T->task_name+"_pos_target",T->pos_target);

            if (T->type == BODY_ORI){
                // cout << "P: " << T->Kp.cwiseProduct(quatDiff(T->quat_measured, T->quat_target)).transpose() << "\n";
                // cout << "D: " << T->Kd.cwiseProduct(taskDerivative_(T) - T->x_ddot_ff).transpose() << endl;
            }
        }
        else{
            // Diagnostics currently only for tangential reaction force tracking tasks
            int i_foot = feet_indices.at(T->body_id);
            Vector2d optimal_rf_2d;
            Vector2d rf_desired_2d;
            optimal_rf_2d << optimal_rf_(i_foot*3), optimal_rf_(i_foot*3 + 1);
            rf_desired_2d << rf_desired(i_foot*3), rf_desired(i_foot*3 + 1);
            // cout << "Cost: " << w_rf_xy*(optimal_rf_2d-rf_desired_2d).transpose()*(optimal_rf_2d-rf_desired_2d) << endl;
            // cout << "Rf. Optimal: " << optimal_rf_2d.transpose() << endl;
            // cout << "Rf. Desired: " << rf_desired_2d.transpose() << endl;
        }
        
    }
    np_logger.logScalars({"pdg_ms"},{toc()});
}

void PupperWBC::formQP(MatrixNd &P, VectorNd &q, MatrixNd &A, VectorNd &l, VectorNd &u){
    // Optimization problem form:
    // min 1/2(x'Px) + q'x
    // st l <= Ax <= u
    //
    // x is a concatenated vector of qdotdot (18x1) and Fr (12x1) 
    // P is a sparse block matrix, P_b is a diagonal matrix
    // P = [P_a,  0  ;       Sizes: [  18x18  , (18x12) ]
    //       0 , P_b];              [ (12x18) ,  12x12  ]
    //
    // A is sparse block matrix
    // A = [ M  , -Jc'  ;    Sizes: [  18x18 , 18x12 ]
    //       0  , A_fr ];           [ (20x18), 20x12 ]

    // ---------------------------------------------------------------
    // ------------------------- OBJECTIVE ---------------------------
    // ---------------------------------------------------------------

    // Objective is of the form 1/2(x'Px) + q'x

    // Form task cost matrix and vector
    MatrixNd cost_t_mat = MatrixNd::Zero(NUM_JOINTS,NUM_JOINTS); // P
    VectorNd cost_t_vec = VectorNd::Zero(NUM_JOINTS); // q

    for (int i = 0; i < robot_tasks_.size(); i++ ){
        Task* T = robot_tasks_[i];
        
        MatrixNd j = getTaskJacobian_(i);
        MatrixNd U_active = getTaskU_(i);

        VectorNd x_ddot_desired = VectorNd::Zero(T->active_targets.size());

        switch(T->type){
            // Note: desired acceleration is -x_ddot_desired
            case BODY_ORI:
                x_ddot_desired = T->Kp.cwiseProduct(quatDiff(T->quat_measured, T->quat_target)) + T->Kd.cwiseProduct(taskDerivative_(T)) - T->x_ddot_ff;
                break;

            case BODY_POS:
                x_ddot_desired = T->Kp.cwiseProduct((T->pos_measured - T->pos_target)) + T->Kd.cwiseProduct(taskDerivative_(T) - T->dpos_target) - T->x_ddot_ff;
                break;

            case JOINT_POS:
            {
                Eigen::Matrix<double, 12, 1> Kp_rep, Kd_rep;
                // Kp and Kd are repeated for all 4 legs
                Kp_rep << T->Kp, T->Kp, T->Kp, T->Kp;
                Kd_rep << T->Kd, T->Kd, T->Kd, T->Kd;
                x_ddot_desired = Kp_rep.cwiseProduct(T->joint_measured - T->joint_target) + Kd_rep.cwiseProduct(taskDerivative_(T) - T->djoint_target);
                break;
            }
            case REACTION_FORCE:
                int i_foot = feet_indices.at(T->body_id);
                cout << "i_foot" << i_foot << endl;
                cout << "body id: " << T->body_id << endl;
                // Reaction force is opposite of the desired direction
                Eigen::Vector3d rf_desired_3d = T->Kp.cwiseProduct((T->pos_measured - T->pos_target)) + T->Kd.cwiseProduct(taskDerivative_(T) - T->dpos_target);
                for (int j = 0; j < 3; j++)
                {
                    rf_desired(i_foot*3 + j) = rf_desired_3d(j);
                    cout << "rf_desired(" << i_foot*3 + j << "): " << rf_desired(i_foot + j) << endl;
                }
                // For logging
                T->rf_desired = rf_desired;
                cout << "rf_desired full: " << rf_desired.transpose() << endl;
                cout << "rf_redired leg " << T->body_id << ": " << rf_desired_3d.transpose() << endl;
                break;
        }

        // Remove inactive rows
        x_ddot_desired = U_active * x_ddot_desired;

        // For logging
        T->x_ddot_desired = -x_ddot_desired;

        if ((T->type == BODY_POS) &&
           (T->body_id == "back_left_foot" || T->body_id == "back_right_foot" || T->body_id == "front_left_foot" || T->body_id == "front_right_foot")){
            //-----------------Calculate j_dot-----------------
            MatrixNd j_dot;
            double delta_t = (now() - t_prev_); // seconds
            if (T->j_prev_updated == true){
                j_dot = (j - T->j_prev)/delta_t;
                // cout << "j_dot calculated" << endl;
                // cout << "delta_t: " << delta_t << endl;
            }
            else{
                j_dot = MatrixNd::Zero(j.rows(),j.cols());
                T->j_prev_updated = true;
            }
            T->j_prev = j;
            MatrixNd j_dot_q_dot = j_dot * joint_velocities_;
            if (T->body_id == "back_left_foot"){
                np_logger.logVectorXd("j_dot_bl_z",j_dot.row(2));
            }

            // with j_dot_q_dot
            cost_t_vec += T->task_weight * j.transpose() * (j_dot_q_dot + x_ddot_desired); // nq x 1 
        }
        else{
            // without j_dot_q_dot
            cost_t_vec += T->task_weight * j.transpose() * x_ddot_desired; // nq x 1
        }

        //----------------- WITHOUT j_dot_q_dot -------------
        // cost_t_vec += T->task_weight * j.transpose() * x_ddot_desired; // nq x 1
        //---------------------------------------------------

        cost_t_mat += T->task_weight * j.transpose() * j; // nq x nq

    }
    // Add a cost to penalize high joint accelerations
    for (int i = 6; i < NUM_JOINTS; i++){
        cost_t_mat(i,i) += lambda_q;
    }

    // Form reaction force cost matrix and vector
    // 1. Penalize large reaction forces with form: 
    //    lambda_rf_z*||rf_z||^2
    // 2. Penalize error between desired and calculated (only for normal reaction force for now):
    //    w*||rf_d-rf||^2

    //MatrixNd cost_rf_mat = MatrixNd::Identity(12,12);
    VectorNd cost_rf_vec = VectorNd::Zero(12);
    MatrixNd cost_rf_mat = MatrixNd::Zero(12,12);
    VectorNd diag_terms  = VectorNd::Zero(12); 

    // Penalize large lateral and normal reaction forces (lambda_rf), and penalize tracking error (w_rf)
    // These elements go into the diagonal of the cost matrix
    diag_terms << lambda_rf_xy + w_rf_xy, lambda_rf_xy + w_rf_xy, lambda_rf_z +  w_rf_z, 
                  lambda_rf_xy + w_rf_xy, lambda_rf_xy + w_rf_xy, lambda_rf_z +  w_rf_z,
                  lambda_rf_xy + w_rf_xy, lambda_rf_xy + w_rf_xy, lambda_rf_z +  w_rf_z,
                  lambda_rf_xy + w_rf_xy, lambda_rf_xy + w_rf_xy, lambda_rf_z +  w_rf_z;

    cost_rf_mat = diag_terms.asDiagonal();

    // Penalizes tracking error
    cost_rf_vec <<  w_rf_xy * -rf_desired(0), w_rf_xy * -rf_desired(1), w_rf_z * -rf_desired(2),
                    w_rf_xy * -rf_desired(3), w_rf_xy * -rf_desired(4), w_rf_z * -rf_desired(5),
                    w_rf_xy * -rf_desired(6), w_rf_xy * -rf_desired(7), w_rf_z * -rf_desired(8),
                    w_rf_xy * -rf_desired(9), w_rf_xy * -rf_desired(10), w_rf_z * -rf_desired(11);
    // Form P matrix and q vector
    P.topLeftCorner(NUM_JOINTS,NUM_JOINTS) = cost_t_mat;
    P.bottomRightCorner(12,12)             = cost_rf_mat;
    q.head(NUM_JOINTS) = cost_t_vec;
    q.tail(12) = cost_rf_vec;
    
    assert(P.rows() == cost_t_mat.rows() + cost_rf_mat.rows());
    assert(q.rows() == cost_t_vec.rows() + cost_rf_vec.rows());
    assert(P.topRightCorner(18,12) == MatrixNd::Zero(18,12));
    assert(P.bottomLeftCorner(12,18) == MatrixNd::Zero(12,18));

    // -----------------------------------------------------------------
    // ------------------------- CONSTRAINTS ---------------------------
    // -----------------------------------------------------------------

    /* ----- Torque Constraints ----- */

    // Torque is given by the equation tau = A*q_ddot + b_g - Jc^T * Fr
    // Hence we get the form:
    //
    //            tau - b_g = [A, -Jc^T] * [q_ddot; Fr]
    //
    // For the floating base joints we have         0 <= tau <= 0       
    // For the rest of the joints we have    -tau_lim <= tau <= +tau_lim

    const double torque_limit = 4; // 7 Nm is stall torque
    VectorNd torque_lower_limit = VectorNd::Zero(NUM_JOINTS);
    VectorNd torque_upper_limit = VectorNd::Zero(NUM_JOINTS);
    torque_lower_limit.head(6) = -b_g_.head(6);
    torque_upper_limit.head(6) = -b_g_.head(6);
    torque_lower_limit.tail(ROBOT_NUM_JOINTS) = -b_g_.tail(ROBOT_NUM_JOINTS) - torque_limit * VectorNd::Ones(ROBOT_NUM_JOINTS);
    torque_upper_limit.tail(ROBOT_NUM_JOINTS) = -b_g_.tail(ROBOT_NUM_JOINTS) + torque_limit * VectorNd::Ones(ROBOT_NUM_JOINTS);

    // Fill in the A matrix of the form [A, -Jc^T]
    MatrixNd torque_limit_mat(NUM_JOINTS, NUM_JOINTS + 12);
    torque_limit_mat << massMat_, -Jc_.transpose();

    /* ----- Reaction force constraints ----- */

    // For lateral reaction forces (Contact wrench cone constraint):
    // -Fr_z*mu <= Fr_x <= Fr_z*mu   which is equivalent to    -inf <= Fr_x - Fr_z*mu < 0  AND  0 <= Fr_x + Fr_z*mu <= inf
    // -Fr_z*mu <= Fr_y <= Fr_z*mu   which is equivalent to    -inf <= Fr_y - Fr_z*mu < 0  AND  0 <= Fr_y + Fr_z*mu <= inf
    //
    // For normal reaction forces:
    // 0 <= Fr_z <= rf_z_max     (l = u = 0 for feet commanded to swing or floating)

    double rf_z_max = 100; // Max normal reaction force
    double rf_z_min = 1.5; // Min normal reaction force
    MatrixNd reaction_force_mat = MatrixNd::Zero(20, NUM_JOINTS + 12); // Block matrix to store inequality matrix 20x30
    MatrixNd A_fr = MatrixNd::Zero(20,12); // Inequality matrix for reaction forces (12 for Fr_z, 8 for Fr_x/Fr_z)
    VectorNd reaction_force_lower_limit = VectorNd::Zero(20); // Min reaction force (zero for normal reaction forces)
    VectorNd reaction_force_upper_limit = VectorNd::Zero(20); // Max reaction force 

    // Matrix for reaction force constraints. // Indexing ({x,y}, constraint_j, leg_k)
    // Leg 1 
    A_fr(0,0) = 1; // x_1_1
    A_fr(0,2) = -mu;
    A_fr(1,0) = 1; // x_2_1
    A_fr(1,2) = mu;
    A_fr(2,1) = 1; // y_1_1
    A_fr(2,2) = -mu;
    A_fr(3,1) = 1; // y_2_1
    A_fr(3,2) = mu;

    // Leg 2
    A_fr(4,3) = 1;  // x_1_2
    A_fr(4,5) = -mu;
    A_fr(5,3) = 1;  // x_2_2
    A_fr(5,5) = mu;
    A_fr(6,4) = 1;  // y_1_2
    A_fr(6,5) = -mu;
    A_fr(7,4) = 1;  // y_2_2
    A_fr(7,5) = mu;

    // Leg 3
    A_fr(8,6) = 1;  // x_1_3
    A_fr(8,8) = -mu;
    A_fr(9,6) = 1;  // x_2_3
    A_fr(9,8) = mu;
    A_fr(10,7) = 1;  // y_1_3
    A_fr(10,8) = -mu;
    A_fr(11,7) = 1;  // y_2_3
    A_fr(11,8) = mu;

    // Leg 4
    A_fr(12,9) = 1;  // x_1_4
    A_fr(12,11) = -mu;
    A_fr(13,9) = 1;  // x_2_4
    A_fr(13,11) = mu;
    A_fr(14,10) = 1;  // y_1_4
    A_fr(14,11) = -mu;
    A_fr(15,10) = 1;  // y_2_4
    A_fr(15,11) = mu;
    
    // For constraints on Fr_z
    A_fr(16,2) = 1;
    A_fr(17,5) = 1;
    A_fr(18,8) = 1;
    A_fr(19,11) = 1;

    // Min for cone constraints
    reaction_force_lower_limit(0)  = -OSQP_INFTY;
    reaction_force_lower_limit(1)  = 0;
    reaction_force_lower_limit(2)  = -OSQP_INFTY;
    reaction_force_lower_limit(3)  = 0;
    reaction_force_lower_limit(4)  = -OSQP_INFTY;
    reaction_force_lower_limit(5)  = 0;
    reaction_force_lower_limit(6)  = -OSQP_INFTY;
    reaction_force_lower_limit(7)  = 0;
    reaction_force_lower_limit(8)  = -OSQP_INFTY;
    reaction_force_lower_limit(9)  = 0;
    reaction_force_lower_limit(10) = -OSQP_INFTY;
    reaction_force_lower_limit(11) = 0;
    reaction_force_lower_limit(12) = -OSQP_INFTY;
    reaction_force_lower_limit(13) = 0;
    reaction_force_lower_limit(14) = -OSQP_INFTY;
    reaction_force_lower_limit(15) = 0;

    // Max for cone constraints
    reaction_force_upper_limit(0)  = 0;
    reaction_force_upper_limit(1)  = OSQP_INFTY;
    reaction_force_upper_limit(2)  = 0;
    reaction_force_upper_limit(3)  = OSQP_INFTY;
    reaction_force_upper_limit(4)  = 0;
    reaction_force_upper_limit(5)  = OSQP_INFTY;
    reaction_force_upper_limit(6)  = 0;
    reaction_force_upper_limit(7)  = OSQP_INFTY;
    reaction_force_upper_limit(8)  = 0;
    reaction_force_upper_limit(9)  = OSQP_INFTY;
    reaction_force_upper_limit(10) = 0;
    reaction_force_upper_limit(11) = OSQP_INFTY;
    reaction_force_upper_limit(12) = 0;
    reaction_force_upper_limit(13) = OSQP_INFTY;
    reaction_force_upper_limit(14) = 0;
    reaction_force_upper_limit(15) = OSQP_INFTY;

    // Max normal reaction force
    for (int i=0; i<4; i++){
        if (!feet_in_contact_[i]){
            // Constrain reaction force to zero for floating feet
            reaction_force_upper_limit(i+16) = 0; 
            reaction_force_lower_limit(i+16) = 0; 
        }
        else{
            // Constrain reaction force to [rf_z_min, rf_z_max] for contacting feet
            reaction_force_upper_limit(i+16) = rf_z_max;
            reaction_force_lower_limit(i+16) = rf_z_min; 
        }
    }

    //Debug: print contacts in order (BL, BR, FL, FR)
    // cout << "Feet contacts : {";
    // for (bool b : feet_in_contact_){
    //     cout << b << ", ";
    // }
    // cout << "}" << endl;

    reaction_force_mat.rightCols(12) = A_fr;

    A.topRows(NUM_JOINTS) = torque_limit_mat;
    A.bottomRows(20) = reaction_force_mat;

    l.head(NUM_JOINTS) = torque_lower_limit;
    u.head(NUM_JOINTS) = torque_upper_limit;
    l.tail(20) = reaction_force_lower_limit;
    u.tail(20) = reaction_force_upper_limit;
}

void PupperWBC::setupOSQP(int n, int m, MatrixNd &P, c_float  *q, MatrixNd &A, c_float  *lb, c_float  *ub){
    //Convert matrices into csc form
    vector<c_float> P_x, A_x;
    vector<c_int>   P_p, P_i, A_p, A_i;
    convertEigenToCSCSparsePat_(P, P_x, P_p, P_i, true);
    convertEigenToCSCSparsePat_(A, A_x, A_p, A_i);

    // Exitflag
    c_int exitflag = 0;

    // Workspace structures
    OSQPSettings  *settings = QP_settings_.get();
    OSQPData      *data     = QP_data_.get();

    // Populate data
    if (data) {
        data->n = n;
        data->m = m;
        data->P = csc_matrix(n, n, P_p.back(), P_x.data(), P_i.data(), P_p.data());
        data->q = q;
        data->A = csc_matrix(m, n, A_p.back(), A_x.data(), A_i.data(), A_p.data());
        data->l = lb;
        data->u = ub;
    }

    // Define solver settings as default
    if (settings) {
        osqp_set_default_settings(settings);
        settings->eps_rel = 0.0;       // Change relative tolerance to handle large costs
        settings->eps_prim_inf = 1e-2; // Primal infeasibility tolerance
        settings->eps_dual_inf = 1e-2; // Dual infeasibility tolerance
        // settings->polish = 1; // For high quality solution
        settings->verbose = 0;   // Print information? 
        settings->scaling = 10; // Number of scaling iterations - could speed up solver and avoid failures but increases update time significantly  (20)
    }

    // Setup workspace
    exitflag = osqp_setup(&work_, data, settings);
    if (exitflag != 0){
        string message = "OSQP Setup failed with code: " + std::to_string(exitflag);
        throw(std::runtime_error(message));
    }
}

VectorNd PupperWBC::solveQP(int n, int m, MatrixNd &P, c_float *q, MatrixNd &A, c_float *lb, c_float *ub){
    static uint count = 0;
    count += 1;
    if (count == 1){
        setupOSQP(n, m, P, q, A, lb, ub);
        // Change run limit setting (after setup)
        work_->settings->time_limit = 0.001;
    }

    //Convert matrices into csc form
    vector<c_float> P_x, A_x;
    vector<c_int>   P_p, P_i, A_p, A_i;
    convertEigenToCSCSparsePat_(P, P_x, P_p, P_i, true);
    convertEigenToCSCSparsePat_(A, A_x, A_p, A_i);

    // Exitflag
    c_int exitflag = 0;

    osqp_update_lin_cost(work_, q);
    osqp_update_bounds(work_, lb, ub);
    exitflag = osqp_update_P(work_, P_x.data(), OSQP_NULL, P_x.size());
    
    if (exitflag != 0){
        string message = "osqp_update_P failed with code: " + std::to_string(exitflag);
        throw(std::runtime_error(message));
    }

    osqp_update_A(work_, A_x.data(), OSQP_NULL, A_x.size());

    // Solve Problem
    osqp_solve(work_);

    int solve_status = work_->info->status_val;
    if (solve_status != 1 && solve_status != 2 && solve_status != -2 && solve_status != -6){
        string message = "OSQP Solve failed with code: " + std::to_string(solve_status);
        throw(std::runtime_error(message));
        // TODO: Engage motor braking
    }
    if (solve_status == -2){
        cout << "OSQP Solve error: " << std::to_string(solve_status) << endl;
    }
    // QP solution vector [q_ddot;Fr]
    VectorNd sol(NUM_JOINTS + 12);
    std::copy(work_->solution->x, work_->solution->x + sol.size(), sol.data());

    // Log solver run time and status
    float solve_status_f = solve_status;
    np_logger.logScalars({"run_ms"},{work_->info->run_time*1000.0f});
    np_logger.logScalars({"update_ms"},{work_->info->update_time*1000.0f});
    np_logger.logScalars({"solve_ms"},{work_->info->solve_time*1000.0f});
    np_logger.logScalars({"setup_ms"},{work_->info->setup_time*1000.0f});
    np_logger.logScalars({"solve_code"},{solve_status_f});
    np_logger.logScalars({"timelimit"},{work_->settings->time_limit*1000.0f});
    return sol;
}


// VectorNd PupperWBC::solveQP(int n, int m, MatrixNd &P, c_float  *q, MatrixNd &A, c_float  *lb, c_float  *ub){
//     // Old version that setup osqp every iteration
//     //Convert matrices into csc form
//     vector<c_float> P_x, A_x;
//     vector<c_int>   P_p, P_i, A_p, A_i;
//     convertEigenToCSC_(P, P_x, P_p, P_i, true);
//     convertEigenToCSC_(A, A_x, A_p, A_i);

//     // Exitflag
//     c_int exitflag = 0;

//     // Workspace structures
//     OSQPWorkspace *work;
//     OSQPSettings  *settings = QP_settings_.get();
//     OSQPData      *data     = QP_data_.get();

//     // Populate data
//     if (data) {
//         data->n = n;
//         data->m = m;
//         data->P = csc_matrix(n, n, P_p.back(), P_x.data(), P_i.data(), P_p.data());
//         data->q = q;
//         data->A = csc_matrix(m, n, A_p.back(), A_x.data(), A_i.data(), A_p.data());
//         data->l = lb;
//         data->u = ub;
//     }

//     // Define solver settings as default
//     if (settings) {
//         osqp_set_default_settings(settings);
//         settings->eps_rel = 0.0;       // Change relative tolerance to handle large costs
//         settings->eps_prim_inf = 1e-2; // Primal infeasibility tolerance
//         settings->eps_dual_inf = 1e-2; // Dual infeasibility tolerance
//         // settings->polish = 1; // For high quality solution
//         settings->verbose = 1;   // Print information? 
//         settings->scaling = 75; // Number of scaling iterations - could speed up solver and avoid failures (20)
//     }

//     // Setup workspace
//     exitflag = osqp_setup(&work, data, settings);

//     // Solve Problem
//     osqp_solve(work);

//     if (exitflag != 0){
//         string message = "OSQP Setup failed with code: " + std::to_string(exitflag);
//         throw(std::runtime_error(message));
//         // TODO: Engage motor braking
//     }
//     if (work->info->status_val != 1 && work->info->status_val != 2 && work->info->status_val != -2){
//         string message = "OSQP Solve failed with code: " + std::to_string(work->info->status_val);
//         throw(std::runtime_error(message));
//         // TODO: Engage motor braking
//     }
//     if (work->info->status_val == -2){
//         cout << "OSQP Solve error: " << std::to_string(work->info->status_val) << endl;
//     }

//     // Cleanup
//     if (data) {
//         if (data->A) c_free(data->A);
//         if (data->P) c_free(data->P);
//     }

//     // QP solution vector [q_ddot;Fr]
//     VectorNd sol(NUM_JOINTS + 12);
//     std::copy(work->solution->x, work->solution->x + sol.size(), sol.data());
//     return sol;
// }

void PupperWBC::convertEigenToCSCSparsePat_(const MatrixNd &P, vector<c_float> &P_x, vector<c_int> &P_p, vector<c_int> &P_i, bool triup){
    // Convert Eigen types to CSC used in OSQP solver
    // Clear any existing data from the vectors
    P_x.clear();
    P_i.clear();

    P_p.clear();
    P_p.push_back(0);
    
    const int num_rows = P.rows();
    const int num_cols = P.cols();
    for (Eigen::Index c = 0; c < num_cols; c++){
        // Look through the matrix column by column
        const double* col = P.col(c).data();

        // Iterate through the column to look for non-zero elements
        for (int i = 0; i < num_rows; i++){
            if (triup and P_sparsity_.at(i).at(c) == true or not triup and A_sparsity_.at(i).at(c) == true){
                // Store the value of the element in P_x and its row index in P_i
                P_x.push_back(col[i]);
                P_i.push_back(i);
            }

            if (triup and i == c) break;
        }

        P_p.push_back(P_x.size());
    }
}

void PupperWBC::convertEigenToCSC_(const MatrixNd &P, vector<c_float> &P_x, vector<c_int> &P_p, vector<c_int> &P_i, bool triup){
    // Convert Eigen types to CSC used in OSQP solver
    // Clear any existing data from the vectors
    P_x.clear();
    P_i.clear();

    P_p.clear();
    P_p.push_back(0);
    
    const int num_rows = P.rows();
    const int num_cols = P.cols();
    for (Eigen::Index c = 0; c < num_cols; c++){
        // Look through the matrix column by column
        const double* col = P.col(c).data();

        // Iterate through the column to look for non-zero elements
        for (int i = 0; i < num_rows; i++){
            if (col[i] != 0){
                // Store the value of the element in P_x and its row index in P_i
                P_x.push_back(col[i]);
                P_i.push_back(i);
            }

            if (triup and i == c) break;
        }

        P_p.push_back(P_x.size());
    }
}

double PupperWBC::now(){
    // returns time in seconds 
    return time_now_; // in s
}
