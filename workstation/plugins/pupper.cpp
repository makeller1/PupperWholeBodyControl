#include "pupper.hpp"
#include "workstation/PupperModel.h"
#include "workstation/PupperUrdfString.hpp"
#include "workstation/StateEstimation.hpp"
#include <iostream>
#include <math.h>
#include <string>
#include <chrono>

// #define DEBUG_MODE
// #define MOTOR_LOW_PASS_ACTIVE
// #define MEASUREMENT_NOISE_ACTIVE

using std::cout;
using std::endl;
using std::array;
using std::vector;
using std::string;

static double angleDiff(double angle1, double angle2){
    return fmod(angle1 - angle2 + M_PI, 2*M_PI) - M_PI;
}

// start timer (ms)
double tic(int mode=0) {
    static std::chrono::_V2::system_clock::time_point t_start;
    
    if (mode==0)
        t_start = std::chrono::high_resolution_clock::now();
    else {
        auto t_end = std::chrono::high_resolution_clock::now();
        double t_delta = (t_end-t_start).count()*1E-6;
        std::cout << "Elapsed time is " << t_delta << " ms\n";
        return t_delta;
    }
    return 0.0;
}
// return elapsed time (ms)
double toc() { return tic(1); }

namespace gazebo
{

// =========================================================================================
// -----------------------------------       INIT       ------------------------------------
// =========================================================================================

// Constructor
PupperPlugin::PupperPlugin(){
    // Set contacts to false by default
    std::fill(feet_in_contact_.begin(), feet_in_contact_.end(), false);

    // Initialize the COM quaternion as identity
    body_quat_ = Eigen::Quaterniond::Identity();

    // Load the pupper dynamic model controller
    WBC_.Load(*createPupperModel());

    // Choose goal
    GoalName goal = GoalName::GETUP;
    taskmaster_.setGoal(goal, WBC_, this->feet_in_contact_manual);
}

// Load the model
void PupperPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Safety check
    if (_model->GetJointCount() != 12)
    {
        cout << "Invalid joint count, Pupper plugin not loaded" << endl;
        return;
    }

    // Store the model pointer
    model_ = _model;

    // Get the individual leg joints
    front_left_joints_[0]  = model_->GetJoint("front_left_hip_joint");
    front_left_joints_[1]  = model_->GetJoint("front_left_shoulder_joint");
    front_left_joints_[2]  = model_->GetJoint("front_left_elbow_joint");

    front_right_joints_[0] = model_->GetJoint("front_right_hip_joint");
    front_right_joints_[1] = model_->GetJoint("front_right_shoulder_joint");
    front_right_joints_[2] = model_->GetJoint("front_right_elbow_joint");

    back_right_joints_[0]  = model_->GetJoint("back_right_hip_joint");
    back_right_joints_[1]  = model_->GetJoint("back_right_shoulder_joint");
    back_right_joints_[2]  = model_->GetJoint("back_right_elbow_joint");

    back_left_joints_[0]   = model_->GetJoint("back_left_hip_joint");
    back_left_joints_[1]   = model_->GetJoint("back_left_shoulder_joint");
    back_left_joints_[2]   = model_->GetJoint("back_left_elbow_joint");

    // Also collect all joints into a single array
    for (int i = 0; i < 3; i++){
        all_joints_[i + 3*BACK_LEFT_LEG]   = back_left_joints_[i];
        all_joints_[i + 3*BACK_RIGHT_LEG]  = back_right_joints_[i];
        all_joints_[i + 3*FRONT_LEFT_LEG]  = front_left_joints_[i];
        all_joints_[i + 3*FRONT_RIGHT_LEG] = front_right_joints_[i];
    }

    for (int i = 0; i < 12; i++){
        control_torques_[i] = 0;
    }

    // Resize joint vectors
    joint_positions_  = Eigen::VectorXd::Zero(ROBOT_NUM_JOINTS);
    joint_velocities_ = Eigen::VectorXd::Zero(ROBOT_NUM_JOINTS);
    body_COM_         = Eigen::VectorXd::Zero(3);

    //Connect plugin to Gazebo world instance
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&PupperPlugin::onUpdate, this, std::placeholders::_1));
    
    // Set up update rate variables
    update_interval_  = 1;  // (ms) 1000Hz
    last_update_time_ = 0.0;

    // Set up the connection to Gazebo topics
    connection_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    connection_node_->Init("pupper_world");
    contact_sub_ = connection_node_->Subscribe("~/physics/contacts", &PupperPlugin::contactCallback_, this);

    #ifdef DEBUG_MODE
        // Print model information
        double mass_total = 0.0;
        cout << "Gazebo: \n";
        
        for (auto x : model_->GetLinks()){
            cout << "Link name: " << x->GetName() << endl;
            cout << "Link id: " << x->GetId() << endl;
            auto inertia = x->GetInertial();
            cout << "Mass: " << inertia->Mass() << "\n";
            cout << "COM: " << inertia->CoG() << "\n";
            cout << "Inertia:\n\n";
            cout << inertia->IXX() << " " << inertia->IXY() << " " << inertia->IXZ() << endl;
            cout << inertia->IXY() << " " << inertia->IYY() << " " << inertia->IYZ() << endl;
            cout << inertia->IXZ() << " " << inertia->IYZ() << " " << inertia->IZZ() << endl;
            mass_total += inertia->Mass();
        }
        cout << "Model mass: " << mass_total << endl;
        Eigen::Vector3d COM_total;
    #endif
}


// =========================================================================================
// ---------------------------------       MAIN LOOP       ---------------------------------
// =========================================================================================

// Called on every simulation time step
void PupperPlugin::onUpdate(const common::UpdateInfo &_info){
    
    static double dt_wbc = 0.0; // ms elapsed time to run wbc (update model, update problem, and solve)
    simtime_ = _info.simTime.Double()*1e3;


    // First 5 seconds
    if (simtime_ < taskmaster_.time_init_ms){
        setJointPositions(taskmaster_.init_angles);
    }
    // Fall (hopefully) gracefully
    else if (simtime_ < taskmaster_.time_fall_ms){
        std::fill(control_torques_.begin(), control_torques_.end(), 0);
    }
    //Manage publisher update rate and run WBC control loop
    // else if (simtime_ - last_update_time_ >= std::max(update_interval_,dt_wbc)){
    else if (simtime_ - last_update_time_ >= update_interval_){
        static double k = 1;
        static double mean_time = 0.0;

        // Update tasks to accomplish goal
        taskmaster_.updateGoalTasks(WBC_, simtime_);

        // Get the robot state from the simulation
        updateBody_();
        updateJoints_();

        tic();
        // Copy that robot state into the Whole Body Controller
        updateController_();

        // Calculate control commands
        control_torques_ = WBC_.calculateOutputTorque();

        // Print WBC diagnostics
        WBC_.printDiag(); 
        dt_wbc = toc();
        mean_time = (k-1)/(k)*mean_time + (1/k)*dt_wbc;
        cout << "Mean run time: " << mean_time << "\n \n \n " << endl;

        // Log time required to run wbc
        WBC_.np_logger.logScalars({"wbc_ms"},{dt_wbc});

        // Log gazebo contact forces 
        logContactForces_();

        if ((int)k == 1){
            // do not delay for initialization
            dt_wbc = 0.0;
        }
        k = k + 1;
        last_update_time_ = simtime_;
    }
    // Save data to numpy format every second
    // if ((int)simtime_ % 1000 == 0){
    if ((int)simtime_ % 20 == 0){
        WBC_.np_logger.saveData();
    }
    // This needs to be outside the loop or else the joints will go dead on non-update iterations
    applyTorques_();
}



// =========================================================================================
// ----------------------------       CONTROL SIMULATION       -----------------------------
// =========================================================================================

// Control the joints on one leg
void PupperPlugin::controlJoints(enum PupperLegs leg, vector<float> torques){
    switch(leg){
    case FRONT_LEFT_LEG:
        front_left_joints_[0]->SetForce(0, torques[0]);
        front_left_joints_[1]->SetForce(0, torques[1]);
        front_left_joints_[2]->SetForce(0, torques[2]);
        break;

    case FRONT_RIGHT_LEG:
        front_right_joints_[0]->SetForce(0, torques[0]);
        front_right_joints_[1]->SetForce(0, torques[1]);
        front_right_joints_[2]->SetForce(0, torques[2]);
        break;

    case BACK_RIGHT_LEG:
        back_right_joints_[0]->SetForce(0, torques[0]);
        back_right_joints_[1]->SetForce(0, torques[1]);
        back_right_joints_[2]->SetForce(0, torques[2]);
        break;

    case BACK_LEFT_LEG:
        back_left_joints_[0]->SetForce(0, torques[0]);
        back_left_joints_[1]->SetForce(0, torques[1]);
        back_left_joints_[2]->SetForce(0, torques[2]);
        break;
    }
}


// Control all joints on the robot 
void PupperPlugin::controlAllJoints(vector<float> torques){
    for (int i = 0; i < 12; i++){
        all_joints_[i]->SetForce(0, torques[i]);
    }
} 


// Set joint positions through Gazebo (useful for debugging)
void PupperPlugin::setJointPositions(vector<float> angles){
    for (int i = 0; i < 12; i++){
        double error = angleDiff(angles[i], all_joints_[i]->Position(0));
        control_torques_[i] = 10*error;
    }
}


// Update the simulation with the current control torques
void PupperPlugin::applyTorques_(){
    for (int i = 0; i < 12; i++){
        float torque_i = control_torques_.at(i);
        // #ifdef MOTOR_LOW_PASS_ACTIVE
        //     if (simtime_ >= taskmaster_.time_fall_ms)
        //         torque_i = applyLowPass_(torque_i,i);
        // #endif
        all_joints_[i]->SetForce(0, torque_i);
        // all_joints_[i]->SetForce(0, control_torques_[i]);

        // Check for nan
        if (std::isnan(torque_i)){
            throw std::runtime_error("nan in torque command");
        }
    }
}

float PupperPlugin::applyLowPass_(float torque, int i){
    static std::array<float, 12> low_pass_torques = {0,0,0,0,0,0,0,0,0,0,0,0};
    float dt = 0.001; // Seconds Gazebo physics simulation update rate
    float motor_bandwidth = 50; // Hz motor bandwidth
    float alpha = exp(-50*2*M_PI*dt);
    low_pass_torques.at(i) = (alpha)*low_pass_torques.at(i) + (1-alpha)*torque;
    return low_pass_torques.at(i);
}

// =========================================================================================
// ----------------------------       UPDATE INFORMATION       -----------------------------
// =========================================================================================


// Get joint measurements from gazebo
void PupperPlugin::updateJoints_(){
    for (uint8_t i = 0; i < 12; i++){
        joint_positions_[i] = (float) all_joints_[i]->Position();
        joint_velocities_[i] = (float) all_joints_[i]->GetVelocity(0);
        // physics::JointWrench joint_wrench = all_joints_[i]->GetForceTorque(0);
        // joint_torques[i] = (float)joint_wrench.body1Torque.Z();
        // cout<<"joint vel "<< (int) i << ": " << all_joints_[i]->GetVelocity(0) << endl;
    }
}

// Update the body center of mass position and orienation from Gazebo measurements
void PupperPlugin::updateBody_(){
    auto body_pose = model_->WorldPose();
    auto body_lin_vel  = model_->WorldLinearVel();

    Eigen::Quaterniond world_quaternion;
    world_quaternion.x() = body_pose.Rot().X();
    world_quaternion.y() = body_pose.Rot().Y();
    world_quaternion.z() = body_pose.Rot().Z();
    world_quaternion.w() = body_pose.Rot().W();

    static bool initial_update = true;
    static Eigen::Quaterniond initial_quaterion_;
    if (initial_update == true){
        initial_quaterion_ = world_quaternion;
        initial_update = false;
    }
    // Zero the orientation to the initial state
    body_quat_ = initial_quaterion_.conjugate() * world_quaternion;
    // Don't zero the orientation
    // body_quat_ = world_quaternion;

    // Rotate gazebo ang vel to body frame
    Eigen::Vector3d world_ang_vel;
    auto R_world_to_body = world_quaternion.toRotationMatrix();

    world_ang_vel.x() = model_->WorldAngularVel().X();
    world_ang_vel.y() = model_->WorldAngularVel().Y();
    world_ang_vel.z() = model_->WorldAngularVel().Z();

    auto body_ang_vel = R_world_to_body.transpose()*world_ang_vel;

    body_COM_ang_vel_.x() = body_ang_vel.x();
    body_COM_ang_vel_.y() = body_ang_vel.y();
    body_COM_ang_vel_.z() = body_ang_vel.z();

    body_COM_lin_vel_.x() = body_lin_vel.X();
    body_COM_lin_vel_.y() = body_lin_vel.Y();
    body_COM_lin_vel_.z() = body_lin_vel.Z();

    // Measure body position
    body_COM_[2] = body_pose.Pos().Z();
}

// Tell the controller the current state of the robot
void PupperPlugin::updateController_(){
    body_COM_ang_vel_.setZero(); // Remove effect of body ang. vel. on j_dot_q_dot term
    WBC_.updateController(joint_positions_, joint_velocities_, body_quat_, body_COM_ang_vel_, feet_in_contact_manual,simtime_/1000.0f);

    // Update robot COM pos
    Eigen::Vector2d lateral_pos_est;
    lateral_pos_est = estimateLateralPos(WBC_, body_quat_);
    body_COM_(0) = lateral_pos_est(0);
    body_COM_(1) = lateral_pos_est(1);
    body_COM_(2) = estimateHeight(WBC_, feet_in_contact_manual);

    // Update tasks
    VectorNd jointPos(12);
    std::copy(joint_positions_.data(), joint_positions_.data() + 12, jointPos.data());
    WBC_.updateBodyPosTask("COM_HEIGHT", body_COM_);
    WBC_.updateBodyPosTask("COM_LATERAL_POS", body_COM_);
    WBC_.updateBodyOriTask("COM_ORIENTATION", body_quat_);
    WBC_.updateJointTask("JOINT_ANGLES", jointPos);
    WBC_.updateBodyPosTask("BACK_LEFT_FOOT_POS",   WBC_.calcBodyPosInBaseCoordinates("back_left_foot"));
    WBC_.updateBodyPosTask("BACK_RIGHT_FOOT_POS",  WBC_.calcBodyPosInBaseCoordinates("back_right_foot"));
    WBC_.updateBodyPosTask("FRONT_LEFT_FOOT_POS",  WBC_.calcBodyPosInBaseCoordinates("front_left_foot"));
    WBC_.updateBodyPosTask("FRONT_RIGHT_FOOT_POS", WBC_.calcBodyPosInBaseCoordinates("front_right_foot"));
}

void PupperPlugin::logContactForces_(){
    for (int i = 0; i < 4; i++){
        auto link_ptr = model_->GetLink(link_names_[i]);
        auto link_pose = link_ptr->WorldPose();
        ignition::math::v6::Vector3d force;
        force.Set(contact_forces_body_[i](0), contact_forces_body_[i](1), contact_forces_body_[i](2));
        auto force_world = link_pose.Rot().RotateVector(force);
        contact_forces_world_[i] << force_world.X(), force_world.Y(), force_world.Z();
    }
    WBC_.np_logger.logVectorXd("Fr_gazebo_bl",contact_forces_world_[0]);
    WBC_.np_logger.logVectorXd("Fr_gazebo_br",contact_forces_world_[1]);
    WBC_.np_logger.logVectorXd("Fr_gazebo_fl",contact_forces_world_[2]);
    WBC_.np_logger.logVectorXd("Fr_gazebo_fr",contact_forces_world_[3]);
}

// =========================================================================================
// ----------------------------       SUBSCRIBER CALLBACK       ----------------------------
// =========================================================================================

void PupperPlugin::contactCallback_(ConstContactsPtr &_msg){
    // // Detect feet in contact
    // // Measure reaction forces (unfinished)
    static const array<string,4> collision_names = {
        "pupper::back_left_lower_link::",
        "pupper::back_right_lower_link:",
        "pupper::front_left_lower_link:",
        "pupper::front_right_lower_link"
    };
    // std::fill(feet_in_contact_.begin(), feet_in_contact_.end(), false);
    for (auto C : _msg->contact()){
        if (C.has_collision1())
            for (size_t i = 0; i < 4; i++)
                if (collision_names[i] == C.collision1().substr(0, 30))
                {
            //         feet_in_contact_[i] = true;
                    contact_forces_body_[i].setZero();
                }

            for (auto W : C.wrench()){
                // Read topic in terminal: gz topic -e /gazebo/pupper_world/physics/contacts
                // W.body_1_name().substr
                for (size_t i = 0; i < 4; i++){
                    if (collision_names[i] == W.body_1_name().substr(0, 30)){
                        contact_forces_body_[i] << W.body_1_wrench().force().x(), W.body_1_wrench().force().y(), W.body_1_wrench().force().z();
                        // auto link_ptr = model_->GetLink(link_names_[i]);
                        // cout << "link_ptr is null?: " << std::to_string(link_ptr == NULL) << endl;
                        // auto link_pose = link_ptr->WorldPose();
                        // ignition::math::v6::Vector3d force;
                        // force.Set(W.body_1_wrench().force().x(), W.body_1_wrench().force().y(), W.body_1_wrench().force().z());
                        // auto force_world = link_pose.Rot().RotateVectorReverse(force);
                        // cout << "Body: " << W.body_1_name() << "\n";
                        // cout << "Force world: " << force_world << endl;
                        // contact_forces_world_[i] << force_world.X(), force_world.Y(), force_world.Z();
                    }

                }
                // auto id = W.body_1_id();
                // uint32_t id_int = id;
                // cout << "id from wrench: " << std::to_string(id_int) << endl;
                // // auto link_ptr = model_->GetLinkById(id_int);
                // auto link_ptr = model_->GetLink("front_right_lower_link");
                // cout << "link_ptr is null?: " << std::to_string(link_ptr == NULL) << endl;
                
                // cout << "Body: " << W.body_1_name() << "\n";
                // cout << "Force x:  " << W.body_1_wrench().force().x() << " \n";
                // cout << "Force y: " << W.body_1_wrench().force().y() << " \n";
                // cout << "Force z: " << W.body_1_wrench().force().z() << endl;
                // cout << "Force world: " << force_world << endl;
            }
    }

    // // Set targets for foot tasks dependent on contact
    // static const array<string,4> foot_task_names = {
    //     "BACK_LEFT_FOOT_POSITION",
    //     "BACK_RIGHT_FOOT_POSITION",
    //     "FRONT_LEFT_FOOT_POSITION",
    //     "FRONT_RIGHT_FOOT_POSITION",
    // };
    // for (size_t i = 0; i < 4; i++){
    //     WBC_.getTask(foot_task_names[i])->pos_target[2] = -.15; //std::min(-.02,-WBC_.getCalculatedHeight());
    //     if (feet_in_contact_[i]){
    //         // contacting
    //         WBC_.getTask(foot_task_names[i])->active_targets = {true,true,false};
    //         WBC_.getTask(foot_task_names[i])->Kp = foot_pos_Kp_;
    //         WBC_.getTask(foot_task_names[i])->Kd = foot_pos_Kd_;
    //         WBC_.getTask(foot_task_names[i])->task_weight = foot_pos_w_;
    //     }
    //     else{
    //         // floating
    //         // set foot position task to move to ground
    //         WBC_.getTask(foot_task_names[i])->active_targets = {false,false,true};
    //         WBC_.getTask(foot_task_names[i])->Kp = float_pos_Kp_;
    //         WBC_.getTask(foot_task_names[i])->Kd = float_pos_Kd_;
    //         WBC_.getTask(foot_task_names[i])->task_weight = float_pos_w_;
    //     }
    // }
    

    // //Debug: print contacts in order (BL, BR, FL, FR)
    // cout << "Feet contacts : {";
    // for (bool b : feet_in_contact_){
    //     cout << b << ", ";
    // }
    // cout << "}" << endl;

}

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(PupperPlugin)

}   // end namespace gazebo
