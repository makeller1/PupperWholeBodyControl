#include "pupper.hpp"
#include "workstation/PupperModel.h"
#include "workstation/PupperUrdfString.hpp"
#include <iostream>

using std::cout;
using std::endl;
using std::array;
using std::vector;
using std::string;

static double angleDiff(double angle1, double angle2){
    return fmod(angle1 - angle2 + M_PI, 2*M_PI) - M_PI;
}

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

    // Task for Body center of mass to be 10cm high // range .02
    static Task CoM_Position_Task;
    CoM_Position_Task.type    = BODY_POS;
    CoM_Position_Task.body_id = "bottom_PCB";
    CoM_Position_Task.task_weight = 0; // 10
    // CoM_Position_Task.active_targets = {false, false, true};    // only account for z-position
    CoM_Position_Task.active_targets = {false, false, true};   
    CoM_Position_Task.pos_target << 0, 0, 0.09; // .10
    CoM_Position_Task.Kp = 0;//400;
    CoM_Position_Task.Kd = 0;//200
    CoM_Position_Task.x_ddot_ff << 0, 0, 9.81; // feed forward acceleration - counter gravity

    // Task for Body center of mass to be flat // .001
    static Task CoM_Orientation_Task;
    CoM_Orientation_Task.type    = BODY_ORI;
    CoM_Orientation_Task.body_id = "bottom_PCB";
    CoM_Orientation_Task.task_weight = 0; // 100;
    CoM_Orientation_Task.quat_target = Eigen::Quaternion<double>::Identity();
    CoM_Orientation_Task.Kp = 1000;//1000;
    CoM_Orientation_Task.Kd = 200;

    // static Task JointPositionTask; // .01
    // JointPositionTask.type = JOINT_POS;
    // JointPositionTask.task_weight = 1; //0.1;
    // JointPositionTask.joint_target = VectorNd::Zero(12);
    // JointPositionTask.active_targets = {true, false, false, true, false, false, true, false, false, true, false, false};
    // JointPositionTask.Kp = 200;
    // JointPositionTask.Kd = 200;

    static Task JointPositionTask; // .01
    JointPositionTask.type = JOINT_POS;
    JointPositionTask.task_weight = 100; 
    JointPositionTask.joint_target <<   0.0,  M_PI_4,  M_PI_2, 
                                        0.0, -M_PI_4, -M_PI_2,
                                        0.0,  M_PI_4,  M_PI_2,
                                        0.0, -M_PI_4, -M_PI_2;
    JointPositionTask.active_targets = {true, true, true, true, true, true, true, true, true, true, true, true};
    JointPositionTask.Kp = 5000;
    JointPositionTask.Kd = 500; //2000

    foot_pos_Kp_ = 10;
    foot_pos_Kd_ = 5;
    foot_pos_w_  = 3;
    float_pos_Kp_ = 50; //50
    float_pos_Kd_ = 0; // 0
    float_pos_w_  = 50;

    // Keep the front left foot in place
    static Task FLFootTask;
    FLFootTask.type = BODY_POS;
    FLFootTask.body_id = "front_left_foot";
    FLFootTask.task_weight = foot_pos_w_;
    // FLFootTask.active_targets = {true, true, false};  // We'll let the COM task take care of heigh
    FLFootTask.active_targets = {false, false, false};  // We'll let the COM task take care of heigh
    FLFootTask.pos_target << 0.08, 0.075, -0.1;
    FLFootTask.Kp = foot_pos_Kp_;
    FLFootTask.Kd = foot_pos_Kd_;

    // Keep the front right foot in place
    static Task FRFootTask;
    FRFootTask.type = BODY_POS;
    FRFootTask.body_id = "front_right_foot";
    FRFootTask.task_weight = foot_pos_w_;
    FRFootTask.active_targets = {false, false, false};  // We'll let the COM task take care of heigh
    FRFootTask.pos_target << 0.08, -0.065, -0.1;
    FRFootTask.Kp = foot_pos_Kp_;
    FRFootTask.Kd = foot_pos_Kd_;

    // Keep the back left foot in place
    static Task BLFootTask;
    BLFootTask.type = BODY_POS;
    BLFootTask.body_id = "back_left_foot";
    BLFootTask.task_weight = foot_pos_w_;
    // BLFootTask.active_targets = {true, true, false};  // We'll let the COM task take care of heigh
    BLFootTask.active_targets = {false, false, false};  // We'll let the COM task take care of heigh
    BLFootTask.pos_target << -0.11, 0.075, -0.1;
    BLFootTask.Kp = foot_pos_Kp_;
    BLFootTask.Kd = foot_pos_Kd_;

    // Keep the back right foot in place
    static Task BRFootTask;
    BRFootTask.type = BODY_POS;
    BRFootTask.body_id = "back_right_foot";
    BRFootTask.task_weight = foot_pos_w_;
    // BRFootTask.active_targets = {true, true, false};  // We'll let the COM task take care of heigh
    BRFootTask.active_targets = {false, false, false};  // We'll let the COM task take care of heigh
    BRFootTask.pos_target << -0.11, -0.065, -0.1;
    BRFootTask.Kp = foot_pos_Kp_;
    BRFootTask.Kd = foot_pos_Kd_;

    WBC_.addTask("COM_POSITION", &CoM_Position_Task);
    WBC_.addTask("COM_ORIENTATION", &CoM_Orientation_Task);
    WBC_.addTask("JOINT_ANGLES", &JointPositionTask);

    // Foot position tasks for standstill
    WBC_.addTask("FRONT_LEFT_FOOT_POSITION", &FLFootTask);
    WBC_.addTask("FRONT_RIGHT_FOOT_POSITION", &FRFootTask);
    WBC_.addTask("BACK_LEFT_FOOT_POSITION", &BLFootTask);
    WBC_.addTask("BACK_RIGHT_FOOT_POSITION", &BRFootTask);
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
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&PupperPlugin::onUpdate, this));

    // Set up update rate variables
    update_interval_  = 0.002;  // 500Hz
    last_update_time_ = 0.0;

    // Set up the connection to Gazebo topics
    connection_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    connection_node_->Init("pupper_world");
    contact_sub_ = connection_node_->Subscribe("~/physics/contacts", &PupperPlugin::contactCallback_, this);

    // TODO: Create subscriber to simulation time, -> store in pupper as private class variable, pass to WBC on update.
    stats_sub_ = connection_node_->Subscribe("~/world_stats", &PupperPlugin::statsCallback_, this);

    start_time = simtime_;

    cout << "Loaded successfully" << endl;
}




// =========================================================================================
// ---------------------------------       MAIN LOOP       ---------------------------------
// =========================================================================================

// Called on every simulation time step
void PupperPlugin::onUpdate(){
    static vector<float> init_angles = { 0.0,  0.0,  0.0, 
                                         0.0,  0.0,  0.0, 
                                         0.0,  0.0,  0.0,  
                                         0.0,  0.0,  0.0};

    double now = simtime_;

    // // Oscillate COM height task
    // float target_height = 0.12 + 0.02*sin(0.5*now); // 0.5 Hz (2 sec)
    // WBC_.getTask("COM_POSITION")->pos_target.z() = target_height; 

    // // Oscillate left/right tilt
    // float target_roll = M_PI/12 * sin(0.5 * now); // 0.5 HZ (2 sec)
    // WBC_.getTask("COM_ORIENTATION")->quat_target = Eigen::AngleAxisd(target_roll, Eigen::Vector3d::UnitX());

    // First 2 seconds
    if (now - start_time < 4e3){
        setJointPositions(init_angles);
    }
    // // Fall (hopefully) gracefully
    // else if (now - start_time < 4e5){
    //     std::fill(control_torques_.begin(), control_torques_.end(), 0);
    // }
    //Manage publisher update rate
    else if (now - last_update_time_ > update_interval_){
        // Get the robot state from the simulation
        updateBody_();
        updateJoints_();
        // Copy that robot state into the Whole Body Controller
        updateController_();
        // Calculate control commands
        control_torques_ = WBC_.calculateOutputTorque();
        last_update_time_ = simtime_;
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
        all_joints_[i]->SetForce(0, control_torques_[i]);
    }
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
        cout<<"joint vel "<< (int) i << ": " << all_joints_[i]->GetVelocity(0) << endl;
    }
}

// Update the body center of mass position and orienation
void PupperPlugin::updateBody_(){
    auto body_pose = model_->WorldPose();
    auto body_vel  = model_->WorldLinearVel();

    body_COM_[0] = body_pose.Pos().X();
    body_COM_[1] = body_pose.Pos().Y();
    body_COM_[2] = body_pose.Pos().Z();

    cout << "Z measured  : " << body_COM_[2] << endl;
    cout << "Z calculated: " << WBC_.getCalculatedHeight() << endl;

    body_quat_.x() = body_pose.Rot().X();
    body_quat_.y() = body_pose.Rot().Y();
    body_quat_.z() = body_pose.Rot().Z();
    body_quat_.w() = body_pose.Rot().W();

    body_COM_vel_.x() = body_vel.X();
    body_COM_vel_.y() = body_vel.Y();
    body_COM_vel_.z() = body_vel.Z();
}

// Tell the controller the current state of the robot
void PupperPlugin::updateController_(){
    // WBC_.updateController(joint_positions_, joint_velocities_, body_COM_, body_quat_, feet_in_contact_, simtime_/1e3);
    WBC_.updateController(joint_positions_, joint_velocities_, body_COM_, body_quat_, {true,true,true,true},simtime_/1e3);
    WBC_.updateBodyPosTask("COM_POSITION", body_COM_);
    WBC_.updateBodyOriTask("COM_ORIENTATION", body_quat_);
    VectorNd jointPos(12);
    std::copy(joint_positions_.data(), joint_positions_.data() + 12, jointPos.data());
    WBC_.updateJointTask("JOINT_ANGLES", jointPos);
    WBC_.updateBodyPosTask("BACK_LEFT_FOOT_POSITION",   WBC_.getRelativeBodyLocation("back_left_foot"));
    WBC_.updateBodyPosTask("BACK_RIGHT_FOOT_POSITION",  WBC_.getRelativeBodyLocation("back_right_foot"));
    WBC_.updateBodyPosTask("FRONT_LEFT_FOOT_POSITION",  WBC_.getRelativeBodyLocation("front_left_foot"));
    WBC_.updateBodyPosTask("FRONT_RIGHT_FOOT_POSITION", WBC_.getRelativeBodyLocation("front_right_foot"));
    // cout << "COM height to ground = " << body_COM_[2] << endl;
}




// =========================================================================================
// ----------------------------       SUBSCRIBER CALLBACK       ----------------------------
// =========================================================================================

void PupperPlugin::contactCallback_(ConstContactsPtr &_msg){
    
    static const array<string,4> collision_names = {
        "pupper::back_left_lower_link::",
        "pupper::back_right_lower_link:",
        "pupper::front_left_lower_link:",
        "pupper::front_right_lower_link",
    };

    std::fill(feet_in_contact_.begin(), feet_in_contact_.end(), false);
    for (auto C : _msg->contact()){
        if (C.has_collision1())
            for (size_t i = 0; i < 4; i++)
                if (collision_names[i] == C.collision1().substr(0, 30))
                    feet_in_contact_[i] = true;
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

void PupperPlugin::statsCallback_(ConstWorldStatisticsPtr &_msg){
    auto timeMsg = _msg->sim_time(); // get sim time
    common::Time time_ns = timeMsg.nsec(); // get nanoseconds 
    common::Time time_s = timeMsg.sec(); // get seconds 
    simtime_ = time_ns.Double()/1e6 + time_s.Double()*1e3; // ms
}

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(PupperPlugin)

}   // end namespace gazebo
