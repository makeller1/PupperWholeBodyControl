#ifndef _PUPPER_PLUGIN_HH_
#define _PUPPER_PLUGIN_HH_

#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "Eigen/Dense"

#include "workstation/PupperWBC.hpp"

namespace gazebo{

// This order matches that of RBDL: it should not be changed
enum PupperLegs{
    BACK_LEFT_LEG,
    BACK_RIGHT_LEG,
    FRONT_LEFT_LEG,
    FRONT_RIGHT_LEG,
};

//A plugin to control a the Stanford Pupper V3 robot
class PupperPlugin : public ModelPlugin
{
public:
    //Constructor
    PupperPlugin();

    // Destructor
    virtual ~PupperPlugin(){gazebo::transport::fini();}

    // Load function - Called on model creation and is used for setup and initialization
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Called on every timestep of the simulation (control code goes here)
    void onUpdate(const common::UpdateInfo &_info);

    // Apply command to the joints as we would actually do on the robot
    void controlJoints(enum PupperLegs leg, std::vector<float> torques);
    void controlAllJoints(std::vector<float> torques);

    // Debug function
    void setJointPositions(std::vector<float> angles);

private:
    // Model
    physics::ModelPtr model_;                    // Pointer to the model in Gazebo
    physics::JointPtr front_left_joints_[3];     // Array of joints on the front left leg
    physics::JointPtr front_right_joints_[3];    // Array of joints on the front right leg
    physics::JointPtr back_left_joints_[3];      // Array of joints on the back left leg
    physics::JointPtr back_right_joints_[3];     // Array of joints on the back right leg
    physics::JointPtr all_joints_[12];           // Order: BL{hip,shoulder,elbow}, BR{hip,shoulder,elbow}, FL{hip,shoulder,elbow}, FR{hip,shoulder,elbow}
    std::array<bool, 4> feet_in_contact_;        // Order: BL, BR, FL, FR

    // Gazebo connections
    event::ConnectionPtr updateConnection_;        // Event connection between the Gazebo simulation and this plugin
    gazebo::transport::NodePtr connection_node_;   // Subscribes to gazebo update topics (used for contacts)
    gazebo::transport::SubscriberPtr contact_sub_; // Used to subscribe to a specific topic (contacts)
    void contactCallback_(ConstContactsPtr &_msg);

    // Robot State
    Eigen::VectorXd joint_positions_;
    Eigen::VectorXd joint_velocities_;
    Eigen::VectorXd body_COM_;
    Eigen::Quaterniond body_quat_;
    Eigen::Vector3d body_COM_vel_;
    void updateJoints_();
    void updateBody_();

    // Control
    PupperWBC WBC_;
    double simtime_;                            // ms current simulation time
    double last_update_time_;                   // ms used to keep track of update rate
    double update_interval_;                    // ms between each control update loop
    // common::Time last_update_time_;             // Used to keep track of update rate
    // common::Time update_interval_;              // Seconds between each control update loop
    std::array<float, ROBOT_NUM_JOINTS> control_torques_;
    void updateController_();
    void applyTorques_();
    float foot_pos_Kp_;                         // Contacting foot task parameter
    float foot_pos_Kd_;                         // Contacting foot task parameter
    float foot_pos_w_;                          // Contacting foot task parameter

    float float_pos_Kp_;                         // floating foot task parameter
    float float_pos_Kd_;                         // floating foot task parameter
    float float_pos_w_;                          // floating foot task parameter

    double start_time;                           // start time of simulation ms
};


} // end namespace gazebo


#endif