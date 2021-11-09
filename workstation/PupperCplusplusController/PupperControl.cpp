#include <iostream>
#include "ase389/PupperWBC.hpp"
#include "ase389/PupperModel.h"
#include "ase389/PupperUrdfString.hpp"

using std::array;
using std::cout;
using std::endl;
using std::string;

// Set floating precision to 3 sig figs
static Eigen::IOFormat f(3);

// Note: For debug only, function is very inefficient
static void printMatrix(Eigen::MatrixXd A, std::string title = ""){
    
    // Clean up values close to zero
    for (Eigen::Index i = 0; i < A.rows(); i++){
        for (Eigen::Index j = 0; j < A.cols(); j++){
            if (abs(A(i,j)) < 1e-6) A(i,j) = 0;
        }
    }

    // Print
    if (title != "") cout << title << endl;
    cout << A.format(f) << endl;
}


int main(int argc, char** argv){

    auto new_model = createPupperModel();

    PupperWBC Pup;
    Pup.Load(*new_model);

    VectorNd joint_angles(12);
    joint_angles.setZero();
    Eigen::Quaterniond robot_quaternion;
    robot_quaternion.setIdentity();
    Eigen::Vector3d robot_position;
    robot_position.setZero();

    // cout << "Mass of bottom_PCB: " << new_model->mBodies[pcbid].mMass << endl;
    // cout << "Mass of bottom PCB in inertia: " << new_model->I[pcbid].m << endl;

    // Task for Body center of mass to be 10cm high
    Task CoM_Position_Task;
    CoM_Position_Task.body_id = "bottom_PCB";
    CoM_Position_Task.type    = BODY_POS;
    CoM_Position_Task.task_weight = 0.1;
    CoM_Position_Task.active_targets = {false, false, true};    // only account for z-position
    CoM_Position_Task.pos_target << 0, 0, 0.10;
    CoM_Position_Task.Kp = 100;
    CoM_Position_Task.Kd = 0;

    // Task for Body center of mass to be flat
    Task CoM_Orientation_Task;
    CoM_Orientation_Task.body_id = "bottom_PCB";
    CoM_Orientation_Task.type    = BODY_ORI;
    CoM_Orientation_Task.task_weight = 1;
    CoM_Orientation_Task.quat_target = Eigen::Quaternion<double>::Identity();
    CoM_Orientation_Task.Kp = 500;
    CoM_Orientation_Task.Kd = 0;

    // Task for joints to avoid limits
    Task JointPositionTask;
    JointPositionTask.type = JOINT_POS;
    JointPositionTask.task_weight = 0.1;
    JointPositionTask.joint_target = VectorNd::Zero(12);
    JointPositionTask.active_targets = {true, false, false, true, false, false, true, false, false, true, false, false};
    JointPositionTask.Kp = 1000;
    JointPositionTask.Kd = 0;

    // Foot contact tasks
    Task FrontLeftContactTask;
    FrontLeftContactTask.type = BODY_POS;
    FrontLeftContactTask.task_weight = 0.1;
    FrontLeftContactTask.active_targets = {false, false, true};
    FrontLeftContactTask.pos_target << 0, 0, 0; 
    FrontLeftContactTask.Kp = 100;
    FrontLeftContactTask.Kd = 0;

    // Add the tasks with priority 0 and 1
    Pup.addTask("COM_elevation", &CoM_Position_Task);
    Pup.addTask("COM_orientation", &CoM_Orientation_Task);
    Pup.addTask("JointPos", &JointPositionTask);

    VectorNd joint_positions  =  VectorNd::Zero(12);
    joint_positions << 0, 0.7, 0.7,  0, 0.7, 0.7,  0, 0.7, 0.7,  0, 0.7, 0.7; // Init in squatted position
    VectorNd joint_velocities          = VectorNd::Zero(12);
    Eigen::Quaterniond robot_quat      = Eigen::Quaterniond::Identity();
    Eigen::Vector3d body_position      = Eigen::Vector3d::Zero();

    std::array<bool,4> feet_in_contact = {true, true, true, true}; // BL, BR, FL, FR
    body_position << 0, 0, 0.12;
    Pup.updateController(joint_positions, joint_velocities, body_position, robot_quat, feet_in_contact);

    // Testing positions
    Pup.updateController(joint_angles, joint_velocities, robot_position, robot_quaternion, feet_in_contact);

    // Ensure that the hip really is joint 12
    // printMatrix(Pup.getBodyJacobian_("front_left_upper_link"), "front_left_upper_link Jacobian");
    // printMatrix(Pup.getBodyJacobian_("front_left_lower_link"), "front_left_lower_link Jacobian");

    std::string body_name = "front_left_hip";

    // Test at different hip angles
    auto pos = Pup.getRelativeBodyLocation(body_name, VectorNd::Zero(3));
    cout << "Pos: \n" << pos  << endl;
    joint_angles(12) = M_PI_4;
    Pup.updateController(joint_angles, joint_velocities, robot_position, robot_quaternion, feet_in_contact);
    pos = Pup.getRelativeBodyLocation(body_name, VectorNd::Zero(3));
    cout << "Pos:\n" << pos << endl;
    joint_angles(12) = -M_PI_4;
    Pup.updateController(joint_angles, joint_velocities, robot_position, robot_quaternion, feet_in_contact);
    pos = Pup.getRelativeBodyLocation(body_name, VectorNd::Zero(3));
    cout << "Pos:  \n" << pos   << endl;
    joint_angles(12) = M_PI;
    Pup.updateController(joint_angles, joint_velocities, robot_position, robot_quaternion, feet_in_contact);
    pos = Pup.getRelativeBodyLocation(body_name, VectorNd::Zero(3));
    cout << "Pos: \n" << pos  << endl;
    
    Pup.calculateOutputTorque();

    // // //Test height calculation
    // // //////////////////////////////////////////////////////////////////////////////////////
    // IF BASE IS ALWAYS ALIGNED WITH WORLD, WE SHOULDN'T HAVE TO USE ROTATION MATRIX. Just use z values for foot location.
    // auto front_left_pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId("front_left_lower_link"), Eigen::Vector3d::Zero(), true);
    // auto front_right_pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId("front_right_lower_link"), Eigen::Vector3d::Zero(), true);
    // auto back_left_pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId("back_left_lower_link"), Eigen::Vector3d::Zero(), true);
    // auto back_right_pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId("back_right_lower_link"), Eigen::Vector3d::Zero(), true);
    // cout << "front_left_pos: \n" << front_left_pos << endl;
    // cout << "front_right_pos: \n" << front_right_pos << endl;
    // cout << "back_left_pos: \n" << back_left_pos << endl;
    // cout << "back_right_pos: \n" << back_right_pos << endl;

    //Test height calculation
    //////////////////////////////////////////////////////////////////////////////////////
    // //Test height calculation
    // //////////////////////////////////////////////////////////////////////////////////////
 
    // // Pupper rotated with front right leg and back right leg in contact
    // // From Gazebo: height = .0965
    // // Calculated:  h0: 0.100148
    // //              h1: 0.0945443
    // // Values read from gazebo
    // joint_positions  << -.113567,.75649,1.60267,.05627936,-.81907876,-1.805184605,-.116226,.780257,1.58085,.0454920574,-.87633305,-1.77936025;
    // robot_quat.x() = 0.000888; 
    // robot_quat.y() = -0.1009519;
    // robot_quat.z() = -0.0096908;
    // robot_quat.w() = 0.9948437;

    // cout << "PUPPER JOINTS: " << Pup.joint_angles_.transpose() << endl;

    // const RigidBodyDynamics::Math::Vector3d body_contact_point_left_(0.0, -.11, 0.0095); 
    // const RigidBodyDynamics::Math::Vector3d body_contact_point_right_(0.0, -.11, -0.0095); 

    // Pup.updateController(joint_positions, joint_velocities, body_position, robot_quat, feet_in_contact);
    // RigidBodyDynamics::Math::Vector3d r0 = RigidBodyDynamics::CalcBodyToBaseCoordinates(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId("front_right_lower_link"), body_contact_point_right_, true);
    // RigidBodyDynamics::Math::Vector3d r1 = RigidBodyDynamics::CalcBodyToBaseCoordinates(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId("back_right_lower_link"), body_contact_point_right_, true);

    // cout << "r0 front right contact point in base coord: \n" << r0.format(f) << endl;
    // cout << "r1 back right contact point in base coord: \n" << r1.format(f) << endl;

    // // Retrieve Orientation of pupper base
    // Eigen::Matrix3d Rsb = robot_quat.toRotationMatrix();
    // cout << "Rsb: \n" << Rsb.format(f) << endl;
    // cout << "Rsb': \n" << Rsb.transpose().format(f) << endl;

    // Eigen::Vector3d r0_s = Rsb * r0;
    // Eigen::Vector3d r1_s = Rsb * r1;
    // cout << "r0 in world frame: \n" << r0_s.format(f) << endl;
    // cout << "r1 in world frame: \n" << r1_s.format(f) << endl;

    // cout << "h0: " << -r0_s(2) << endl;
    // cout << "h1: " << -r1_s(2) << endl;

    // ///////////// TEST where are the lower links relative to the base???  ////////////////
    // cout << "PUPPER JOINTS: " << Pup.joint_angles_.transpose() << endl;

    // const RigidBodyDynamics::Math::Vector3d body_contact_point_left_(0.0, -.11, 0.0095); 
    // const RigidBodyDynamics::Math::Vector3d body_contact_point_right_(0.0, -.11, -0.0095); 
    // const RigidBodyDynamics::Math::Vector3d zero3d = RigidBodyDynamics::Math::Vector3d::Zero(3);
    // RigidBodyDynamics::Math::Vector3d r0 = RigidBodyDynamics::CalcBodyToBaseCoordinates(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId("front_right_lower_link"), zero3d, true);
    // RigidBodyDynamics::Math::Vector3d r1 = RigidBodyDynamics::CalcBodyToBaseCoordinates(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId("back_right_lower_link"), zero3d, true);
    // RigidBodyDynamics::Math::Vector3d r2 = RigidBodyDynamics::CalcBodyToBaseCoordinates(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId("front_left_lower_link"), zero3d, true);
    // RigidBodyDynamics::Math::Vector3d r3 = RigidBodyDynamics::CalcBodyToBaseCoordinates(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId("back_left_lower_link"), zero3d, true);
    // cout << "r0 front right lower link origin in base coord: \n" << r0.format(f) << endl;
    // cout << "r1 back right lower link origin in base coord: \n" << r1.format(f) << endl;
    // cout << "r2 front left lower link origin in base coord: \n" << r2.format(f) << endl;
    // cout << "r3 back left lower link origin in base coord: \n" << r3.format(f) << endl;

    // ///////////// TEST PROVING THAT PUPPER BASE FRAME is fixed to world ////////////////
    // // 1. first calculate body to base coordinates of the foot.
    // // 2. translate pupper
    // // 3. retrieve body to base coordinates again. 
    // //    Results: the coordinates show the large translation. 
    // cout << "PUPPER JOINTS: " << Pup.joint_angles_.transpose() << endl;

    // const RigidBodyDynamics::Math::Vector3d body_contact_point_left_(0.0, -.11, 0.0095); 
    // const RigidBodyDynamics::Math::Vector3d body_contact_point_right_(0.0, -.11, -0.0095); 
    // RigidBodyDynamics::Math::Vector3d r0 = RigidBodyDynamics::CalcBodyToBaseCoordinates(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId("front_right_lower_link"), body_contact_point_right_, true);
    // RigidBodyDynamics::Math::Vector3d r1 = RigidBodyDynamics::CalcBodyToBaseCoordinates(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId("back_right_lower_link"), body_contact_point_right_, true);
    // cout << "r0 Front right contact point in base coord: \n" << r0.format(f) << endl;
    // cout << "r1 Back right contact point in base coord: \n" << r1.format(f) << endl;

    // // Translate
    // // Pup.updateController(joint_positions, joint_velocities, body_position, robot_quat, feet_in_contact);
    // Pup.joint_angles_ << 10,10,10, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 1;
    // RigidBodyDynamics::Math::Vector3d r0_after = RigidBodyDynamics::CalcBodyToBaseCoordinates(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId("front_right_lower_link"), body_contact_point_right_, true);
    // RigidBodyDynamics::Math::Vector3d r1_after = RigidBodyDynamics::CalcBodyToBaseCoordinates(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId("back_right_lower_link"), body_contact_point_right_, true);
    // cout << "PUPPER JOINTS AFTER TRANSLATION: " << Pup.joint_angles_.transpose() << endl;
    // cout << "r0 Front right contact point in base coord: \n" << r0_after.format(f) << endl;
    // cout << "r1 Back right contact point in base coord: \n" << r0_after.format(f) << endl;

    ///////////////////////  Other Tests ///////////////////////////////////
    // Contact Jacobian Test
    //                // X,Y,Z, Q1,Q2,Q3, BL1,BL2,BL3,    BR1,BR2,BR3,      FL1,FL2,FL3,      FR1,FR2,FR3     Q4
    // Pup.joint_angles_ << 0,0,0,   .7071068,.7071068,0,   0,0,0,           0,0,0,           0, 0,0,            0,0,0,      0; 
    // RigidBodyDynamics::Math::MatrixNd Jc = RigidBodyDynamics::Math::MatrixNd::Zero(3, Pup.Pupper_.qdot_size);
    // const RigidBodyDynamics::Math::Vector3d body_contact_point_left_(0.02, 0.02, 0.02);
    // RigidBodyDynamics::CalcPointJacobian(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId(body_name), body_contact_point_left_, Jc);
    // cout << "JOINT ANGLES: \n" << Pup.joint_angles_.transpose().format(f) << endl;

    // Body Jacobian Test
    // const char* body_name = "front_right_hub";
    // RigidBodyDynamics::Math::MatrixNd Jb = RigidBodyDynamics::Math::MatrixNd::Zero(6, Pup.Pupper_.qdot_size);
    // RigidBodyDynamics::CalcBodySpatialJacobian(Pup.Pupper_, Pup.joint_angles_, Pup.Pupper_.GetBodyId(body_name), Jb, true);
    // printMatrix(Jb.rightCols(12).transpose(), "Jacobian:");

    // Find the necessary torque to achieve this task
    // array<float, 12> Tau = Pup.calculateOutputTorque();

    // Print body name and joint index
    // for(int i = 0; i < Pup.Pupper_.mBodies.size(); i++){
    //     cout << "Body " << Pup.Pupper_.GetBodyName(i) << " Joint q index: "<< Pup.Pupper_.mJoints[i].q_index << endl;
    //     cout << "                           DOF: " << Pup.Pupper_.mJoints[i].mDoFCount << endl;
    // }

    // // TEST CONTACT JACOBIAN.
    // Pup.joint_angles_ = VectorNd::Zero(19);
    // Pup.joint_angles_(18) = 1; // Rotate pupper around x 180 degrees (z -> -z and y -> -y)
    // cout << "Joint angles for test: " << Pup.joint_angles_.transpose() << endl;
    // cout << "CONTACT JACOBIAN IN NEUTRAL CONFIG: " << endl;
    // cout << Pup.Jc_.topRows(3).format(f) << endl;
    // Pup.joint_angles_(18) = 0; // Rotate pupper around x 180 degrees (z -> -z and y -> -y)
    // Pup.joint_angles_(3) = 1; // Rotate pupper around x 180 degrees (z -> -z and y -> -y)
    // Pup.updateContactJacobian_();
    // cout << "Joint angles after flip: " << Pup.joint_angles_.transpose() << endl;
    // cout << "CONTACT JACOBIAN AFTER FLIP: " << endl;
    // cout << Pup.Jc_.topRows(3).format(f) << endl;

    /////////////////////////////// Print body name and joint index //////////////////////////////////
    // double total_mass = 0;
    // for(int i = 0; i < Pup.Pupper_.mBodies.size(); i++){
    //     cout << "Body " << Pup.Pupper_.GetBodyName(i) << " Mass: "<< Pup.Pupper_.mBodies[i].mMass << endl;
    //     total_mass += Pup.Pupper_.mBodies[i].mMass;
    // }
    // cout << "Total mass: " << total_mass << endl;

    // for(int i = 0; i < Pup.Pupper_.mBodies.size(); i++){
    //     cout << "Body " << Pup.Pupper_.GetBodyName(i) << " Intertia: \n"<< Pup.Pupper_.mBodies[i].mInertia << endl;
    // }

    return 0;
}