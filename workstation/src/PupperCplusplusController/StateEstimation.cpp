#include "workstation/StateEstimation.hpp"
#include <numeric>
#include <math.h>

using namespace RigidBodyDynamics::Math;

template<typename T> 
const T& max(const T& a, const T& b)
{
    return (a < b) ? b : a;
}

Eigen::Vector2d estimateLateralPos(PupperWBC& WBC, Eigen::Quaterniond robot_quat){
    // robot_quat is a quaternion describing the orientation of the robot
    // TODO: handle switching contacts, slipping feet
    // Update the measured COM_Position as the average of the support polygon edges (contacts)
    double offset_x = -0.07; // x distance to approximated COM in bottom_pcb frame (m) 
    double offset_y = 0.0; // y distance to approximated COM in bottom_pcb frame (m) 

    Vector3_t support_avg_world = Vector3_t::Zero(3); // Instantaneous support average
    
    // Compute average of support polygon edges in world coordinates
    support_avg_world += WBC.calcBodyPosInBaseCoordinates("back_left_lower_link", WBC.body_contact_point_left);
    support_avg_world += WBC.calcBodyPosInBaseCoordinates("back_right_lower_link", WBC.body_contact_point_right);
    support_avg_world += WBC.calcBodyPosInBaseCoordinates("front_left_lower_link", WBC.body_contact_point_left);
    support_avg_world += WBC.calcBodyPosInBaseCoordinates("front_right_lower_link", WBC.body_contact_point_right);

    // Rotate to align with pupper in z direction
    Eigen::AngleAxisd angle_axis;
    angle_axis = robot_quat;
    double total_angle = angle_axis.angle();
    double z_angle = total_angle*angle_axis.axis()(2);

    Eigen::Matrix3d R_body_to_world; // rotation matrix
    R_body_to_world = Eigen::AngleAxisd(-z_angle, Eigen::Vector3d::UnitZ());

    // Support average in body coordinates (negated for com position relative to support average)
    Vector3_t support_avg_body = -R_body_to_world*support_avg_world;

    support_avg_body(0) = support_avg_body(0) + offset_x;
    support_avg_body(1) = support_avg_body(1) + offset_y;

    return {support_avg_body(0),support_avg_body(1)};
}

float estimateHeight(PupperWBC& WBC, std::array<bool, 4> feet_in_contact){
    // Estimates the height of the pupper from the bottom PCB origin to the ground using forward kinematics.
    // Note: feet_in_contact_, joint_angles_ and orientation should be updated before this. 
    // orientation is implicitly used in the calcBodyPosInBaseCoordinates.

    int num_contacts = std::accumulate(feet_in_contact.begin(), feet_in_contact.end(), 0);
    float max_z = 0.0;
    static float height; // height from floor to COM base
    double alpha = 0.367; // Low pass filter coefficient (500hz cutoff with Ts = 2 ms)

    Vector3d r_bl = Vector3d::Zero(3);
    Vector3d r_br = Vector3d::Zero(3);
    Vector3d r_fl = Vector3d::Zero(3);
    Vector3d r_fr = Vector3d::Zero(3);

    if (num_contacts == 0){
        return height;
    }

    if (feet_in_contact[0]){
        r_bl = WBC.calcBodyPosInBaseCoordinates("back_left_lower_link", WBC.body_contact_point_left);
        max_z = max(max_z,(float)-r_bl(2));
    }
    if (feet_in_contact[1]){
        r_br = WBC.calcBodyPosInBaseCoordinates("back_right_lower_link", WBC.body_contact_point_right);
        max_z = max(max_z,(float)-r_br(2));
    }
    if (feet_in_contact[2]){
        r_fl = WBC.calcBodyPosInBaseCoordinates("front_left_lower_link", WBC.body_contact_point_left);
        max_z = max(max_z,(float)-r_fl(2));
    }
    if (feet_in_contact[3]){
        r_fr = WBC.calcBodyPosInBaseCoordinates("front_right_lower_link", WBC.body_contact_point_right);
        max_z = max(max_z,(float)-r_fr(2));
    }

    height = (alpha)*height + (1-alpha)*max((float)0.0, max_z);
    // std::cout << "Instantant Height: " << max((float)0.0, max_z) << std::endl;
    return height;

    // Since the base frame is aligned with the prismatic floating joints (which are aligned with world frame), the z axis should be the height. 
    // However, I'm keeping the code below just in case.
    // Below, we rotate the vectors to the world frame and extract the z component. 
    // Retrieve Orientation of pupper base
    // Eigen::Matrix3d Rsb = robot_orientation_.toMatrix(); // Rotation matrix from world to PCB orientation 

    // Eigen::Vector3d s_bl = Rsb * r_bl;
    // Eigen::Vector3d s_br = Rsb * r_br;
    // Eigen::Vector3d s_fl = Rsb * r_fl;
    // Eigen::Vector3d s_fr = Rsb * r_fr;

    // double s_height = -(s_bl(2) + s_br(2) + s_fl(2) + s_fr(2) ) / num_contacts;
    // cout << "------------height calculation ---------------" << endl;
    // cout << "Back left contact point in base coord: \n" << r_bl.transpose().format(f) << endl;
    // cout << "Back right contact point in base coord: \n" << r_br.transpose().format(f) << endl;
    // cout << "Front left contact point in base coord: \n" << r_fl.transpose().format(f) << endl;
    // cout << "Front right contact point in base coord: \n" << r_fr.transpose().format(f) << endl;
    // cout << "robot joint angles: " << joint_angles_.format(f) << endl;
    // cout << "robot_orientation: " << endl << robot_orientation_ << endl;
    // cout << "Rsb: \n" << Rsb.format(f) << endl;
    // cout << "bl rotated: \n" << s_bl.transpose().format(f) << endl;
    // cout << "br rotated: \n" << s_br.transpose().format(f) << endl;
    // cout << "fl rotated: \n" << s_fl.transpose().format(f) << endl;
    // cout << "fr rotated: \n" << s_fr.transpose().format(f) << endl;

    // cout << "height assuming base frame is not aligned with world (this should be wrong): " << s_height << endl;
    // cout << "height (this should be right): " << height << endl;
    // cout << "---------------------------- ----------------" << endl;

}
