#include "workstation/StateEstimation.hpp"
#include <numeric>
#include <math.h>

using namespace RigidBodyDynamics::Math;

template<typename T> 
const T& max(const T& a, const T& b)
{
    return (a < b) ? b : a;
}

float estimateHeight(PupperWBC& WBC, std::array<bool, 4> feet_in_contact)
{
    // Estimates the height of the pupper from the bottom PCB origin to the ground using forward kinematics.
    // Note: feet_in_contact_, joint_angles_ and orientation should be updated before this. 
    // orientation is implicitly used in the getRelativeBodyLocation.

    int num_contacts = std::accumulate(feet_in_contact.begin(), feet_in_contact.end(), 0);
    float max_z = 0.0;
    static float height; // height from floor to COM base

    Vector3d r_bl = Vector3d::Zero(3);
    Vector3d r_br = Vector3d::Zero(3);
    Vector3d r_fl = Vector3d::Zero(3);
    Vector3d r_fr = Vector3d::Zero(3);

    if (num_contacts == 0){
        return height;
    }

    if (feet_in_contact[0]){
        r_bl = WBC.getRelativeBodyLocation("back_left_lower_link", WBC.body_contact_point_left);
        max_z = max(max_z,(float)-r_bl(2));
    }
    if (feet_in_contact[1]){
        r_br = WBC.getRelativeBodyLocation("back_right_lower_link", WBC.body_contact_point_right);
        max_z = max(max_z,(float)-r_br(2));
    }
    if (feet_in_contact[2]){
        r_fl = WBC.getRelativeBodyLocation("front_left_lower_link", WBC.body_contact_point_left);
        max_z = max(max_z,(float)-r_fl(2));
    }
    if (feet_in_contact[3]){
        r_fr = WBC.getRelativeBodyLocation("front_right_lower_link", WBC.body_contact_point_right);
        max_z = max(max_z,(float)-r_fr(2));
    }

    height = max((float)0.0, max_z);
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
