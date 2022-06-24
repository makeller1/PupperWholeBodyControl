#ifndef PUPPER_STATE_EST_HH_
#define PUPPER_STATE_EST_HH_

#include "workstation/PupperWBC.hpp"
#include <array>

Eigen::Vector2d estimateLateralPos(PupperWBC& WBC, Eigen::Quaterniond robot_quat);
float estimateHeight(PupperWBC& WBC, std::array<bool, 4> feet_in_contact);
#endif