#ifndef PUPPER_STATE_EST_HH_
#define PUPPER_STATE_EST_HH_

#include "workstation/PupperWBC.hpp"
#include <array>

float estimateHeight(PupperWBC& WBC_, std::array<bool, 4> feet_in_contact);

#endif