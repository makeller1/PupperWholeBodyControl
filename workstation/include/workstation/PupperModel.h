#ifndef PUPPER_MODEL_HH_
#define PUPPER_MODEL_HH_

#include <memory>
#include "rbdl/rbdl.h"

std::shared_ptr<RigidBodyDynamics::Model> createPupperModel();

#endif