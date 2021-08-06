
#pragma once

#include "kinematics.h"

namespace PandaKinematics
{
  void setup(Kinematics& kinematics);
}

extern "C"
{
  void panda_inv(const double* pose, const double wrist, double* joints);

  void panda_sum(double* i, double j, double* result);
}
