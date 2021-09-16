
#pragma once

#include "kinematics.h"

class PandaKinematics : public Kinematics
{
  public:
    PandaKinematics(Vec6d endEffector = Vec6d::Zero());
    MatXd xToQ(CVec6dRef pose, const double& wrAngle = .0,
               CVecXdRef qinit = VecXd::Zero(7));
};

extern "C"
{
  void panda_inv(double* pose, double wrist, double* qOut);
}
