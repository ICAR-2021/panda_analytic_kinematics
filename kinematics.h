#pragma once

#include "types.h"

class Kinematics
{
  private:
    std::vector<Vec6d>  _disps;
    std::vector<Vec2d>  _joint_limits;
    Mat6Xd              _jacobian;

    std::function<VecXd(CVec6dRef, CVecXdRef, const double&)> _inv_kin;

  public:
    Kinematics* addDisplacement(double x,
                                double y,
                                double z,
                                double a,
                                double b,
                                double c);

    Vec6d getDisplacement(int i);

    Kinematics* addJointInfo(double min, double max);

    void setInvKin(std::function<VecXd(CVec6dRef, CVecXdRef, const double&)> invKin);

    VecXd xToQ(CVec6dRef pose, const double& wrist = .0);
    VecXd xToQ(CVec6dRef pose, CVecXdRef qinit, const double& wrist = .0);
    Vec6d qToX(CVecXdRef q);

    Mat6XdRef calcJacobian(CVecXdRef q);

    class Exception
    {
      private:
        std::string _message;

      public:
        Exception(std::string message);
        ~Exception();
        std::string what();
    };
};
