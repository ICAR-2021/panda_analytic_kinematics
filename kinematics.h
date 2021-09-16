#pragma once

#include <vector>

#include "types.h"

class Kinematics
{
  private:
    std::vector<Vec6d>  _disps;
    std::vector<Vec2d>  _joint_limits;
    Mat6Xd              _jacobian;

  public:
    Kinematics* addDisplacement(double x, double y, double z,
                                double a, double b, double c);

    Vec6d getDisplacement(int i);

    std::vector<Vec6d> getDisplacements();

    Kinematics* addJointLimits(double min, double max);

    void checkLimits(VecXdRef q);

    virtual MatXd xToQ(CVec6dRef pose, const double& wrist, CVecXdRef qinit) = 0;
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
