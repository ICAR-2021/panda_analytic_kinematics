#pragma once

#include <vector>
#include <memory>

#include "types.h"

class Kinematics
{
  private:
    std::vector<Vec6d>  _disps;
    Mat6Xd              _jacobian;

  protected:
    std::vector<Vec3d>  _joint_limits;

  public:
    Kinematics* addDisplacement(double x, double y, double z,
                                double a, double b, double c);

    Vec6d getDisplacement(int i);

    std::vector<Vec6d> getDisplacements();

    Kinematics* addJointLimits(double min, double max);

    void checkLimits(VecXdRef q);

    virtual VecXd xToQ(CVec6dRef pose, const double& wrist, CVecXdRef qinit) = 0;

    Vec6d qToX(CVecXdRef q);

    Mat6Xd calcJacobian(CVecXdRef q);

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

typedef std::shared_ptr<Kinematics> KinematicsPtr;
