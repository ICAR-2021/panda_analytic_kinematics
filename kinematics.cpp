#include <iostream>

#include "kinematics.h"
#include "geometry.h"

Kinematics* Kinematics::addDisplacement(
    double x, double y, double z, double a, double b, double c)
{
  Vec6d disp;
  disp << x, y, z, a, b, c;
  _disps.push_back(disp);

  return this;
}

Vec6d Kinematics::getDisplacement(int i)
{
  if (_disps.empty()) return Vec6d::Zero();
  return _disps[i % _disps.size()];
}

std::vector<Vec6d> Kinematics::getDisplacements()
{
  return _disps;
}

Kinematics* Kinematics::addJointLimits(double min, double max)
{
  double mean((min + max) / 2);
  _joint_limits.push_back(Vec3d(min, max, mean));

  return this;
}

void Kinematics::checkLimits(VecXdRef q)
{
  for (int i = 0; i < std::min((int) _joint_limits.size(), (int) q.size()); i++)
  {
    q[i] = std::min(_joint_limits[i][1], std::max(_joint_limits[i][0], q[i]));
  }
}

void Kinematics::checkMean(VecXdRef q)
{
  for (int i = 0; i < std::min((int) _joint_limits.size(), (int) q.size()); i++)
  {
    while (q[i] > _joint_limits[i][2] + M_PI) q[i] -= 2 * M_PI;
    while (q[i] < _joint_limits[i][2] - M_PI) q[i] += 2 * M_PI;
  }
}

Vec6d Kinematics::qToX(CVecXdRef q)
{
  if (q.size() >= _disps.size())
    throw Exception("Too many joint values for number of displacements.");

  Vec6d qvec;
  Vec6d x(_disps[q.size()]);
  // Vec6d x(Vec6d::Zero());
  for (int i = q.size() - 1; i >= 0; i--)
  {
    qvec << 0, 0, 0, 0, 0, q[i];
    Geometry::apply(qvec, x);
    Geometry::apply(_disps[i], x);
  }

  return x;
}

Mat6Xd Kinematics::calcJacobian(CVecXdRef q)
{
  if (q.size() >= _disps.size())
    throw Exception("Too many joint values for number of displacements.");

  Mat6Xd jacobian(6, q.size());

  Vec3d axis(0, 0, 1);
  Vec3d rot(0, 0, 0);
  Vec3d pos;

  Geometry::apply(_disps[0].tail(3), axis);
  // TODO: check indices
  for (int i = 0; i < q.size(); i++)
  {
    if (i >= 0)
    {
      rot[2] = q[i];
      Geometry::apply(rot, axis);
    }
    pos << qToX(q.head(i)).head(3);
    jacobian.col(i) << axis, pos.cross(axis);
  }

  return jacobian;
}

Kinematics::Exception::Exception(std::string message)
{
  _message = message;
}

Kinematics::Exception::~Exception()
{
}

std::string Kinematics::Exception::what()
{
  return "Kinematics Exception: " + _message;
}
