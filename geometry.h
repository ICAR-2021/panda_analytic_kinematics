#pragma once

#include "types.h"

#include <eigen3/Eigen/Geometry>
#include <iostream>

namespace Geometry
{
  static Vec3d project(Vec3d direction, CVec3dRef vector)
  {
    direction.normalize();
    return vector.dot(direction) * direction;
  }

  static void intSphereSphere(CVec3dRef s1Center, const double& s1Radius,
                              CVec3dRef s2Center, const double& s2Radius,
                              Vec3dRef cCenter, Vec3dRef cNormal, double& cRadius)
  {
    cNormal = s2Center - s1Center;
    double distance(cNormal.norm());
    cNormal.normalize();

    // find angles between cNormal and (s1Center/s2Center to intersection)
    double alpha, beta;
    alpha = acos((pow(s1Radius, 2) + pow(distance, 2) - pow(s2Radius, 2))
                 / (2 * s1Radius * distance));
    beta = acos((pow(s2Radius, 2) + pow(distance, 2) - pow(s1Radius, 2))
                / (2 * s2Radius * distance));

    cRadius = s1Radius * sin(alpha);
    cCenter = s1Center + cos(alpha) * s1Radius * cNormal;
  }

  static void intPlanePlane(Vec3d p0Origin, Vec3d p0Normal,
                            Vec3d p1Origin, Vec3d p1Normal,
                            Vec3dRef lOrigin, Vec3dRef lDirection)
  {
    p0Normal.normalize();
    p1Normal.normalize();

    lDirection = p0Normal.cross(p1Normal);
    Vec2d distances(p0Origin.dot(p0Normal),
                    p1Origin.dot(p1Normal));

    MatXd normals(2, 3);
    normals << p0Normal[0], p0Normal[1], p0Normal[2],
               p1Normal[0], p1Normal[1], p1Normal[2];

    MatXd transp(3, 2);
    transp << p0Normal[0], p1Normal[0],
              p0Normal[1], p1Normal[1],
              p0Normal[2], p1Normal[2];

    lOrigin = (transp * (normals * transp).inverse()) * distances;
  }

  /**
   * @returns x as 2d vector of two scalar solutions for lOrigin + x * lDirection
   */
  static Vec2d intLineSphere(Vec3d lOrigin, Vec3d lDirection,
                             Vec3d sCenter, double sRadius)
  {
    // a t^2 + b t + c = 0
    double a(lDirection.squaredNorm());
    double b(2 * (lDirection.dot(lOrigin) - lDirection.dot(sCenter)));
    double c(lOrigin.squaredNorm() + sCenter.squaredNorm()
             - 2 * lOrigin.dot(sCenter) - pow(sRadius, 2));

    // t [+/-] = -b/(2a) +/- sqrt((b/(2a))^2 - c/a)
    double d(-b / (2 * a));
    double e(pow(d, 2) - c / a);

    Vec2d solVec(d, d);
    if (e >= 0)
    {
      solVec[0] += sqrt(e);
      solVec[1] -= sqrt(e);
    }
    else std::cerr << "Line and sphere do not intersect! (e = " << e << ") -> "
                   << "Returning nearest point on line!" << std::endl;

    return solVec;
  }

  static void intCircleSphere(CVec3dRef cCenter, CVec3dRef cNormal,
                               const double& cRadius,
                               CVec3dRef sCenter, const double& sRadius,
                               Vec3dRef solA, Vec3dRef solB)
  {
    Vec3d sc(cCenter - sCenter);
    if (sc.norm() >= sRadius + cRadius)
    {
      solA = sCenter + sc.normalized() * sRadius;
      solB = solA;
    }
    else
    {
      Vec3d icCenter, icNormal;
      double icRadius;
      intSphereSphere(sCenter, sRadius, cCenter, cRadius, icCenter, icNormal, icRadius);

      Vec3d lOrigin, lDirection;
      intPlanePlane(icCenter, icNormal, cCenter, cNormal, lOrigin, lDirection);

      Vec2d solFactors(intLineSphere(lOrigin, lDirection, sCenter, sRadius));
      solA = lOrigin + solFactors[0] * lDirection;
      solB = lOrigin + solFactors[1] * lDirection;
    }
  }

  static void apply(CVecXdRef transform, VecXdRef vec)
  {
    Eigen::AngleAxisd rot(transform.tail(3).norm(), transform.tail(3).normalized());

    vec.head(3) = rot * vec.head(3);

    if (vec.size() == 6)
    {
      Eigen::AngleAxisd orient(vec.tail(3).norm(), vec.tail(3).normalized());
      Eigen::AngleAxisd rot12(rot * orient);
      vec.tail(3) = rot12.axis();
      vec.tail(3) *= rot12.angle();
    }

    if (transform.size() == 6) vec.head(3) += transform.head(3);
  }
}
