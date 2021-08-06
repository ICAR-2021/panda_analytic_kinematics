
#include "panda_kinematics.h"
#include "geometry.h"
#include <iostream>

#define TRIG_PREC     1e5
#define TRIG_RND(val) (round(val * TRIG_PREC) / TRIG_PREC)

void PandaKinematics::setup(Kinematics& kinematics)
{
  kinematics.addDisplacement( .0   ,  .0  , .14 ,      .0, .0,      .0);
  kinematics.addDisplacement( .0   ,  .0  , .193, -M_PI_2, .0,      .0);
  kinematics.addDisplacement( .0   , -.192, .0  ,  M_PI_2, .0,      .0);
  kinematics.addDisplacement( .0825,  .0  , .124,  M_PI_2, .0,      .0);
  kinematics.addDisplacement(-.0825,  .124, .0  , -M_PI_2, .0,      .0);
  kinematics.addDisplacement( .0   ,  .0  , .26 ,  M_PI_2, .0,      .0);
  kinematics.addDisplacement( .088 ,  .0  , .0  ,  M_PI_2, .0,      .0);
  // without additional end effector
  kinematics.addDisplacement( .0   ,  .0  , .107,      .0, .0, -M_PI_4);
  // with franka hand
  // kinematics.addDisplacement( .0   ,  .0  , .22 ,      .0, .0, -M_PI_4);

  kinematics.addJointInfo(-170, 170);
  kinematics.addJointInfo(-105, 105);
  kinematics.addJointInfo(-170, 170);
  kinematics.addJointInfo(-180, 5);
  kinematics.addJointInfo(-170, 170);
  kinematics.addJointInfo(-5, 219);
  kinematics.addJointInfo(-170, 170);

  kinematics.setInvKin([&](CVec6dRef pose, CVecXdRef qinit, const double& wrAngle) -> VecXd {
    VecXd q;
    q.setZero(qinit.size());

    // shoulder position
    Vec3d shoulder(kinematics.qToX(VecXd::Zero(1)).head(3));
    // std::cout << "Shoulder: " << shoulder.transpose() << std::endl;

    // determine end effector's normal, open, and approach vector
    Vec3d eex(Vec3d::UnitX()), eey(Vec3d::UnitY()), eez(Vec3d::UnitZ());
    Geometry::apply(pose.tail(3), eex);
    Geometry::apply(pose.tail(3), eey);
    Geometry::apply(pose.tail(3), eez);

    // define wrist circle (wc): all possible wrist positions
    Vec3d wcCenter(pose.head(3) - eez * kinematics.getDisplacement(-1)[2]);
    double wcRadius(kinematics.getDisplacement(-2)[0]);

    // use projection of (shoulder to wc center) on eez to find x67
    Vec3d x67(wcCenter - (shoulder + eez * eez.dot(wcCenter - shoulder)));
    Geometry::apply(wrAngle * -eez, x67);
    x67.normalize();

    // set length to radius and add wc center
    Vec3d wrist(wcRadius * -x67);
    wrist += wcCenter;
    // std::cout << "Wrist: " << wrist.transpose() << std::endl;

    // find q67 from eex and x67 using a temporary vector (tmpVec)
    Vec3d tmpVec(eex);
    Vec3d l7inv(-kinematics.getDisplacement(-1).tail(3));
    Geometry::apply(pose.tail(3), l7inv);
    Geometry::apply(l7inv, tmpVec);
    q[6] = acos(TRIG_RND(x67.dot(tmpVec)));
    // find correct sign
    Geometry::apply(Vec3d(0, 0, -q[6]), tmpVec);
    if (tmpVec.dot(x67) > .01) q[6] *= -1;

    // relative zero positions from j12 to j34 (upperarm) and from j34 to j56 (forearm)
    Vec3d upperarm(kinematics.getDisplacement(3).head(3));
    Geometry::apply(kinematics.getDisplacement(2), upperarm);
    Vec3d forearm(kinematics.getDisplacement(5).head(3));
    Geometry::apply(kinematics.getDisplacement(4), forearm);
    double lenUa(upperarm.norm());
    double lenFa(forearm.norm());
    double ratio(lenUa / (lenUa + lenFa));

    double elbowOffset(kinematics.getDisplacement(3)[0]);
    double uaOffset(asin(elbowOffset / lenUa));
    double faOffset(asin(elbowOffset / lenFa));
    double lenWrSh((wrist - shoulder).norm());

    double cosShElWr((pow(lenUa, 2) + pow(lenFa, 2) - pow(lenWrSh, 2)) / (2 * lenUa * lenFa));
    double angle(acos(cosShElWr));
    q[3] = -M_PI + angle + faOffset + uaOffset;

    // check plausibility
    double tmpVal((kinematics.qToX(q.head(5)).head(3) - shoulder).norm() - lenWrSh);
    if (abs(tmpVal) > .01) q[3] = -M_PI + angle - faOffset - uaOffset;

    // virtual elbow (ve): intersetion of z23 and z45
    double veExtension(0);
    double q3half((M_PI - abs(q[3])) / 2);
    if (sin(q3half) > .0001) veExtension = elbowOffset * cos(q3half) / sin(q3half);

    // distance from ve to wrist
    double vewRadius(forearm[1] + veExtension);
    // distance from ve to shoulder
    double vesRadius(-upperarm[1] + veExtension);

    Vec3d z56(eez.cross(x67));
    Vec3d ve(Geometry::intCircleSphere(wrist, z56, vewRadius, shoulder, vesRadius, wcCenter));
    // std::cout << "Virtual Elbow: " << ve.transpose() << std::endl;

    // find z34 from virtual forearm (vf) and virtual upper arm (vu)
    Vec3d vf(wrist - ve), z45(vf.normalized());
    Vec3d vu(ve - shoulder), z23(vu.normalized());
    Vec3d z34;
    if (abs(z45.dot(z23)) > .999) z34 = z56;
    else z34 = z45.cross(z23).normalized();

    // find elbow from ve, z45, and z34
    Vec3d elbow(ve + veExtension * z45 + elbowOffset * z45.cross(z34));
    // std::cout << "Elbow: " << elbow.transpose() << std::endl;

    // calc q56 from z45 and eez
    q[5] = z45.dot(x67) < 0 ? M_PI + acos(eez.dot(z45)) : acos(eez.dot(-z45));

    // calc q45 from z34 and z56
    q[4] = acos(TRIG_RND(z34.dot(z56)));
    // find correct sign
    tmpVec = z34;
    Geometry::apply(q[4] * z45, tmpVec);
    if (abs(tmpVec.dot(z56)) < .999) q[4] *= -1;

    Vec3d z12;
    // calc q12 from ve position
    q[1] = acos(TRIG_RND((ve[2] - shoulder[2]) / vesRadius));
    if (q[1] < .001)
    {
      q[0] = acos((elbow - ve).head(2).normalized().dot(Vec2d::UnitX())) / 2;
      q[0] = std::copysign(q[0], Vec2d::UnitY().dot((elbow - ve).head(2)));
      q[2] = q[0];
    }
    else
    {
      double theta(atan2(ve[1], ve[0])), q23, sum, minSum(4 * M_PI);
      for (double q01 : {theta, theta - std::copysign(M_PI, theta)})
      {
        z12 = Vec3d::UnitY();
        Geometry::apply(q01 * Vec3d::UnitZ(), z12);
        q23 = acos(TRIG_RND(z12.dot(-z34)));
        tmpVec = z12;
        Geometry::apply(q23 * z23, tmpVec);
        if (abs(tmpVec.dot(-z34)) < .999) q23 *= -1;
        sum = abs(q01) + abs(q23);
        if (minSum > sum)
        {
          q[0] = q01;
          q[2] = q23;
          minSum = sum;
        }
      }
    }

    z12 = Vec3d::UnitY();
    Geometry::apply(q[0] * Vec3d::UnitZ(), z12);
    tmpVec = Vec3d::UnitZ();
    Geometry::apply(q[1] * z12, tmpVec);
    if (abs(tmpVec.dot(z23)) < .999) q[1] *= -1;

    return q;
  });
}

void panda_inv(const double* pose, const double wrist, double* joints)
{
  Kinematics panda;
  PandaKinematics::setup(panda);
  Vec6d x;
  x.setZero();

  for (int i = 0; i < 6; i++) x[i] = pose[i];

  VecXd q(panda.xToQ(x, pose[6]));

  for (int i = 0; i < 7; i++) joints[i] = q[i];
}

void panda_sum(double* joints, double wrist, double* result)
{
  for (int i = 0; i < 3; i++)
  {
    std::cout << "Joint value " << i << ": " << joints[i] << std::endl;
    result[i] = joints[i] * wrist;
  }
}
