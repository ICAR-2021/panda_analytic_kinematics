
#include "panda_kinematics.h"
#include "geometry.h"
#include <iostream>

#define TRIG_PREC     1e-7
#define TRIG_RND(val) (round(val / TRIG_PREC) * TRIG_PREC)

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
  // kinematics.addDisplacement( .0   ,  .0  , .107,      .0, .0, -M_PI_4);
  // with franka hand
  kinematics.addDisplacement( .0   ,  .0  , .22 ,      .0, .0, -M_PI_4);

  kinematics.addJointInfo(-170, 170);
  kinematics.addJointInfo(-105, 105);
  kinematics.addJointInfo(-170, 170);
  kinematics.addJointInfo(-180, 5);
  kinematics.addJointInfo(-170, 170);
  kinematics.addJointInfo(-5, 219);
  kinematics.addJointInfo(-170, 170);

  kinematics.setInvKin([&](CVec6dRef pose, CVecXdRef qinit, const double& wrAngle) -> VecXd {
    //////////////////////////// should be done only once //////////////////////////
    // shoulder position
    const Vec3d shoulder(kinematics.qToX(VecXd::Zero(2)).head(3));

    // define wrist circle radius: distance between wrist and z67
    const double wcRadius(kinematics.getDisplacement(-2)[0]);

    // relative zero positions from j12 to j34 (upperarm) and from j34 to j56 (forearm)
    Vec3d upperarm(kinematics.getDisplacement(3).head(3));
    Geometry::apply(kinematics.getDisplacement(2), upperarm);
    Vec3d forearm(kinematics.getDisplacement(5).head(3));
    Geometry::apply(kinematics.getDisplacement(4), forearm);
    const double lenUa(upperarm.norm());
    const double lenFa(forearm.norm());
    const double ratio(lenUa / (lenUa + lenFa));

    // offsets between elbow and joint axes z23 and z45
    const double elbowOffset(kinematics.getDisplacement(3)[0]);
    // angle offset between upper arm and virtual upper arm
    const double uaOffset(asin(elbowOffset / lenUa));
    // angle offset between forearm and virtual forearm
    const double faOffset(asin(elbowOffset / lenFa));
    ////////////////////////////////////////////////////////////////////////////////

    VecXd q;
    q.setZero(qinit.size());
    Vec3d ee(pose.head(3));

    // determine end effector's normal, open, and approach vector
    Vec3d eex(Vec3d::UnitX()), eey(Vec3d::UnitY()), eez(Vec3d::UnitZ());
    Geometry::apply(pose.tail(3), eex);
    Geometry::apply(pose.tail(3), eey);
    Geometry::apply(pose.tail(3), eez);

    // define wrist circle (wc): all possible wrist positions
    Vec3d wcCenter(ee - eez * kinematics.getDisplacement(-1)[2]);

    // use projection of (shoulder to wc center) on eez to find x67
    Vec3d x67(wcCenter - (shoulder + eez * eez.dot(wcCenter - shoulder)));
    Geometry::apply(wrAngle * -eez, x67);
    x67.normalize();

    // find wrist by moving by wc radius along negative x67 from wc center
    Vec3d wrist(wcRadius * -x67);
    wrist += wcCenter;

    // find q67 from eex and x67 using a temporary vector (tmpVec)
    Vec3d tmpVec(eex);
    Vec3d l7inv(-kinematics.getDisplacement(-1).tail(3));
    Geometry::apply(pose.tail(3), l7inv);
    Geometry::apply(l7inv, tmpVec);
    q[6] = acos(TRIG_RND(x67.dot(tmpVec)));
    // find correct sign
    Geometry::apply(-q[6] * eez, tmpVec);
    if (tmpVec.dot(x67) < 1 - TRIG_PREC) q[6] *= -1;

    // check workspace exceedance
    Vec3d wrSh = shoulder - wrist;
    double gap = wrSh.norm() - (lenUa + lenFa);
    if (gap > 0)
    {
      // move everything towards shoulder
      wrSh.normalize();
      wrSh *= gap + TRIG_PREC;
      std::cout << "Gap before: " << gap << std::endl;

      wcCenter += wrSh;
      wrist += wrSh;
      ee += wrSh;
      wrSh = shoulder - wrist;
      std::cout << "Gap after: " << (wrSh.norm() - (lenUa + lenFa)) << std::endl;
    }

    // find angle spanned by forearm and upper arm
    double cosShElWr((pow(lenUa, 2) + pow(lenFa, 2) - pow(wrSh.norm(), 2))
                      / (2 * lenUa * lenFa));
    double shElWr(acos(cosShElWr));
    q[3] = -M_PI + shElWr + faOffset + uaOffset;

    // check plausibility
    double tmpVal((kinematics.qToX(q.head(5)).head(3) - shoulder).norm() - wrSh.norm());
    if (abs(tmpVal) > TRIG_PREC) q[3] = -M_PI + shElWr - faOffset - uaOffset;

    // virtual elbow (ve): intersetion of z23 and z45
    double veExtension(0);
    double q3half((M_PI - abs(q[3])) / 2);
    if (sin(q3half) > TRIG_PREC) veExtension = elbowOffset * cos(q3half) / sin(q3half);

    // distance from ve to wrist
    double vewDist(forearm[1] + veExtension);

    // distance from ve to shoulder
    double vesDist(-upperarm[1] + veExtension);

    Vec3d ve, veAlt, z56(eez.cross(x67));
    Geometry::intCircleSphere(wrist, z56, vewDist, shoulder, vesDist, ve, veAlt);
    if ((ve - ee).norm() < (veAlt - ee).norm()) ve = veAlt;

    // find z34 from virtual forearm (vf) and virtual upper arm (vu)
    Vec3d vf(wrist - ve), z45(vf.normalized());
    Vec3d vu(ve - shoulder), z23(vu.normalized());
    Vec3d z34;
    if (z45.dot(z23) > 1 - TRIG_PREC) z34 = z56;
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
    if (tmpVec.dot(z56) < 1 - TRIG_PREC) q[4] *= -1;

    // calc q12 from ve position
    q[1] = acos(TRIG_RND((ve[2] - shoulder[2]) / vesDist));
    if (q[1] < TRIG_PREC)
    {
      q[0] = acos((elbow - ve).head(2).normalized().dot(Vec2d::UnitX())) / 2;
      q[0] = std::copysign(q[0], Vec2d::UnitY().dot((elbow - ve).head(2)));
      q[2] = q[0];
    }
    else
    {
      double q23, sum, minSum(4 * M_PI);
      double theta(atan2(ve[1], ve[0]));
      Vec3d z12;
      for (double q01 : {theta, theta - std::copysign(M_PI, theta)})
      {
        // calc z12 from q01
        z12 = Vec3d::UnitY();
        Geometry::apply(q01 * Vec3d::UnitZ(), z12);

        // calc q23 from z12
        q23 = acos(TRIG_RND(z12.dot(-z34)));
        tmpVec = z12;
        Geometry::apply(q23 * z23, tmpVec);
        // find correct sign
        if (tmpVec.dot(-z34) < 1 - TRIG_PREC) q23 *= -1;

        sum = abs(q01 - qinit[0]) + abs(q[1] - qinit[1]) + abs(q23 - qinit[2]);
        if (minSum > sum)
        {
          q[0] = q01;
          q[2] = q23;

          // find correct sign of q12 by applying to Z and compare with z23
          tmpVec = Vec3d::UnitZ();
          Geometry::apply(q[1] * z12, tmpVec);
          if (abs(tmpVec.dot(z23)) < 1 - TRIG_PREC) q[1] *= -1;

          minSum = sum;
        }
      }
    }

    return q;
  });
}

void panda_inv(const double* pose, const double wrist, double* joints)
{
  Kinematics panda;
  PandaKinematics::setup(panda);
  CVec6dMap x(pose);
  VecXdMap qOut(joints, 7);
  VecXd qIn(qOut);

  qOut = panda.xToQ(x, qIn, wrist);
}

void panda_sum(double* joints, double wrist, double* result)
{
  for (int i = 0; i < 3; i++)
  {
    std::cout << "Joint value " << i << ": " << joints[i] << std::endl;
    result[i] = joints[i] * wrist;
  }
}
