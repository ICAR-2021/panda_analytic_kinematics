
#include "panda_kinematics.h"
#include "geometry.h"
#include <iostream>

#define TRIG_PREC     1e-7
#define TRIG_RND(val) (round(val / TRIG_PREC) * TRIG_PREC)

// #define SVE_RADIUS  0

// static void setDebugValue(const int* keys, double* vals,
//                           int& keyIndex, int& valIndex,
//                           const int& key, double* data, int length)
// {
//   if (keys && keys[keyIndex] == key)
//   {
//     memcpy(&vals[valIndex], data, length);
//     keyIndex++;
//     valIndex += length;
//   }
// }

PandaKinematics::PandaKinematics(Vec6d endEffector)
{
  addDisplacement( .0   ,  .0  , .14 ,      .0, .0,      .0);
  addDisplacement( .0   ,  .0  , .193, -M_PI_2, .0,      .0);
  addDisplacement( .0   , -.192, .0  ,  M_PI_2, .0,      .0);
  addDisplacement( .0825,  .0  , .124,  M_PI_2, .0,      .0);
  addDisplacement(-.0825,  .124, .0  , -M_PI_2, .0,      .0);
  addDisplacement( .0   ,  .0  , .26 ,  M_PI_2, .0,      .0);
  addDisplacement( .088 ,  .0  , .0  ,  M_PI_2, .0,      .0);
  Vec6d l7;
  l7 << .0, .0, .107, .0, .0, -M_PI_4;
  Geometry::apply(l7, endEffector);
  addDisplacement(endEffector[0], endEffector[1], endEffector[2],
                  endEffector[3], endEffector[4], endEffector[5]);

  addJointLimits(-2.8973,  2.8973);
  addJointLimits(-1.7628,  1.7628);
  addJointLimits(-2.8973,  2.8973);
  addJointLimits(-3.0718, -0.0698);
  addJointLimits(-2.8973,  2.8973);
  addJointLimits(-0.0175,  3.7525);
  addJointLimits(-2.8973,  2.8973);
}

MatXd PandaKinematics::xToQ(CVec6dRef pose, const double& wrAngle, CVecXdRef qinit)
{
  //////////////////////////// should be done only once //////////////////////////
  // shoulder position
  const Vec3d shoulder(qToX(VecXd::Zero(2)).head(3));

  // define wrist circle radius: distance between wrist and z67
  const double wcRadius(getDisplacement(-2)[0]);

  // relative zero positions from j12 to j34 (upperarm) and from j34 to j56 (forearm)
  Vec3d upperarm(getDisplacement(3).head(3));
  Geometry::apply(getDisplacement(2), upperarm);
  Vec3d forearm(getDisplacement(5).head(3));
  Geometry::apply(getDisplacement(4), forearm);
  const double lenUa(upperarm.norm());
  const double lenFa(forearm.norm());
  const double ratio(lenUa / (lenUa + lenFa));

  // offsets between elbow and joint axes z23 and z45
  const double elbowOffset(getDisplacement(3)[0]);
  // angle offset between upper arm and virtual upper arm
  const double uaOffset(asin(elbowOffset / lenUa));
  // angle offset between forearm and virtual forearm
  const double faOffset(asin(elbowOffset / lenFa));
  ////////////////////////////////////////////////////////////////////////////////

  int debugKeyIndex(0), debugValIndex(0);

  MatXd qSol;
  qSol.setZero(4, 7);
  Vec3d ee(pose.head(3));

  // determine end effector's normal, open, and approach vector
  Vec3d eex(Vec3d::UnitX()), eey(Vec3d::UnitY()), eez(Vec3d::UnitZ());
  Geometry::apply(pose.tail(3), eex);
  Geometry::apply(pose.tail(3), eey);
  Geometry::apply(pose.tail(3), eez);

  // define wrist circle (wc): all possible wrist positions
  Vec3d wcCenter(ee - eez * getDisplacement(-1)[2]);

  // use projection of (shoulder to wc center) on eez to find x67
  Vec3d x67(wcCenter - (shoulder + eez * eez.dot(wcCenter - shoulder)));
  Geometry::apply(wrAngle * -eez, x67);
  x67.normalize();

  // find wrist by moving by wc radius along negative x67 from wc center
  Vec3d wrist(wcRadius * -x67);
  wrist += wcCenter;

  // find q67 from eex and x67 using a temporary vector (tmpVec)
  Vec3d tmpVec(eex);
  Vec3d l7inv(-getDisplacement(-1).tail(3));
  Geometry::apply(pose.tail(3), l7inv);
  Geometry::apply(l7inv, tmpVec);
  qSol(0,6) = acos(TRIG_RND(x67.dot(tmpVec)));
  // find correct sign
  Geometry::apply(-qSol(0,6) * eez, tmpVec);
  if (tmpVec.dot(x67) < 1 - TRIG_PREC) qSol(0,6) *= -1;
  for (int i : {1, 2, 3}) qSol(i,6) = qSol(0,6);

  // check workspace exceedance
  Vec3d wrSh = shoulder - wrist;
  double gap = wrSh.norm() - (lenUa + lenFa);
  if (gap > 0)
  {
    // move everything towards shoulder
    wrSh.normalize();
    wrSh *= gap + TRIG_PREC;

    wcCenter += wrSh;
    wrist += wrSh;
    ee += wrSh;
    wrSh = shoulder - wrist;
  }

  // find angle spanned by forearm and upper arm
  double cosShElWr((pow(lenUa, 2) + pow(lenFa, 2) - pow(wrSh.norm(), 2))
                    / (2 * lenUa * lenFa));
  double shElWr(acos(cosShElWr));
  qSol(0,3) = -M_PI + shElWr + faOffset + uaOffset;

  // check plausibility by comparison with forward kinematics
  tmpVec = qToX(qSol.row(0).head(5)).head(3) - shoulder;
  if (abs(tmpVec.norm() - wrSh.norm()) > TRIG_PREC)
    qSol(0,3) = -M_PI + shElWr - faOffset - uaOffset;
  for (int i : {1, 2, 3}) qSol(i,3) = qSol(0,3);

  // virtual elbow (ve): intersetion of z23 and z45
  double veExtension(0);
  double q3half((M_PI - abs(qSol(0,3))) / 2);
  if (sin(q3half) > TRIG_PREC) veExtension = elbowOffset * cos(q3half) / sin(q3half);

  // distance from ve to wrist
  double vewDist(forearm[1] + veExtension);

  // distance from ve to shoulder
  double vesDist(-upperarm[1] + veExtension);
  // setDebugValue(debugKeys, debugVals,
  //               debugKeyIndex, debugValIndex,
  //               SVE_RADIUS, &vesDist, 1);

  Vec3d veA, veB, z56(eez.cross(x67));
  Geometry::intCircleSphere(wrist, z56, vewDist, shoulder, vesDist, veA, veB);

  int sol = 0;
  for (Vec3d ve : {veA, veB})
  {
    // find z34 from virtual forearm (vf) and virtual upper arm (vu)
    Vec3d vf(wrist - ve), z45(vf.normalized());
    Vec3d vu(ve - shoulder), z23(vu.normalized());
    Vec3d z34;
    if (z45.dot(z23) > 1 - TRIG_PREC) z34 = z56;
    else z34 = z45.cross(z23).normalized();

    // find elbow from ve, z45, and z34
    Vec3d elbow(ve + veExtension * z45 + elbowOffset * z45.cross(z34));

    // calc q56 from z45 and eez
    qSol(sol,5) = z45.dot(x67) < 0 ? M_PI + acos(eez.dot(z45)) : acos(eez.dot(-z45));
    qSol(sol+1,5) = qSol(sol,5);

    // calc q45 from z34 and z56
    qSol(0,4) = acos(TRIG_RND(z34.dot(z56)));
    // find correct sign
    tmpVec = z34;
    Geometry::apply(qSol(0,4) * z45, tmpVec);
    if (tmpVec.dot(z56) < 1 - TRIG_PREC) qSol(0,4) *= -1;
    qSol(sol+1,4) = qSol(sol,4);

    // calc q12 from ve position
    qSol(sol,1) = acos(TRIG_RND((ve[2] - shoulder[2]) / vesDist));
    qSol(sol+1,1) = qSol(sol,1);
    if (qSol(sol,1) < TRIG_PREC)
    {
      double theta = acos((elbow - ve).head(2).normalized().dot(Vec2d::UnitX()));
      theta = std::copysign(theta, Vec2d::UnitY().dot((elbow - ve).head(2)));

      qSol(sol,0) = qinit[0];
      qSol(sol+1,0) = qSol(sol,0);

      qSol(sol,2) = theta - qSol(sol,0);
      qSol(sol+1,2) = qSol(sol,2);
    }
    else
    {
      double theta = atan2(ve[1], ve[0]);
      Vec3d z12;
      for (int i : {0, 1})
      {
        // calc q01
        qSol(sol,0) = theta - i * std::copysign(M_PI, theta);

        // calc z12 from q01
        z12 = Vec3d::UnitY();
        Geometry::apply(qSol(sol,0) * Vec3d::UnitZ(), z12);

        // calc q23 from z12
        qSol(sol,2) = acos(TRIG_RND(z12.dot(-z34)));
        tmpVec = z12;
        Geometry::apply(qSol(sol,2) * z23, tmpVec);
        // find correct sign
        if (tmpVec.dot(-z34) < 1 - TRIG_PREC) qSol(sol,2) *= -1;

        // find correct sign of q12 by applying to Z and compare with z23
        tmpVec = Vec3d::UnitZ();
        Geometry::apply(qSol(sol,1) * z12, tmpVec);
        if (tmpVec.dot(z23) < 1 - TRIG_PREC) qSol(sol,1) *= -1;

        sol++;
      }
    }
  }

  return qSol;
}

void panda_inv(double* pose, double wrist, double* qOut)
{
  static PandaKinematics panda;
  // Franka Hand:
  // panda.addDisplacement( .0   ,  .0  , .22 ,      .0, .0, -M_PI_4);
  // std::cout << "[ ";
  // for (int i : {0, 1, 2, 3, 4, 5, 6}) std::cout << qOut[i] << " ";
  // std::cout << "]" << std::endl;
  CVec6dMap x(pose);
  MatXdMap qOutMap(qOut, 4, 7);
  qOutMap = panda.xToQ(x, wrist, qOutMap.row(0));
}
