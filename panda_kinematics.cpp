
#include "panda_kinematics.h"
#include "geometry.h"
#include <iostream>

#define TRIG_PREC     1e-7
#define TRIG_RND(val) (round(val / TRIG_PREC) * TRIG_PREC)

PandaKinematics::PandaKinematics(Vec6d endEffector)
: shoulder_pos(.0, .0, .333),
  wc_radius(.088),
  upper_arm_zero(.0825, -.316, .0),
  forearm_zero(-.0825, .384, .0),
  len_ua(upper_arm_zero.norm()),
  len_fa(forearm_zero.norm()),
  elbow_offset(.0825),
  ua_offset(asin(elbow_offset / len_ua)),
  fa_offset(asin(elbow_offset / len_fa))
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

VecXd PandaKinematics::xToQ(CVec6dRef pose, const double& wrAngle, CVecXdRef qinit)
{
  MatRCd<4, 7> allQ = xToAllQ(pose, wrAngle, qinit);
  VecRd<4> diff = (allQ.transpose().colwise() - qinit).colwise().norm();
  int minSol = 0;
  for (int i = 1; i < allQ.rows(); i++)
  {
    if (diff[i] < diff[minSol]) minSol = i;
  }

  return allQ.row(minSol);
}

MatRCd<4, 7> PandaKinematics::xToAllQ(CVec6dRef pose, const double& wrAngle,
                                      CVecRdRef<7> qinit)
{
  MatRCd<4, 7> qSol;
  qSol.setZero();
  ee_pos << pose.head(3);

  // determine end effector's normal, open, and approach vector
  ee_x << Vec3d::UnitX();
  ee_y << Vec3d::UnitY();
  ee_z << Vec3d::UnitZ();
  Geometry::apply(pose.tail(3), ee_x);
  Geometry::apply(pose.tail(3), ee_y);
  Geometry::apply(pose.tail(3), ee_z);

  // define wrist circle (wc): all possible wrist positions
  wc_center << ee_pos - ee_z * getDisplacement(-1)[2];

  // use projection of (shoulder to wc center) on ee_z to find x_67
  x_67 << wc_center - (shoulder_pos + ee_z * ee_z.dot(wc_center - shoulder_pos));
  Geometry::apply(wrAngle * -ee_z, x_67);
  x_67.normalize();

  // find wrist by moving by wc radius along negative x_67 from wc center
  wrist_pos << wc_radius * -x_67;
  wrist_pos += wc_center;

  // find q67 from ee_x and x_67 using a temporary vector (tmp_vec)
  tmp_vec << ee_x;
  ee_inv = -getDisplacement(-1).tail(3);
  Geometry::apply(pose.tail(3), ee_inv);
  Geometry::apply(ee_inv, tmp_vec);
  qSol(0,6) = acos(TRIG_RND(x_67.dot(tmp_vec)));
  // find correct sign
  Geometry::apply(-qSol(0,6) * ee_z, tmp_vec);
  if (tmp_vec.dot(x_67) < 1 - TRIG_PREC) qSol(0,6) *= -1;
  for (int i : {1, 2, 3}) qSol(i,6) = qSol(0,6);

  // check workspace exceedance
  wr_sh << shoulder_pos - wrist_pos;
  double elbow_gap = wr_sh.norm() - (len_ua + len_fa);
  if (elbow_gap > 0)
  {
    // move everything towards shoulder
    wr_sh.normalize();
    wr_sh *= elbow_gap + TRIG_PREC;

    wc_center += wr_sh;
    wrist_pos += wr_sh;
    ee_pos += wr_sh;
    wr_sh << shoulder_pos - wrist_pos;
  }

  // find angle spanned by forearm and upper arm
  double cos_sh_el_wr((pow(len_ua, 2) + pow(len_fa, 2) - pow(wr_sh.norm(), 2))
                    / (2 * len_ua * len_fa));
  double sh_el_wr(acos(cos_sh_el_wr));
  qSol(0,3) = -M_PI + sh_el_wr - fa_offset - ua_offset;

  // check plausibility by comparison with forward kinematics
  tmp_vec << qToX(qSol.row(0).head(5)).head(3) - shoulder_pos;
  if (abs(tmp_vec.norm() - wr_sh.norm()) > TRIG_PREC)
    qSol(0,3) = -M_PI + sh_el_wr + fa_offset + ua_offset;
  for (int i : {1, 2, 3}) qSol(i,3) = qSol(0,3);

  // virtual elbow (ve): intersetion of z23 and z45
  double ve_extension(0);
  double q3_2((M_PI - abs(qSol(0,3))) / 2);
  if (sin(q3_2) > TRIG_PREC) ve_extension = elbow_offset * cos(q3_2) / sin(q3_2);

  // distance from ve to wrist
  double vew_dist(forearm_zero[1] + ve_extension);

  // distance from ve to shoulder
  double ves_dist(-upper_arm_zero[1] + ve_extension);
  // setDebugValue(debugKeys, debugVals,
  //               debugKeyIndex, debugValIndex,
  //               SVE_RADIUS, &ves_dist, 1);

  z_56 << ee_z.cross(x_67);
  Geometry::intCircleSphere(wrist_pos, z_56, vew_dist, shoulder_pos, ves_dist, ve_a, ve_b);

  int sol = 0;
  for (Vec3d ve : {ve_a, ve_b})
  {
    // find z34 from virtual forearm (v_f) and virtual upper arm (v_u)
    v_f << wrist_pos - ve;
    z_45 << v_f.normalized();
    v_u << ve - shoulder_pos;
    z_23 << v_u.normalized();

    if (z_45.dot(z_23) > 1 - TRIG_PREC) z_34 << z_56;
    else z_34 << z_45.cross(z_23).normalized();

    // find elbow from ve, z45, and z34
    elbow_pos << ve + ve_extension * z_45 + elbow_offset * z_45.cross(z_34);

    // calc q56 from z45 and eez
    qSol(sol,5) = z_45.dot(x_67) < 0 ? M_PI + acos(ee_z.dot(z_45)) : acos(ee_z.dot(-z_45));
    qSol(sol+1,5) = qSol(sol,5);

    // calc q45 from z34 and z56
    qSol(sol,4) = acos(TRIG_RND(z_34.dot(z_56)));
    // find correct sign
    tmp_vec << z_34;
    Geometry::apply(qSol(sol,4) * z_45, tmp_vec);
    if (tmp_vec.dot(z_56) < 1 - TRIG_PREC) qSol(sol,4) *= -1;
    qSol(sol+1,4) = qSol(sol,4);

    // calc q12 from ve position
    qSol(sol,1) = acos(TRIG_RND((ve[2] - shoulder_pos[2]) / ves_dist));
    qSol(sol+1,1) = qSol(sol,1);
    if (qSol(sol,1) < TRIG_PREC)
    {
      double theta_ = acos((elbow_pos - ve).head(2).normalized().dot(Vec2d::UnitX()));
      theta_ = std::copysign(theta_, Vec2d::UnitY().dot((elbow_pos - ve).head(2)));

      qSol(sol,0) = qinit[0];
      qSol(sol+1,0) = qSol(sol,0);

      qSol(sol,2) = theta_ - qSol(sol,0);
      qSol(sol+1,2) = qSol(sol,2);
    }
    else
    {
      double theta_ = atan2(ve[1], ve[0]);
      for (int i : {0, 1})
      {
        // calc q01
        qSol(sol,0) = theta_ - i * std::copysign(M_PI, theta_);

        // calc z12 from q01
        z_12 << Vec3d::UnitY();
        Geometry::apply(qSol(sol,0) * Vec3d::UnitZ(), z_12);

        // calc q23 from z12
        qSol(sol,2) = acos(TRIG_RND(z_12.dot(-z_34)));
        tmp_vec << z_12;
        Geometry::apply(qSol(sol,2) * z_23, tmp_vec);
        // find correct sign
        if (tmp_vec.dot(-z_34) < 1 - TRIG_PREC) qSol(sol,2) *= -1;

        // find correct sign of q12 by applying to Z and compare with z23
        tmp_vec << Vec3d::UnitZ();
        Geometry::apply(qSol(sol,1) * z_12, tmp_vec);
        if (tmp_vec.dot(z_23) < 1 - TRIG_PREC) qSol(sol,1) *= -1;

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
  VecXdMap qOutMap(qOut, 7);
  qOutMap = panda.xToQ(x, wrist, qOutMap);
}
