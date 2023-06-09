
#include "panda_kinematics.h"
#include "geometry.h"
#include <iostream>

#define PK_OUT std::cout << "[PandaKin] "

#define TRIG_PREC     1e-7
#define TRIG_RND(val) (round(val / TRIG_PREC) * TRIG_PREC)

const int PandaKinematics::STATUS_OK = 0;
const int PandaKinematics::STATUS_TOO_FAR = 1;

const int PandaKinematics::DEBUG_LOUD = 100;
const int PandaKinematics::DEBUG_SILENT = 1000;
const int PandaKinematics::DEBUG_MUTE = INT_MAX;

int PandaKinematics::_debug_level = PandaKinematics::DEBUG_MUTE;

std::unordered_map<std::string, PandaKinematics> PandaKinematics::_pk_map = {};

PandaKinematics::PandaKinematics(Vec6d endEffector)
: shoulder_pos(.0, .0, .333),
  wc_radius(.088),
  upper_arm_zero(.0825, -.316, .0),
  forearm_zero(-.0825, .384, .0),
  len_ua(upper_arm_zero.norm()),
  len_fa(forearm_zero.norm()),
  elbow_offset(.0825),
  ua_offset(asin(elbow_offset / len_ua)),
  fa_offset(asin(elbow_offset / len_fa)),
  ee_x(1, 0, 0),
  ee_y(0, 1, 0),
  ee_z(0, 0, 1)
{
  addDisplacement( .0   ,  .0  , .14 ,      .0, .0,      .0);
  addDisplacement( .0   ,  .0  , .193, -M_PI_2, .0,      .0);
  addDisplacement( .0   , -.192, .0  ,  M_PI_2, .0,      .0);
  addDisplacement( .0825,  .0  , .124,  M_PI_2, .0,      .0);
  addDisplacement(-.0825,  .124, .0  , -M_PI_2, .0,      .0);
  addDisplacement( .0   ,  .0  , .26 ,  M_PI_2, .0,      .0);
  addDisplacement( .088 , -.107, .0  ,  M_PI_2, .0,      .0);
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
  std::vector<VecXd> allQ = xToAllQ(pose, wrAngle, qinit);

  double diff, maxDiff, minMax = 2 * M_PI;
  int minMaxSol = 0;
  for (int i = 0; i < allQ.size(); i++)
  {
    maxDiff = 0;
    for (int j = 0; j < qinit.size(); j++)
    {
      diff = Geometry::getAngularDifference(allQ[i][j], qinit[j]);
      if (diff > maxDiff) maxDiff = diff;
    }
    if (maxDiff < minMax)
    {
      minMax = maxDiff;
      minMaxSol = i;
    }
  }

  return allQ[minMaxSol];
}

std::vector<VecXd> PandaKinematics::xToAllQElbow(CVec6dRef pose, const double& elbow,
                                                 CVecRdRef<7> qinit)
{
  status = STATUS_OK;

  std::vector<VecXd> qSol;
  for (int i = 0; i < 4; i++) qSol.push_back(Vec7d::Zero());

  eeAxesFromPose(pose);

  wc_sh = shoulder_pos - wc_center;
  wrist_pos = wc_sh - Geometry::project(ee_z, wc_sh);
  wrist_pos = wrist_pos.normalized() * wc_radius;
  PK_OUT << "minimum circle projection: " << wrist_pos.transpose() << std::endl;
  wrist_pos += wc_center;

  PK_OUT << "minimum wrist:  " << wrist_pos.transpose() << std::endl;

  return qSol;
}

std::vector<VecXd> PandaKinematics::xToAllQ(CVec6dRef pose, const double& wrAngle,
                                            CVecRdRef<7> qinit)
{
  status = STATUS_OK;

  std::vector<VecXd> qSol;
  for (int i = 0; i < 4; i++) qSol.push_back(Vec7d::Zero());

  eeAxesFromPose(pose);

  // apply wrist (redundancy) angle
  Geometry::apply(wrAngle * -ee_z, x_67);
  x_67.normalize();

  // find wrist by moving by wc_radius along negative x_67 from wc_center
  wrist_pos << wc_radius * -x_67;
  wrist_pos += wc_center;

  // find q67 from ee_x and x_67 using a temporary vector (tmp_vec)
  tmp_vec << ee_x;
  ee_inv = -getDisplacement(-1).tail(3);
  Geometry::apply(pose.tail(3), ee_inv);
  Geometry::apply(ee_inv, tmp_vec);
  qSol[0][6] = acos(TRIG_RND(x_67.dot(tmp_vec)));
  // find correct sign
  help_vec << -qSol[0][6] * ee_z;
  Geometry::apply(help_vec, tmp_vec);
  if (tmp_vec.dot(x_67) < 1 - TRIG_PREC) qSol[0][6] *= -1;
  for (int i : {1, 2, 3}) qSol[i][6] = qSol[0][6];

  // check workspace exceedance
  wr_sh << shoulder_pos - wrist_pos;
  double elbow_gap = wr_sh.norm() - (len_ua + len_fa);
  if (elbow_gap > 0)
  {
    status += STATUS_TOO_FAR;

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
  qSol[0][3] = -M_PI + sh_el_wr - fa_offset - ua_offset;

  // check plausibility by comparison with forward kinematics
  tmp_vec << qToX(qSol[0].head(5)).head(3) - shoulder_pos;
  if (abs(tmp_vec.norm() - wr_sh.norm()) > TRIG_PREC)
    qSol[0][3] = -M_PI + sh_el_wr + fa_offset + ua_offset;
  for (int i : {1, 2, 3}) qSol[i][3] = qSol[0][3];

  // virtual elbow (ve): intersetion of z23 and z45
  double ve_extension(0);
  double q3_2((M_PI - abs(qSol[0][3])) / 2);
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

  if ((ve_a - wc_center).norm() < (ve_b - wc_center).norm()) ve_a.swap(ve_b);

  int sol = 0;
  double theta;
  for (Vec3d ve : {ve_a, ve_b})
  {
    // find z34 from virtual forearm (v_f) and virtual upper arm (v_u)
    v_f << wrist_pos - ve;
    z_45 << v_f.normalized();
    v_u << ve - shoulder_pos;
    z_23 << v_u.normalized();

    // FIXME!
    if (z_45.dot(z_23) > 1 - TRIG_PREC) z_34 << z_56;
    else z_34 << z_45.cross(z_23).normalized();

    // find elbow from ve, z45, and z34
    elbow_pos << ve + ve_extension * z_45 + elbow_offset * z_45.cross(z_34);

    // calc q56 from z45 and eez
    qSol[sol][5] = z_45.dot(x_67) < 0 ? M_PI + acos(ee_z.dot(z_45)) : acos(ee_z.dot(-z_45));
    qSol[sol+1][5] = qSol[sol][5];

    // calc q45 from z34 and z56
    qSol[sol][4] = acos(TRIG_RND(z_34.dot(z_56)));
    // find correct sign
    tmp_vec << z_34;
    help_vec << qSol[sol][4] * z_45;
    Geometry::apply(help_vec, tmp_vec);
    if (tmp_vec.dot(z_56) < 1 - TRIG_PREC) qSol[sol][4] *= -1;
    qSol[sol+1][4] = qSol[sol][4];

    // calc q12 from ve position
    qSol[sol][1] = acos(TRIG_RND((ve[2] - shoulder_pos[2]) / ves_dist));
    qSol[sol+1][1] = qSol[sol][1];
    if (qSol[sol][1] < TRIG_PREC)
    {
      theta = acos((elbow_pos - ve).head(2).normalized().dot(Vec2d::UnitX()));
      theta = std::copysign(theta, Vec2d::UnitY().dot((elbow_pos - ve).head(2)));

      qSol[sol][0] = qinit[0];
      qSol[sol+1][0] = qSol[sol][0];

      qSol[sol][2] = theta - qSol[sol][0];
      qSol[sol+1][2] = qSol[sol][2];

      sol += 2;
    }
    else
    {
      // calc q01
      qSol[sol][0] = atan2(-ve[1], -ve[0]);
      qSol[sol+1][0] = qSol[sol][0] + M_PI;
      for (int i : {0, 1})
      {
        // calc z12 from q01
        z_12 << Vec3d::UnitY();
        help_vec = qSol[sol][0] * Vec3d::UnitZ();
        Geometry::apply(help_vec, z_12);

        // calc q23 from z12
        qSol[sol][2] = acos(TRIG_RND(z_12.dot(-z_34)));
        tmp_vec << z_12;
        help_vec = qSol[sol][2] * z_23;
        Geometry::apply(help_vec, tmp_vec);
        // find correct sign
        if (tmp_vec.dot(-z_34) < 1 - TRIG_PREC) qSol[sol][2] *= -1;

        // find correct sign by checking distance to ve when using the calculated q12
        tmp_vec << 0, 0, ves_dist;
        help_vec = qSol[sol][1] * z_12;
        Geometry::apply(help_vec, tmp_vec);
        double dist = (ve - (shoulder_pos + tmp_vec)).norm();
        tmp_vec << 0, 0, ves_dist;
        help_vec = -qSol[sol][1] * z_12;
        Geometry::apply(help_vec, tmp_vec);
        if (dist > (ve - (shoulder_pos + tmp_vec)).norm()) qSol[sol][1] *= -1;

        sol++;
      }
    }
  }

  for (VecXdRef q : qSol) checkMean(q);

  return qSol;
}

void PandaKinematics::eeAxesFromPose(CVecXdRef pose)
{
  // determine end effector's normal, open, and approach vector
  ee_x << Vec3d::UnitX();
  ee_y << Vec3d::UnitY();
  ee_z << Vec3d::UnitZ();
  Geometry::apply(pose.tail(3), ee_x);
  Geometry::apply(pose.tail(3), ee_y);
  Geometry::apply(pose.tail(3), ee_z);

  // read end effector position
  ee_pos << pose.head(3);

  Vec6d dTail = getDisplacement(-1);
  Vec6d d2ndLast = getDisplacement(-2);
  d2ndLast[0] = 0;
  Geometry::apply(d2ndLast, dTail);
  dTail *= -1;
  Geometry::apply(dTail.tail(3), dTail.head(3));
  Geometry::apply(pose.tail(3), dTail.head(3));

  // define wrist circle (wc): all possible wrist positions
  wc_center << ee_pos + dTail.head(3);

  // // use projection of (shoulder to wc_center) on ee_z to find x_67
  // x_67 << wc_center - (shoulder_pos + ee_z * ee_z.dot(wc_center - shoulder_pos));

  // // use axis perpendicular to local z and global y axis as x_67
  // x_67 << ee_z.cross(Vec3d::UnitY());
  // if (ee_z.z() > 0) x_67 *= -1;

  // use axis between end effector's x and y axis as x_67
  x_67 << ee_x;

}

double PandaKinematics::getWristAngle(CVecXdRef q, Vec6dRef pose)
{
  if (pose.norm() == 0) pose = qToX(q);

  eeAxesFromPose(pose);

  // get x_67 for redundancy angle = 0
  Vec3d x_67_0 = x_67.normalized();

  // get x_67 for the actual joint config
  x_67 = Vec3d::UnitX();
  Vec3d rot_67 = qToX(q.head(6)).tail(3);
  Geometry::apply(rot_67, x_67);
  x_67.normalize();

  Vec3d y_67_0 = ee_z.cross(x_67_0).normalized();
  double angle = acos(x_67.dot(x_67_0));

  return acos(x_67.dot(y_67_0)) > M_PI_2 ? angle : -angle;
}

int PandaKinematics::getStatus()
{
  return status;
}

PandaKinematics& PandaKinematics::get(std::string name)
{
  if (_pk_map.find(name) == _pk_map.end()) PandaKinematics::add(name, PandaKinematics());
  return _pk_map[name];
}

void PandaKinematics::add(std::string name, PandaKinematics pk)
{
  _pk_map.insert(std::pair<std::string, PandaKinematics>(name, pk));
}

void panda_create(char* name, double* ee)
{
  CVec6dMap endEffector(ee);
  PK_OUT << "End effector: " << endEffector.transpose() << std::endl;
  PandaKinematics::add(std::string(name), PandaKinematics(endEffector));
}

int panda_ik(char* name, double* pose, double wrist, double* qOut, int sol)
{
  PandaKinematics panda = PandaKinematics::get(std::string(name));

  // // Franka Hand:
  // panda.addDisplacement( .0   ,  .0  , .22 ,      .0, .0, -M_PI_4);
  // std::cout << "[ ";
  // for (int i : {0, 1, 2, 3, 4, 5, 6}) std::cout << qOut[i] << " ";
  // std::cout << "]" << std::endl;

  CVec6dMap x(pose);
  VecXdMap qOutMap(qOut, 7);

  panda.xToAllQElbow(x);

  if (sol < 0 || sol > 3) qOutMap = panda.xToQ(x, wrist, qOutMap);
  else qOutMap = panda.xToAllQ(x, wrist, qOutMap)[sol];

  return panda.getStatus();
}

int panda_fk(char* name, double* q, double* pose, int dof)
{
  PandaKinematics panda = PandaKinematics::get();

  CVecXdMap qVec(q, dof);
  VecXdMap poseVec(pose, 7);

  poseVec[6] = panda.getWristAngle(qVec, poseVec.head(6));

  return 0;
}

void panda_debug(int level)
{
  PandaKinematics::_debug_level = level;
}
