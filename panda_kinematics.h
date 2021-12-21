
#pragma once

#include "kinematics.h"


class PandaKinematics : public Kinematics
{
  public:
    PandaKinematics(Vec6d endEffector = Vec6d::Zero());
    VecXd xToQ(CVec6dRef pose, const double& wrAngle = 0, CVecXdRef qinit = Vec7d::Zero());
    MatRCd<4, 7> xToAllQ(CVec6dRef pose, const double& wrAngle = .0,
                         CVecRdRef<7> qinit = Vec7d::Zero());
  private:
    // shoulder position
    const Vec3d shoulder_pos;

    // define wrist circle radius: distance between wrist and z67
    const double wc_radius;

    // relative zero positions from j12 to j34 (upperarm) and from j34 to j56 (forearm)
    const Vec3d upper_arm_zero;
    const Vec3d forearm_zero;
    const double len_ua;
    const double len_fa;

    // offsets between elbow and joint axes z23 and z45
    const double elbow_offset;
    // angle offset between upper arm and virtual upper arm
    const double ua_offset;
    // angle offset between forearm and virtual forearm
    const double fa_offset;

    Vec3d ee_inv;
    Vec3d ee_pos;
    Vec3d ee_x;
    Vec3d ee_y;
    Vec3d ee_z;
    Vec3d wc_center;
    Vec3d x_67;
    Vec3d wrist_pos;
    Vec3d tmp_vec;
    Vec3d wr_sh;
    Vec3d ve_a;
    Vec3d ve_b;
    Vec3d z_56;
    Vec3d v_f;
    Vec3d v_u;
    Vec3d z_34;
    Vec3d z_23;
    Vec3d z_45;
    Vec3d elbow_pos;
    Vec3d z_12;
};

extern "C"
{
  void panda_inv(double* pose, double wrist, double* qOut);
}
