
#pragma once

#include "kinematics.h"


class PandaKinematics : public Kinematics
{
  public:
    static PandaKinematics& get(std::string name = "");
    static void add(std::string name, PandaKinematics pk);

    PandaKinematics(Vec6d endEffector = Vec6d::Zero());

    VecXd xToQ(CVec6dRef pose, const double& wrAngle = 0, CVecXdRef qinit = Vec7d::Zero());
    std::vector<VecXd> xToAllQ(CVec6dRef pose, const double& wrAngle = .0,
                               CVecRdRef<7> qinit = Vec7d::Zero());
    std::vector<VecXd> xToAllQElbow(CVec6dRef pose, const double& elbow = .0,
                                    CVecRdRef<7> qinit = Vec7d::Zero());

    double getWristAngle(CVecXdRef q, Vec6dRef pose);

    int getStatus();

    static const int STATUS_OK;
    static const int STATUS_TOO_FAR;

    static int _debug_level;
    static const int DEBUG_LOUD;
    static const int DEBUG_SILENT;
    static const int DEBUG_MUTE;

  private:
    // map for different end effectors
    static std::unordered_map<std::string, PandaKinematics> _pk_map;

    // retrieve ee axes and some other stuff needed for the wrist calculations
    void eeAxesFromPose(CVecXdRef pose);

    // status code for incidents while calculation
    int status;

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

    Vec3d tmp_vec;
    Vec3d help_vec;
    Vec3d ee_inv;
    Vec3d ee_pos;
    Vec3d ee_x;
    Vec3d ee_y;
    Vec3d ee_z;
    Vec3d wc_center;
    Vec3d x_67;
    Vec3d wrist_pos;
    Vec3d wc_sh;
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
  void panda_create(char* name, double* ee);

  int panda_ik(char* name, double* pose, double wrist, double* qOut, int sol = -1);

  int panda_fk(char* name, double* q, double* pose, int dof = 7);

  void panda_debug(int level);
}
