
#include "panda_kinematics.h"
#include <iostream>

int main(int argc, char** argv)
{
  Kinematics panda;
  PandaKinematics::setup(panda);
  Vec6d x;
  x.setZero();

  for (int i = 0; i < std::min(6, argc - 1); i++)
    x[i] = std::atof(argv[i + 1]);

  if (argc < 8)
  {
    double qout[7];
    panda_inv(x.data(), .0, qout);
  }

  VecXd q(panda.xToQ(x, argc > 7 ? std::atof(argv[7]) : 0));
  std::cout << q.transpose() << std::endl;

}
