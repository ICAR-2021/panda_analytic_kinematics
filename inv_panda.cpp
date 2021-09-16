
#include "panda_kinematics.h"
#include <iostream>

int main(int argc, char** argv)
{
  PandaKinematics panda;
  Vec6d x;
  x.setZero();

  for (int i = 0; i < std::min(6, argc - 1); i++)
    x[i] = std::atof(argv[i + 1]);

  MatXd qout(4, 7);
  Eigen::IOFormat format(-1, 0, " ", "\n", "  ");

  // for (int i = 0; i < 10000; i++)
  panda_inv(x.data(), argc > 7 ? std::atof(argv[7]) : 0, qout.data());

  std::cout << "  Solutions:\n" << qout.format(format) << std::endl;
}
