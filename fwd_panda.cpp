
#include "panda_kinematics.h"
#include <iostream>

int main(int argc, char** argv)
{
  Kinematics panda;
  PandaKinematics::setup(panda);
  VecXd q;
  q.setZero(7);

  for (int i = 0; i < std::min(7, argc - 1); i++)
    q[i] = std::atof(argv[i + 1]);

  Vec6d x(panda.qToX(q));
  std::cout << x.transpose() << std::endl;

}
