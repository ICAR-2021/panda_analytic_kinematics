
#include "panda_kinematics.h"
#include <iostream>

int main(int argc, char** argv)
{
  PandaKinematics panda;
  VecXd q;
  q.setZero(argc - 1);

  for (int i = 0; i < q.size(); i++)
    q[i] = std::atof(argv[i + 1]);

  Vec6d x(panda.qToX(q));
  std::cout << "  " << x.transpose() << std::endl;

}
