
#include "panda_kinematics.h"
#include <iostream>

int main(int argc, char** argv)
{
  PandaKinematics panda;
  std::vector<Vec6d> poses;
  std::vector<double> angles({0});

  Vec6d pose;
  if (argc < 2)
  {
    // performance test
    pose << .3, 0, .6, 3.1416, 0, 0;
    srand(time(NULL));
    for (int i = 0; i < 1000; i++) poses.push_back(pose + Vec6d::Random() * .2);
    for (int i = 0; i < 4; i++) angles.push_back(rand() * 1.5708 / RAND_MAX);
  }
  else
  {
    pose.setZero();
    for (int i = 1; i < argc; i++)
      if (i < 7) pose[i - 1] = std::atof(argv[i]);
      else angles.push_back(std::atof(argv[i]));
    poses.push_back(pose);
  }

  MatXd qout(4, 7);
  Eigen::IOFormat format(-1, 0, " ", "\n", "  ");

  std::chrono::high_resolution_clock::time_point start;
  Duration duration;
  start = std::chrono::high_resolution_clock::now();
  for (Vec6d x : poses)
    for (double angle : angles)
      qout = panda.xToAllQ(x, angle);

  duration = std::chrono::high_resolution_clock::now() - start;

  if (poses.size() > 1)
    std::cout << "  Time: " << duration.count() << "sec" << std::endl;
  else
    std::cout << "  Solutions:\n" << qout.format(format) << std::endl;
}
