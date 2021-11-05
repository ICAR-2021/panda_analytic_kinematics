# Analytical Kinematics Library for the Franka Emika Panda Robot

![IK used in pyBullet (code will follow soon)](misc/panda_pybullet.gif)

## Prerequisites

You need a current version of the Eigen 3 library (https://eigen.tuxfamily.org) installed.

## Build instructions

The repository includes a `CMakeLists.txt` and can be build from terminal within the project directory as follows:

```
$ mkdir -p build && cd build
$ cmake -D CMAKE_BUILD_TYPE=RelWithDebInfo ..
$ make PandaKin     # builds the library
$ make inv_panda    # builds the executable binary for the inverse kinematics
$ make fwd_panda    # builds the executable binary for the forward kinematics
```

The executable binaries, of course, also include the library.
Hence, `make PandaKin` can be omitted when building the executables.

## Usage

Examples of how to use the library can be found in `inv_panda.cpp` and `fwd_panda.cpp`.
The binary executables can be used as follows from terminal within the project's build folder:

```
$ ./fwd_panda .0 .0 .0 .0 .0 .0 .0
      0.088 -1.72675e-17        0.926     -2.90245     -1.20224  1.74393e-16
$ ./inv_panda .3 .0 .6 3.1416 .0 .0
  Solutions:
  8.49658e-06      1.8313     3.14159    -2.33898     3.14159      5.7755    0.785405
     -3.14158     -1.8313           0    -2.33898     3.14159      5.7755    0.785405
     -3.14158    0.799934     3.14159    -2.33898           0     1.53905    0.785405
  1.36742e-05   -0.799934           0    -2.33898           0     1.53905    0.785405
  Time: 0.000427728sec
$ ./inv_panda .3 .0 .6 3.1416 .0 .0 .7854 # ik with redundancy angle
  Solutions:
    0.33343   1.76137  -2.81694  -2.27643  -2.54313    5.6937    1.5708
   -2.80816  -1.76137  0.324652  -2.27643  -2.54313    5.6937    1.5708
   -1.87496   1.18498   2.53121  -2.27643 -0.598458   1.23041    1.5708
    1.26663  -1.18498 -0.610387  -2.27643 -0.598458   1.23041    1.5708
$ ./inv_panda .3 .0 .6 3.1416 .0 .0 .7854 # performance measure
  Time: 0.0325822sec
```

For the forward kinematics, the (up to) seven arguments are the seven joint angles and the output is the end effector pose (or the joint pose if less than seven arguments were given).

For the inverse kinematics, the first six arguments describe the end effector pose.
If a seventh argument is given, it will be used for redundancy resolution defining an angle on the wrist circle.

The poses are given by six parameters.
The first three parameters define the x, y, and z coordinate.
The last three describe the rotation in an axis angle representation where the norm represents the rotation angle in radians.

