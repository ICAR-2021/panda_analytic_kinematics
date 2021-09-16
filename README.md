# Analytical Kinematics Library for the Franka Emika Panda Robot

![IK used in pyBullet (code will follow soon)](misc/panda_pybullet.gif)

## Prerequisites

You need a current version of the Eigen 3 library (https://eigen.tuxfamily.org) installed.

## Build instructions

The repository includes a `CMakeLists.txt` and can be build from terminal within the project directory as follows:

```
$ mkdir -p build && cd build
$ cmake ..
$ make PandaKin     # builds the library
$ make inv_panda    # builds the executable binary for the inverse kinematics
$ make fwd_panda    # builds the executable binary for the forward kinematics
```

The executable binaries, of course, also include the binary.
Hence, this step can be omitted when building the executables.

## Usage

Examples of how to use the library can be found in `inv_panda.cpp` and `fwd_panda.cpp`.
The binary executables can be used as follows from terminal within the project's build folder:

```
$ ./fwd_panda .0 .0 .0 .0 .0 .0 .0
      0.088 -4.1638e-18       1.033     3.14159           0           0
$ ./inv_panda .3 .0 .6 3.1416 .0 .0
  Solutions:
  8.49658e-06      1.8313     3.14159    -2.33898           0      5.7755    0.785405
     -3.14158     -1.8313           0    -2.33898     3.14159      5.7755    0.785405
     -3.14158    0.799934     3.14159    -2.33898           0     1.53905    0.785405
  1.36742e-05   -0.799934           0    -2.33898           0     1.53905    0.785405
$ ./inv_panda .3 .0 .6 3.1416 .0 .0 .7854
  Solutions:
  0.0932566    1.9532  -3.06397  -2.56538         0   6.16648  0.831459
   -3.04834   -1.9532 0.0776236  -2.56538   3.14159   6.16648  0.831459
   -3.02833  0.974079   3.05451  -2.56538         0    2.0875  0.831459
   0.113262 -0.974079 -0.087085  -2.56538         0    2.0875  0.831459
```

For the forward kinematics, the seven arguments are the seven joint angles and the output is the end effector pose.

For the inverse kinematics, the first six arguments describe the end effector pose.
If a seventh argument is given, it will be used for redundancy resolution defining an angle on the wrist circle.

The poses are give by six parameters.
The first three parameters define the x, y, and z coordinate.
The last three describe the rotation in an axis angle representation where the norm represents the rotation angle in radians.

