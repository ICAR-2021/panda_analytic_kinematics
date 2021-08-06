# Analytical Kinematics Library for the Franka Emika Panda Robot

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
2.37916e-05    -0.53039           0    -1.61396           0     1.08357    0.785408
$ ./inv_panda .3 .0 .6 3.1416 .0 .0 .7854
  1.55525 -0.876649 -0.539861  -1.54676 -0.564642  0.830546   1.57081
```

For the forward kinematics, the seven arguments are the seven joint angles and the output is the end effector pose.

For the inverse kinematics, the first six arguments describe the end effector pose.
If a seventh argument is given, it will be used for redundancy resolution defining an angle on the wrist circle.

The poses are give by six parameters.
The first three parameters define the x, y, and z coordinate.
The last three describe the rotation in an axis angle representation where the norm represents the rotation angle in radians.

