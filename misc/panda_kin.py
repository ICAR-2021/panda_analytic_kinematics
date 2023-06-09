#!/bin/python3

import sys
import numpy
from ctypes import c_char, c_int, cdll, c_double, POINTER

import file_utils as fu

class PandaKinematics:
  def __init__(self):
    libfile = fu.findFile("libPandaKin.so", '..') # 2nd argument is the library root path
    self.lib_panda = cdll.LoadLibrary(libfile)
    self.lib_panda.panda_create.argtypes = POINTER(c_char), POINTER(c_double)
    self.lib_panda.panda_ik.restype = c_int
    self.lib_panda.panda_ik.argtypes = POINTER(c_char), POINTER(c_double), c_double, POINTER(c_double), c_int
    self.lib_panda.panda_fk.restype = c_int
    self.lib_panda.panda_fk.argtypes = POINTER(c_char), POINTER(c_double), POINTER(c_double), c_int
    self.lib_panda.panda_debug.argtypes = c_int,
    self.lib_panda.panda_debug(1000)

  def setEndEffector(self, ee, name = ""):
    n_size = len(name)
    name = (c_char * n_size)(*name)
    ee_size = len(ee)
    ee = (c_double * ee_size)(*ee)
    self.lib_panda.panda_create(name, ee)

  def ik(self, pose, qInOut, name = "", optionals = [0, 0], results=[0]):
    wrist = optionals[0]
    solution = 0 if len(optionals) < 2 else optionals[1]
    angle = numpy.max(numpy.abs(pose[3:6]))
    norm = numpy.linalg.norm(pose[3:6])
    normPose = numpy.copy(pose)
    if norm > .001:
      normPose[3:6] = angle * normPose[3:6] / numpy.linalg.norm(normPose[3:6])
    p_size = len(normPose)
    normPose = (c_double * p_size)(*normPose)
    n_size = len(name)
    name = (c_char * n_size)(*name)
    q_size = len(qInOut)
    qInOut = (c_double * q_size)(*qInOut)
    results[0] = self.lib_panda.panda_ik(name, normPose, wrist, qInOut, solution)
    qInOut = numpy.array(qInOut[:])
    return qInOut

  def fk(self, joints, name = "", resultList=[0]):
    j_size = len(joints)
    joints = (c_double * j_size)(*joints)
    pose = (c_double * 7)(*numpy.zeros(6))
    n_size = len(name)
    name = (c_char * n_size)(*name)
    resultList[0] = self.lib_panda.panda_fk(name, joints, pose, j_size)
    pose = numpy.array(pose)
    return pose

if __name__ == "__main__":
  numpy.set_printoptions(suppress=True, precision=4)
  qRobot = numpy.zeros(7)
  angle = 0
  x = numpy.array([.4, .0, .7, 3.1416, 0, 0])
  args = sys.argv[1:6]
  x[:len(args)] = [a for a in args]
  kin = PandaKinematics()
  print("Goal joint config:", kin.ik(x, qRobot, [angle]), sep="\n  ");
