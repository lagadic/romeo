import sys; sys.path.append('..')

import rbdyn as rbd

import romeo_common

mb, mbc, mbg, limits, visual_tf, collision_tf =\
  romeo_common.readUrdf('romeo', romeo_common.rootBody,
                       romeo_common.virtualJoints, romeo_common.handJoints,
                       romeo_common.halfSitting)


def robot(fixed=False):
  if not fixed:
    return mb, mbc, mbg
  else:
    mbF = mbg.makeMultiBody(mbg.bodyIdByName(romeo_common.rootBody), True)
    mbcF = rbd.MultiBodyConfig(mbF)
    mbcF.zero(mbF)
    return mbF, mbcF, mbg


def convexHull():
  fileByBodyName = romeo_common.stdCollisionsFiles(mb)
  return romeo_common.convexHull(fileByBodyName, mb)


def stpbvHull():
  fileByBodyName = romeo_common.stdCollisionsFiles(mb)
  return romeo_common.stpbvHull(fileByBodyName, mb)


def collisionTransforms():
  return collision_tf


def bounds():
  return romeo_common.nominalBounds(limits)


def stance():
  return romeo_common.halfSittingPose(mb), ('Lfoot', 'Rfoot')


def forceSensors():
  return romeo_common.forceSensors


def accelerometerBody():
  return romeo_common.accelBody
