import roslib; roslib.load_manifest('romeo_description_mc')

import os

import numpy as np

from eigen3 import Vector3d, toEigen
import spacevecalg as sva
import rbdyn as rbd

import mc_rbdyn_urdf


path = roslib.packages.get_pkg_dir('romeo_description_mc')


virtualJoints = [
  'l_wrist',
  'r_wrist',
  'l_gripper',
  'r_gripper',
  'base_link_fixedjoint',
  'gaze_joint',
  'LLeg_effector_fixedjoint',
  'RLeg_effector_fixedjoint',
  'RFoot/FSR/ToeRearRight_sensor_fixedjoint',
  'CameraLeftEye_sensor_fixedjoint',
  'GyrometerHead_sensor_fixedjoint',
  'RFoot/FSR/CenterRear_sensor_fixedjoint',
  'RFoot/FSR/CenterFront_sensor_fixedjoint',
  'CameraLeftEye_optical_frame_fixedjoint',
  'LFoot/FSR/HeelRight_sensor_fixedjoint',
  'CameraRightEye_sensor_fixedjoint',
  'CameraRight_optical_frame_fixedjoint',
  'CameraRightEye_optical_frame_fixedjoint',
  'RFoot/FSR/ToeFrontRight_sensor_fixedjoint',
  'RFoot/FSR/HeelRight_sensor_fixedjoint',
  'RFoot/FSR/ToeRearLeft_sensor_fixedjoint',
  'AccelerometerDownTorso_sensor_fixedjoint',
  'LFoot/FSR/ToeRearRight_sensor_fixedjoint',
  'LFoot/FSR/HeelLeft_sensor_fixedjoint',
  'CameraRight_sensor_fixedjoint',
  'GyrometerDownTorso_sensor_fixedjoint',
  'LFoot/FSR/ToeRearLeft_sensor_fixedjoint',
  'LFoot/FSR/CenterRear_sensor_fixedjoint',
  'CameraLeft_sensor_fixedjoint',
  'CameraLeft_optical_frame_fixedjoint',
  'LFoot/FSR/ToeFrontRight_sensor_fixedjoint',
  'RFoot/FSR/HeelLeft_sensor_fixedjoint',
  'AccelerometerHead_sensor_fixedjoint',
  'LFoot/FSR/CenterFront_sensor_fixedjoint',
  'RFoot/FSR/ToeFrontLeft_sensor_fixedjoint',
  'LFoot/FSR/ToeFrontLeft_sensor_fixedjoint',
  'CameraDepth_joint',
  'Cap_joint'
]


handJoints = [
  'LHand',
  'RHand',
  'LThumb1',
  'LFinger11',
  'LFinger21',
  'LFinger31',
  'RThumb1',
  'RFinger11',
  'RFinger21',
  'RFinger31',
]


forceSensors = [
]


rootBody = 'torso'

accelBody = "torso"


halfSitting = {
  "NeckYaw": 0.,
  "NeckPitch": 0.,
  "HeadPitch": 0.,
  "HeadRoll": 0.,

  "LHipYaw": 0.,
  "LHipRoll": 0.,
  "LHipPitch": -0.3490658,
  "LKneePitch": 0.6981317,
  "LAnklePitch": -0.3490658,
  "LAnkleRoll": 0.,

  "RHipYaw": 0.,
  "RHipRoll": 0.,
  "RHipPitch": -0.3490658,
  "RKneePitch": 0.6981317,
  "RAnklePitch": -0.3490658,
  "RAnkleRoll": 0.,

  "TrunkYaw": 0.,

  "LShoulderPitch": 1.75,
  "LShoulderYaw": 0.,
  "LElbowRoll": -1.30,
  "LElbowYaw": -0.17,
  "LWristRoll": -0.35,
  "LWristYaw": -0.,
  "LWristPitch": 0.,
  "LHand": 0.1,

  "RShoulderPitch": 1.75,
  "RShoulderYaw": 0.,
  "RElbowRoll": 1.30,
  "RElbowYaw": 0.17,
  "RWristRoll": 0.35,
  "RWristYaw": 0.,
  "RWristPitch": 0.,
  "RHand": 0.9,

  "LEyeYaw": 0.,
  "LEyePitch": 0.,
  "REyeYaw": 0.,
  "REyePitch": 0.,

  "LFinger11": 0.,
  "LFinger12": 0.,
  "LFinger13": 0.,
  "LFinger21": 0.,
  "LFinger22": 0.,
  "LFinger23": 0.,
  "LFinger31": 0.,
  "LFinger32": 0.,
  "LFinger33": 0.,
  "LThumb1": 0.,
  "LThumb2": 0.,
  "LThumb3": 0.,

  "RFinger11": 0.,
  "RFinger12": 0.,
  "RFinger13": 0.,
  "RFinger21": 0.,
  "RFinger22": 0.,
  "RFinger23": 0.,
  "RFinger31": 0.,
  "RFinger32": 0.,
  "RFinger33": 0.,
  "RThumb1": 0.,
  "RThumb2": 0.,
  "RThumb3": 0.,
}


def readUrdf(robotName, rootBodyName, filteredJoints, mergedJoints,
             halfSittingByName):
  # read romeo urdf
  urdfPath = '%s/urdf/%s.urdf' % (path, robotName)
  urdf = open(urdfPath, 'r').read()
  mb, mbc, mbg, limits, visual_tf, collision_tf =\
    mc_rbdyn_urdf.rbdyn_from_urdf(urdf, fixed=False, baseLink='base_link')

  # sort half sitting by id
  rootBodyId = mbg.bodyIdByName(rootBodyName)
  halfSittingById = []
  for name, value in halfSittingByName.items():
    try:
      jId = mbg.jointIdByName(name)
      halfSittingById.append((jId, [value]))
    except Exception:
      pass

  # remove filtered joints and merge joints with theirs parents
  mbg.removeJoints(rootBodyId, filteredJoints)
  for mj in mergedJoints:
    mbg.mergeSubBodies(rootBodyId, mj, halfSittingById)

  # regenerate the MultiBody and the MultiBodyConfig
  mb = mbg.makeMultiBody(rootBodyId, False)
  mbc = rbd.MultiBodyConfig(mb)
  mbc.zero(mb)
  return mb, mbc, mbg, limits, visual_tf, collision_tf


def halfSittingPose(mb):
  """Return the half sitting pose in a dict jointId -> radian"""
  return {j.id(): [halfSitting[j.name()]] for j in mb.joints() if j.dof() == 1}


def nominalBounds(limits):
  """Return the romeo_left_arm bounds in qp standard form"""
  def minus(d):
    return {id: map(lambda x: -x, val) for id, val in d.iteritems()}
  return limits.lower, limits.upper, minus(limits.velocity), limits.velocity,\
      minus(limits.torque), limits.torque


def stdCollisionsFiles(mb):
  """Return the standard collision file name without the convex/stpbv suffix."""
  # {name: (bodyName, filename)}
  fileByBodyName = {b.name():(b.name(), b.name().replace('_link', '')) for b in mb.bodies()}

  def addBody(bodyName, file):
    fileByBodyName[bodyName] = (bodyName, file)

  # The mismatched names of the Body and convex files
  addBody('body', 'TrunkYaw')
  addBody('torso', 'Torso')

  addBody('LThigh', 'LHipPitch')
  addBody('RThigh', 'RHipPitch')

  addBody('LTibia', 'LKneePitch')
  addBody('RTibia', 'RKneePitch')

  addBody('l_ankle', 'LAnkleRoll')
  addBody('r_ankle', 'RAnkleRoll')

  addBody('l_wrist', 'LHand')
  addBody('r_wrist', 'RHand')

  addBody('LElbow', 'LElbowYaw')
  addBody('RElbow', 'RElbowYaw')


  #TODO: unsure, no daes for these? took the next closest instead
  addBody('HeadPitch_link', 'HeadRoll')
  addBody('NeckYaw_link', 'NeckPitch')

  addBody('LShoulder', 'LShoulderYaw')
  addBody('RShoulder', 'RShoulderYaw')

  addBody('LForeArm', 'LElbowYaw')
  addBody('RForeArm', 'RElbowYaw')

  addBody('LEyeYaw_link', 'LEye')
  addBody('REyeYaw_link', 'REye')

  addBody('LHipYaw_link', 'LHipPitch')
  addBody('RHipYaw_link', 'RHipPitch')

  addBody('LHip', 'LHipPitch')
  addBody('RHip', 'RHipPitch')

  addBody('LAnklePitch_link', 'LAnkleRoll')
  addBody('RAnklePitch_link', 'RAnkleRoll')

  #TODO: handle with merging the hand and fingers
  addBody('l_gripper', 'LThumb1')
  addBody('r_gripper', 'RThumb1')

  # loop over all the fingers
  for side in ['L','R']:
    for n in range(1,4):
      for m in range(1,4):
        num = str(n)+str(m)
        addBody((side+'Finger'+num+'_link'), (side+'Finger'+num))

  return fileByBodyName


def convexHull(files, mb):
  """
  Take a dict of the same format of stdCollisionsFiles or noGripperCollisionsFiles
  and return a bodyId -> file dict of convex files
  """
  convexPath = os.path.join(path, 'convex')

  convexFiles = {name: (bodyName, os.path.join(convexPath, filename + '-ch.txt'))
                 for name, (bodyName, filename) in files.iteritems()}
  return convexFiles


def stpbvHull(files, mb):
  """
  Take a dict of the same format of stdCollisionsFiles or noGripperCollisionsFiles
  and return a bodyId -> file dict of stpbv files
  """
  stpbvPath = os.path.join(path, 'stpbv')

  stpbvFiles = {name: (bodyName, os.path.join(stpbvPath, filename + '.txt'))
                 for name, (bodyName, filename) in files.iteritems()}

  return stpbvFiles

def fixedHandsCollisionsFiles(mb):
  """Return the fixed hand collision file name without the convex/stpbv suffix."""
  # {name: (bodyName, filename)}
  fileByBodyName = stdCollisionsFiles(mb)
  def addBody(bodyName, file):
    fileByBodyName[bodyName] = (bodyName, file)
  return fileByBodyName


