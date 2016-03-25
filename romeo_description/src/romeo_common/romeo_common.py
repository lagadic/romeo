import roslib; roslib.load_manifest('romeo_description')

import os

import numpy as np

from eigen3 import Vector3d, toEigen
import spacevecalg as sva
import rbdyn as rbd

import mc_rbdyn_urdf


path = roslib.packages.get_pkg_dir('romeo_description')


virtualJoints = [
  'l_wrist',
  'r_wrist',
  'l_gripper', # FIXME not sure if needed here
  'r_gripper', # FIXME
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
  'LThumb1',
  'LThumb2',
  'LThumb3',
  'LFinger11',
  'LFinger12',
  'LFinger13',
  'LFinger21',
  'LFinger22',
  'LFinger23',
  'LFinger31',
  'LFinger32',
  'LFinger33', # New:
  'RThumb1',
  'RThumb2',
  'RThumb3',
  'RFinger11',
  'RFinger12',
  'RFinger13',
  'RFinger21',
  'RFinger22',
  'RFinger23',
  'RFinger31',
  'RFinger32',
  'RFinger33'
]


forceSensors = [
]


rootBody = 'base_link'

accelBody = "base_link"


halfSitting = { #TODO
  "NeckPitch": 0., 
  "HeadPitch": 0.,
  "HeadRoll": 0., 
  "LHipYaw": 0., 
  "LHipRoll": 0., 
  "LHipPitch": -0.0929, 
  "LKneePitch": 0.2565, 
  "LAnklePitch": -0.163522, 
  "LAnkleRoll": 0., 
  "RHipYaw": 0., 
  "RHipRoll": 0., 
  "RHipPitch": -0.0929,  
  "RKneePitch": 0.2565, 
  "RAnklePitch":-0.163522, 
  "RAnkleRoll": 0., 
  "TrunkYaw": 0., 
  "LShoulderPitch": 1.75, 
  "LShoulderYaw": 0., 
  "LElbowRoll": -1.30, 
  "LElbowYaw": -0.35, 
  "LWristRoll": 0., 
  "LWristYaw": 0., 
  "LWristPitch": 0., 
  "LHand": 0., 
  "RShoulderPitch": 1.75, 
  "RShoulderYaw": 0., 
  "RElbowRoll": 1.30, 
  "RElbowYaw": 0.17, 
  "RWristRoll": 0.35, 
  "RWristYaw": 0., 
  "RWristPitch": 0., 
  "RHand": 0., 
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
  "LHand": 1.,
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
  "RHand": 1.
}


def readUrdf(robotName, rootBodyName,  filteredJoints, mergedJoints,
             halfSittingByName):
  # read romeo urdf
  urdfPath = '%s/urdf/%s.urdf' % (path, robotName)
  urdf = open(urdfPath, 'r').read()
  mb, mbc, mbg, limits, visual_tf, collision_tf =\
    mc_rbdyn_urdf.rbdyn_from_urdf(urdf, fixed=True)

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
  mb = mbg.makeMultiBody(rootBodyId, True)
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
  fileByBodyName = {b.name():(b.name(), b.name().replace('_LINK', '')) for b in mb.bodies()}

  def addBody(bodyName, file):
    fileByBodyName[bodyName] = (bodyName, file)

  # addBody('LThumb1_link', 'LThumb1') #FIXME
  # addBody('LThumb2_link', 'LThumb2')
  # addBody('LThumb3_link', 'LThumb3')

  return fileByBodyName


def convexHull(files, mb):
  """
  Take a dict of the same format of stdCollisionsFiles or noGripperCollisionsFiles
  and return a bodyId -> file dict of convex files
  """
  convexPath = os.path.join(path, 'convex')

  convexFiles = {name: (bodyName, os.path.join(convexPath, filename + '-ch.txt'))
                 for name, (bodyName, filename) in files.iteritems()}
  print "convex", convexFiles
  print 'nr bodies', mb.nrBodies()
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


