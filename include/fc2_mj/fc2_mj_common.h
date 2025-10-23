#ifndef HHFC_MJ_COMMON_HPP
#define HHFC_MJ_COMMON_HPP

#include "bitbot_mujoco/device/mujoco_imu.h"
#include "bitbot_mujoco/device/mujoco_joint.h"
#include "bitbot_mujoco/kernel/mujoco_kernel.hpp"
#include "utils/parallel_ankle.hpp"

namespace ovinf {

enum MotorIdx {
  LHipYawMotor = 0,
  LHipRollMotor = 1,
  LHipPitchMotor = 2,
  LKneeMotor = 3,
  LAnklePitchMotor = 4,
  LAnkleRollMotor = 5,

  RHipYawMotor = 6,
  RHipRollMotor = 7,
  RHipPitchMotor = 8,
  RKneeMotor = 9,
  RAnklePitchMotor = 10,
  RAnkleRollMotor = 11,

  LShoulderPitchMotor = 12,
  LShoulderRollMotor = 13,
  LShoulderYawMotor = 14,
  LElbowPitchMotor = 15,

  RShoulderPitchMotor = 16,
  RShoulderRollMotor = 17,
  RShoulderYawMotor = 18,
  RElbowPitchMotor = 19,
};

enum JointIdx {
  LHipYawJoint = 0,
  LHipRollJoint = 1,
  LHipPitchJoint = 2,
  LKneeJoint = 3,
  LAnklePitchJoint = 4,
  LAnkleRollJoint = 5,

  RHipYawJoint = 6,
  RHipRollJoint = 7,
  RHipPitchJoint = 8,
  RKneeJoint = 9,
  RAnklePitchJoint = 10,
  RAnkleRollJoint = 11,

  LShoulderPitchJoint = 12,
  LShoulderRollJoint = 13,
  LShoulderYawJoint = 14,
  LElbowPitchJoint = 15,

  RShoulderPitchJoint = 16,
  RShoulderRollJoint = 17,
  RShoulderYawJoint = 18,
  RElbowPitchJoint = 19,
};

}  // namespace ovinf

using KernelBus = bitbot::MujocoBus;
using ImuDevice = bitbot::MujocoImu;
using ImuPtr = ImuDevice*;
using MotorDevice = bitbot::MujocoJoint;
using MotorPtr = MotorDevice*;
using AnklePtr = std::shared_ptr<ovinf::ParallelAnkle<float>>;

struct UserData {};

using Kernel = bitbot::MujocoKernel<UserData>;

#endif  // !HHFC_MJ_COMMON_HPP
