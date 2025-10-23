#ifndef FC2_CIFX_COMMON_HPP
#define FC2_CIFX_COMMON_HPP

#include "bitbot_cifx/device/imu_mti300.h"
#include "bitbot_cifx/device/joint_elmo.h"
#include "bitbot_cifx/kernel/cifx_kernel.hpp"

namespace ovinf {

enum LeftRight {
  LEFT = 0,
  RIGHT = 1,
};

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

using KernelBus = bitbot::CifxBus;
using ImuDevice = bitbot::ImuMti300;
using ImuPtr = ImuDevice*;
using MotorDevice = bitbot::JointElmo;
using MotorPtr = MotorDevice*;

struct UserData {};

using Kernel = bitbot::CifxKernel<UserData>;

#endif  // !FC2_CIFX_COMMON_HPP
