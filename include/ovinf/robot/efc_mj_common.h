#ifndef EFC_MJ_COMMON_HPP
#define EFC_MJ_COMMON_HPP

#include "bitbot_mujoco/device/mujoco_imu.h"
#include "bitbot_mujoco/device/mujoco_joint.h"
#include "bitbot_mujoco/kernel/mujoco_kernel.hpp"
#include "utils/parallel_ankle.hpp"

namespace ovinf {

enum class MotorIdx {
  LHipPitchMotor = 0,
  LHipRollMotor = 1,
  LHipYawMotor = 2,
  LKneePitchMotor = 3,
  LAnklePitchMotor = 4,
  LAnkleRollMotor = 5,

  RHipPitchMotor = 6,
  RHipRollMotor = 7,
  RHipYawMotor = 8,
  RKneePitchMotor = 9,
  RAnklePitchMotor = 10,
  RAnkleRollMotor = 11,

  WaistMotor = 12,

  LShoulderPitchMotor = 13,
  LShoulderRollMotor = 14,
  LShoulderYawMotor = 15,
  LElbowPitchMotor = 16,
  LElbowYawMotor = 17,

  RShoulderPitchMotor = 18,
  RShoulderRollMotor = 19,
  RShoulderYawMotor = 21,
  RElbowPitchMotor = 21,
  RElbowYawMotor = 22,

  HeadYawMotor = 23,
};

enum JointIdx {
  LHipPitchJoint = 0,
  LHipRollJoint = 1,
  LHipYawJoint = 2,
  LKneePitchJoint = 3,
  LAnklePitchJoint = 4,
  LAnkleRollJoint = 5,

  RHipPitchJoint = 6,
  RHipRollJoint = 7,
  RHipYawJoint = 8,
  RKneePitchJoint = 9,
  RAnklePitchJoint = 10,
  RAnkleRollJoint = 11,

  WaistJoint = 12,

  LShoulderPitchJoint = 13,
  LShoulderRollJoint = 14,
  LShoulderYawJoint = 15,
  LElbowPitchJoint = 16,
  LElbowYawJoint = 17,

  RShoulderPitchJoint = 18,
  RShoulderRollJoint = 19,
  RShoulderYawJoint = 21,
  RElbowPitchJoint = 21,
  RElbowYawJoint = 22,

  HeadYawJoint = 23,
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

#endif  // !EFC_MJ_COMMON_HPP
