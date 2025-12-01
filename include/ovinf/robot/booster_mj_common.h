#ifndef BOOSTER_MJ_COMMON_HPP
#define BOOSTER_MJ_COMMON_HPP

#include "bitbot_mujoco/device/mujoco_imu.h"
#include "bitbot_mujoco/device/mujoco_joint.h"
#include "bitbot_mujoco/kernel/mujoco_kernel.hpp"
#include "utils/parallel_ankle.hpp"

namespace ovinf {

enum class MotorIdx {
  HeadYawMotor = 0,
  HeadPitchMotor = 1,

  LShoulderPitchMotor = 2,
  LShoulderRollMotor = 3,
  LElbowPitchMotor = 4,
  LElbowYawMotor = 5,

  RShoulderPitchMotor = 6,
  RShoulderRollMotor = 7,
  RElbowPitchMotor = 8,
  RElbowYawMotor = 9,

  WaistMotor = 10,

  LHipPitchMotor = 11,
  LHipRollMotor = 12,
  LHipYawMotor = 13,
  LKneePitchMotor = 14,
  LAnkleLongMotor = 15,
  LAnkleShortMotor = 16,

  RHipPitchMotor = 17,
  RHipRollMotor = 18,
  RHipYawMotor = 19,
  RKneePitchMotor = 20,
  RAnkleLongMotor = 21,
  RAnkleShortMotor = 22,
};

enum JointIdx {
  HeadYawJoint = 0,
  HeadPitchJoint = 1,

  LShoulderPitchJoint = 2,
  LShoulderRollJoint = 3,
  LElbowPitchJoint = 4,
  LElbowYawJoint = 5,

  RShoulderPitchJoint = 6,
  RShoulderRollJoint = 7,
  RElbowPitchJoint = 8,
  RElbowYawJoint = 9,

  WaistJoint = 10,

  LHipPitchJoint = 11,
  LHipRollJoint = 12,
  LHipYawJoint = 13,
  LKneePitchJoint = 14,
  LAnklePitchJoint = 15,
  LAnkleRollJoint = 16,

  RHipPitchJoint = 17,
  RHipRollJoint = 18,
  RHipYawJoint = 19,
  RKneePitchJoint = 20,
  RAnklePitchJoint = 21,
  RAnkleRollJoint = 22,
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

#endif  // !BOOSTER_MJ_COMMON_HPP
