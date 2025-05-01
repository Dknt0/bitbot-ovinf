#ifndef USER_FUNC_HPP
#define USER_FUNC_HPP

#include <float.h>
#include <math.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <memory>

#include "controller/init_pos.hpp"
#include "controller/policy_controller.hpp"
#include "controller/zero_pos.hpp"

#ifdef BITBOT_DEPLOY
#include "robot/hhfc_cifx/hhfc_cifx_common.h"
#include "robot/hhfc_cifx/robot_hhfc_cifx.hpp"
using RobotT = ovinf::RobotHhfcCifx;
#else
// #include "robot/fc2_mj/robot_fc2_mj.hpp"
// #include "robot/fc2_mj/fc2_mj_common.h"
// using RobotT = ovinf::RobotFc2Mj;
#include "robot/hhfc_mj/robot_hhfc_mj.hpp"
#include "robot/hhfc_mj/hhfc_mj_common.h"
using RobotT = ovinf::RobotHhfcMj;
#endif  // BITBOT_DEOPLOY

enum Events {
  InitPose = 1001,
  PolicyRun,

  VeloxIncrease = 2001,
  VeloxDecrease = 2002,
  VeloyIncrease = 2003,
  VeloyDecrease = 2004,
  VelowIncrease = 2005,
  VelowDecrease = 2006,

  SetVelX = 2014,
  SetVelY = 2015,
  SetVelW = 2016,
};

enum class States : bitbot::StateId {
  Waiting = 1001,

  InitPose,
  PolicyRun,
};

class MakeBitbotEverywhere {
 public:
  MakeBitbotEverywhere(std::string const &kernel_config,
                       std::string const &controller_config)
      : kernel_(kernel_config) {
    logger_ = bitbot::Logger().ConsoleLogger();
    YAML::Node config =
        YAML::LoadFile(controller_config);

    // robot
    robot_ = std::make_shared<RobotT>(config["RobotConfig"]);
    // robot_->PrintInfo();

    // init controller
    init_pos_controller_ = std::make_shared<ovinf::InitPosController>(
        robot_, config["RobotConfig"]["init_pos"]);

    // zero controller
    zero_pos_controller_ = std::make_shared<ovinf::ZeroPosController>(
        robot_, config["RobotConfig"]["zero_pos"]);

    // Policy net
    humanoid_controller_ = std::make_shared<ovinf::PolicyController>(
        robot_, config["RobotConfig"]["policy_humanoid"]);

    command_.setZero();
  }

  void WillMake() {
    // Config
    kernel_.RegisterConfigFunc([this](const KernelBus &bus, UserData &) {
      robot_->GetDevice(bus);
      humanoid_controller_->WarmUp();
    });

    // Event
    kernel_.RegisterEvent(
        "init_pose", static_cast<bitbot::EventId>(Events::InitPose),
        [this](bitbot::EventValue, UserData &) {
          init_pos_controller_->Init();
          return static_cast<bitbot::StateId>(States::InitPose);
        });

    kernel_.RegisterEvent(
        "policy_run", static_cast<bitbot::EventId>(Events::PolicyRun),
        [this](bitbot::EventValue, UserData &) {
          humanoid_controller_->Init();
          return static_cast<bitbot::StateId>(States::PolicyRun);
        });

    kernel_.RegisterEvent(
        "velo_x_increase", static_cast<bitbot::EventId>(Events::VeloxIncrease),
        [this](bitbot::EventValue key_state, UserData &) {
          if (key_state ==
              static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up)) {
            command_[0] += 0.05;
            logger_->info("current velocity: x={}", command_[0]);
          }
          return std::nullopt;
        });

    kernel_.RegisterEvent(
        "velo_x_decrease", static_cast<bitbot::EventId>(Events::VeloxDecrease),
        [this](bitbot::EventValue key_state, UserData &) {
          if (key_state ==
              static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up)) {
            command_[0] -= 0.05;
            logger_->info("current velocity: x={}", command_[0]);
          }
          return std::nullopt;
        });

    kernel_.RegisterEvent(
        "velo_y_increase", static_cast<bitbot::EventId>(Events::VeloyIncrease),
        [this](bitbot::EventValue key_state, UserData &) {
          if (key_state ==
              static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up)) {
            command_[1] += 0.05;
            logger_->info("current velocity: y={}", command_[1]);
          }
          return std::nullopt;
        });

    kernel_.RegisterEvent(
        "velo_y_decrease", static_cast<bitbot::EventId>(Events::VeloyDecrease),
        [this](bitbot::EventValue key_state, UserData &) {
          if (key_state ==
              static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up)) {
            command_[1] -= 0.05;
            logger_->info("current velocity: y={}", command_[1]);
          }
          return std::nullopt;
        });

    kernel_.RegisterEvent(
        "velo_w_increase", static_cast<bitbot::EventId>(Events::VelowIncrease),
        [this](bitbot::EventValue key_state, UserData &) {
          if (key_state ==
              static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up)) {
            command_[2] += 0.05;
            logger_->info("current velocity: w={}", command_[2]);
          }
          return std::nullopt;
        });

    kernel_.RegisterEvent(
        "velo_w_decrease", static_cast<bitbot::EventId>(Events::VelowDecrease),
        [this](bitbot::EventValue key_state, UserData &) {
          if (key_state ==
              static_cast<bitbot::EventValue>(bitbot::KeyboardEvent::Up)) {
            command_[2] -= 0.05;
            logger_->info("current velocity: w={}", command_[2]);
          }
          return std::nullopt;
        });

    kernel_.RegisterEvent(
        "set_vel_x", static_cast<bitbot::EventId>(Events::SetVelX),
        [this](bitbot::EventValue key_state, UserData &) {
          double value = *reinterpret_cast<double *>(&key_state);
          command_[0] = value;
          return std::nullopt;
        });

    kernel_.RegisterEvent(
        "set_vel_y", static_cast<bitbot::EventId>(Events::SetVelY),
        [this](bitbot::EventValue key_state, UserData &) {
          double value = *reinterpret_cast<double *>(&key_state);
          command_[1] = value;
          return std::nullopt;
        });

    kernel_.RegisterEvent(
        "set_vel_w", static_cast<bitbot::EventId>(Events::SetVelW),
        [this](bitbot::EventValue key_state, UserData &) {
          double value = *reinterpret_cast<double *>(&key_state);
          command_[2] = value;
          logger_->info("current velocity: x={} y={} w={}", command_[0],
                        command_[1], command_[2]);
          return std::nullopt;
        });

    // State
    kernel_.RegisterState(
        "waiting", static_cast<bitbot::StateId>(States::Waiting),
        [this](const bitbot::KernelInterface &kernel,
               Kernel::ExtraData &extra_data, UserData &user_data) {
          static bool first = true;
          if (first) {
            first = false;
            robot_->SetExtraData(extra_data);
          }

          robot_->observer_->Update();
        },
        {Events::InitPose});

    kernel_.RegisterState(
        "init_pose", static_cast<bitbot::StateId>(States::InitPose),
        [this](const bitbot::KernelInterface &kernel,
               Kernel::ExtraData &extra_data, UserData &user_data) {
          robot_->observer_->Update();
          init_pos_controller_->Step();
          robot_->executor_->ExecuteJointTorque();
        },
        {Events::PolicyRun});

    kernel_.RegisterState(
        "policy_run", static_cast<bitbot::StateId>(States::PolicyRun),
        [this](const bitbot::KernelInterface &kernel,
               Kernel::ExtraData &extra_data, UserData &user_data) {
          robot_->observer_->Update();
          humanoid_controller_->GetCommand() = command_;
          humanoid_controller_->Step();
          robot_->executor_->ExecuteJointTorque();
        },
        {Events::VeloxDecrease, Events::VeloxIncrease, Events::VeloyDecrease,
         Events::VeloyIncrease, Events::VelowIncrease, Events::VelowDecrease,
         Events::SetVelX, Events::SetVelY, Events::SetVelW});

    // First state
    kernel_.SetFirstState(static_cast<bitbot::StateId>(States::Waiting));
  }

  void BeMaking() { kernel_.Run(); }

  void HaveMade() {
    logger_->info("Make BITBOT great forever!!!!!!!!!!!!!!!!");
  }

 private:
  Kernel kernel_;
  bitbot::SpdLoggerSharedPtr logger_;
  RobotT::Ptr robot_;
  ovinf::InitPosController::Ptr init_pos_controller_;
  ovinf::ZeroPosController::Ptr zero_pos_controller_;
  ovinf::PolicyController::Ptr humanoid_controller_;

  Eigen::Vector3f command_;
};

#endif  // !USER_FUNC_H
