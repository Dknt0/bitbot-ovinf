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

#ifdef BITBOT_DEPLOY
#include "robot/hhfc_cifx/hhfc_cifx_common.h"
#include "robot/hhfc_cifx/robot_hhfc_cifx.hpp"
using RobotT = ovinf::RobotHhfcCifx;
#else
#include "robot/hhfc_mj/hhfc_mj_common.h"
#include "robot/hhfc_mj/robot_hhfc_mj.hpp"
using RobotT = ovinf::RobotHhfcMj;
#endif  // BITBOT_DEPLOY

enum Events {
  InitPose = 1001,
  RunPolicy,
  EnableStandingPolicy,
  EnableWarkingPolicy,
  EnableRobustPolicy,
  PolicySwitch,

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
  PolicyRunning,
  PolicySwitching,
};

class MakeBitbotEverywhere {
 public:
  MakeBitbotEverywhere(std::string const &kernel_config,
                       std::string const &controller_config)
      : kernel_(kernel_config) {
    logger_ = bitbot::Logger().ConsoleLogger();
    YAML::Node config = YAML::LoadFile(controller_config);

    switching_time_ = config["RobotConfig"]["switching_time"].as<double>();

    // robot
    robot_ = std::make_shared<RobotT>(config["RobotConfig"]);

    // init controller
    init_pos_controller_ = std::make_shared<ovinf::InitPosController>(
        robot_, config["RobotConfig"]["init_pos"]);

    // Policy net
    standing_controller_ = std::make_shared<ovinf::PolicyController>(
        robot_, config["RobotConfig"]["policy_standing"]);
    walking_controller_ = std::make_shared<ovinf::PolicyController>(
        robot_, config["RobotConfig"]["policy_walking"]);
    robust_controller_ = std::make_shared<ovinf::PolicyController>(
        robot_, config["RobotConfig"]["policy_robust"]);
    current_policy_controller_ = standing_controller_;
    target_policy_controller_ = standing_controller_;

    command_.setZero();
  }

  void WillMake() {
    // Config
    kernel_.RegisterConfigFunc(
        [this](const KernelBus &bus, UserData &) { robot_->GetDevice(bus); });

    // Event
    kernel_.RegisterEvent(
        "init_pose", static_cast<bitbot::EventId>(Events::InitPose),
        [this](bitbot::EventValue, UserData &) {
          init_pos_controller_->Init();
          return static_cast<bitbot::StateId>(States::InitPose);
        });

    kernel_.RegisterEvent(
        "run_policy", static_cast<bitbot::EventId>(Events::RunPolicy),
        [this](bitbot::EventValue, UserData &) {
          current_policy_controller_ = standing_controller_;
          current_policy_controller_->Init();
          return static_cast<bitbot::StateId>(States::PolicyRunning);
        });

    kernel_.RegisterEvent(
        "enable_standing_policy",
        static_cast<bitbot::EventId>(Events::EnableStandingPolicy),
        [this](bitbot::EventValue, UserData &) {
          if (current_policy_controller_ == standing_controller_) {
            logger_->warn("Standing policy is already enabled");
            return static_cast<bitbot::StateId>(States::PolicyRunning);
          } else {
            logger_->info("Enabling standing policy");
            target_policy_controller_ = standing_controller_;
            target_policy_controller_->Init();
            return static_cast<bitbot::StateId>(States::PolicySwitching);
          }
        });

    kernel_.RegisterEvent(
        "enable_warking_policy",
        static_cast<bitbot::EventId>(Events::EnableWarkingPolicy),
        [this](bitbot::EventValue, UserData &) {
          if (current_policy_controller_ == walking_controller_) {
            logger_->warn("Walking policy is already enabled");
            return static_cast<bitbot::StateId>(States::PolicyRunning);
          } else {
            logger_->info("Enabling walking policy");
            target_policy_controller_ = walking_controller_;
            target_policy_controller_->Init();
            return static_cast<bitbot::StateId>(States::PolicySwitching);
          }
        });

    kernel_.RegisterEvent(
        "enable_robust_policy",
        static_cast<bitbot::EventId>(Events::EnableRobustPolicy),
        [this](bitbot::EventValue, UserData &) {
          if (current_policy_controller_ == robust_controller_) {
            logger_->warn("Robust policy is already enabled");
            return static_cast<bitbot::StateId>(States::PolicyRunning);
          } else {
            logger_->info("Enabling robust policy");
            target_policy_controller_ = robust_controller_;
            target_policy_controller_->Init();
            return static_cast<bitbot::StateId>(States::PolicySwitching);
          }
        });

    kernel_.RegisterEvent(
        "policy_switch", static_cast<bitbot::EventId>(Events::PolicySwitch),
        [this](bitbot::EventValue, UserData &) {
          current_policy_controller_->Stop();
          current_policy_controller_ = target_policy_controller_;
          return static_cast<bitbot::StateId>(States::PolicyRunning);
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

          robot_->Observer()->Update();
        },
        {Events::InitPose});

    kernel_.RegisterState(
        "init_pose", static_cast<bitbot::StateId>(States::InitPose),
        [this](const bitbot::KernelInterface &kernel,
               Kernel::ExtraData &extra_data, UserData &user_data) {
          robot_->Observer()->Update();
          init_pos_controller_->Step();
          target_policy_controller_->WarmUp();
          robot_->Executor()->ExecuteJointTorque();
        },
        {Events::RunPolicy});

    kernel_.RegisterState(
        "policy_running", static_cast<bitbot::StateId>(States::PolicyRunning),
        [this](const bitbot::KernelInterface &kernel,
               Kernel::ExtraData &extra_data, UserData &user_data) {
          robot_->Observer()->Update();
          current_policy_controller_->GetCommand() = command_;
          current_policy_controller_->Step();
          robot_->Executor()->ExecuteJointTorque();
        },
        {Events::VeloxDecrease, Events::VeloxIncrease, Events::VeloyDecrease,
         Events::VeloyIncrease, Events::VelowIncrease, Events::VelowDecrease,
         Events::SetVelX, Events::SetVelY, Events::SetVelW,
         Events::EnableStandingPolicy, Events::EnableWarkingPolicy,
         Events::EnableRobustPolicy});

    kernel_.RegisterState(
        "policy_switching",
        static_cast<bitbot::StateId>(States::PolicySwitching),
        [this](const bitbot::KernelInterface &kernel,
               Kernel::ExtraData &extra_data, UserData &user_data) {
          if (!switching_flag_) {
            switching_flag_ = true;
            switching_start_time_ = std::chrono::steady_clock::now();
          }
          double current_switching_time =
              std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                            switching_start_time_)
                  .count();
          if (current_switching_time < switching_time_) {
            robot_->Observer()->Update();
            current_policy_controller_->GetCommand() = command_;
            current_policy_controller_->Step();
            target_policy_controller_->WarmUp();
            robot_->Executor()->ExecuteJointTorque();
          } else {
            switching_flag_ = false;
            kernel.EmitEvent(Events::PolicySwitch, 0);
          }
        },
        {Events::PolicySwitch});

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

  ovinf::PolicyController::Ptr standing_controller_ = nullptr;
  ovinf::PolicyController::Ptr walking_controller_ = nullptr;
  ovinf::PolicyController::Ptr robust_controller_ = nullptr;
  ovinf::PolicyController::Ptr current_policy_controller_ = nullptr;
  ovinf::PolicyController::Ptr target_policy_controller_ = nullptr;

  bool switching_flag_ = false;
  double switching_time_;
  std::chrono::steady_clock::time_point switching_start_time_;

  Eigen::Vector3f command_;
};

#endif  // !USER_FUNC_H
