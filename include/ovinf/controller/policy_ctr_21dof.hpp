#ifndef POLICY_CTR_21DOF_HPP
#define POLICY_CTR_21DOF_HPP

#include <chrono>

#include "filter/filter_mean.hpp"
#include "policy_controller_base.hpp"

namespace ovinf {

class PolicyCtr21DoF : public PolicyControllerBase {
 public:
  using Ptr = std::shared_ptr<PolicyCtr21DoF>;

  PolicyCtr21DoF() = delete;
  ~PolicyCtr21DoF() = default;

  PolicyCtr21DoF(RobotBase<float>::RobotPtr robot, YAML::Node const& config)
      : PolicyControllerBase(robot, config) {}

  virtual void WarmUp() final {
    if (counter_++ % decimation_ == 0) {
      VectorT pos_input = VectorT::Zero(21);
      VectorT vel_input = VectorT::Zero(21);
      for (size_t i = 0; i < 17; i++) {
        pos_input[i] = robot_->Observer()->JointActualPosition()[i];
        vel_input[i] = robot_->Observer()->JointActualVelocity()[i];
      }
      for (size_t i = 0; i < 4; i++) {
        pos_input[17 + i] = robot_->Observer()->JointActualPosition()[18 + i];
        vel_input[17 + i] = robot_->Observer()->JointActualVelocity()[18 + i];
      }

      inference_net_->WarmUp({.command = command_,
                              .ang_vel = robot_->Observer()->AngularVelocity(),
                              .proj_gravity = robot_->Observer()->ProjGravity(),
                              .joint_pos = pos_input,
                              .joint_vel = vel_input});
    }
    policy_target_position_ = robot_->Executor()->JointTargetPosition();
    // target_pos_filter_->Filter(policy_target_position_);
    // inference_net_->PrintInfo();
  }

  virtual void Init() final {
    counter_ = 0;
    command_ = VectorT::Zero(3);
    ready_ = true;
    target_pos_filter_->Reset();
    policy_target_position_ = robot_->Executor()->JointTargetPosition();
  }

  virtual void Step(bool set_target = true) final {
    if (!ready_) {
      std::cerr << "PolicyController not ready" << std::endl;
      return;
    }

    if (counter_++ % decimation_ == 0) {
      VectorT pos_input = VectorT::Zero(21);
      VectorT vel_input = VectorT::Zero(21);
      for (size_t i = 0; i < 17; i++) {
        pos_input[i] = robot_->Observer()->JointActualPosition()[i];
        vel_input[i] = robot_->Observer()->JointActualVelocity()[i];
      }
      for (size_t i = 0; i < 4; i++) {
        pos_input[17 + i] = robot_->Observer()->JointActualPosition()[18 + i];
        vel_input[17 + i] = robot_->Observer()->JointActualVelocity()[18 + i];
      }

      auto err = inference_net_->InferUnsync(
          {.command = command_,
           .ang_vel = robot_->Observer()->AngularVelocity(),
           .proj_gravity = robot_->Observer()->ProjGravity(),
           .joint_pos = pos_input,
           .joint_vel = vel_input});
    }

    if (set_target) {
      auto target_pos = inference_net_->GetResult();
      if (target_pos.has_value()) {
        policy_target_position_ = default_position_;
        for (size_t i = 0; i < 17; i++) {
          policy_target_position_[i] = target_pos.value()[i];
        }
        for (size_t i = 0; i < 4; i++) {
          policy_target_position_[18 + i] = target_pos.value()[17 + i];
        }
      } else {
        // std::cout << "target pos is empty" << std::endl;
      }

      robot_->Executor()->JointTargetPosition() =
          target_pos_filter_->Filter(policy_target_position_);
      ComputeJointPd();
    }
  }

  virtual void Stop() final {
    command_ = VectorT::Zero(3);
    ready_ = false;
  }
};

}  // namespace ovinf

#endif  // !POLICY_CTR_21DOF_HPP
