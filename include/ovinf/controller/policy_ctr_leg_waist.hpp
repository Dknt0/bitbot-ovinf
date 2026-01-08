#ifndef POLICY_CTR_LEG_WAIST_HPP
#define POLICY_CTR_LEG_WAIST_HPP

#include <chrono>

#include "filter/filter_mean.hpp"
#include "policy_controller_base.hpp"

namespace ovinf {

class PolicyCtrLegWaist : public PolicyControllerBase {
 public:
  using Ptr = std::shared_ptr<PolicyCtrLegWaist>;

  PolicyCtrLegWaist() = delete;
  ~PolicyCtrLegWaist() = default;

  PolicyCtrLegWaist(RobotBase<float>::RobotPtr robot, YAML::Node const& config)
      : PolicyControllerBase(robot, config) {}

  virtual void WarmUp() final {
    if (counter_++ % decimation_ == 0) {
      inference_net_->WarmUp(
          {.command = command_,
           .ang_vel = robot_->Observer()->AngularVelocity(),
           .proj_gravity = robot_->Observer()->ProjGravity(),
           .joint_pos =
               robot_->Observer()->JointActualPosition().segment(0, 13),
           .joint_vel =
               robot_->Observer()->JointActualVelocity().segment(0, 13)});
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
      auto err = inference_net_->InferUnsync(
          {.command = command_,
           .ang_vel = robot_->Observer()->AngularVelocity(),
           .proj_gravity = robot_->Observer()->ProjGravity(),
           .joint_pos =
               robot_->Observer()->JointActualPosition().segment(0, 13),
           .joint_vel =
               robot_->Observer()->JointActualVelocity().segment(0, 13)});
    }

    if (set_target) {
      auto target_pos = inference_net_->GetResult();
      if (target_pos.has_value()) {
        for (size_t i = 0; i < 13; i++) {
          policy_target_position_[i] = target_pos.value()[i];
        }
        for (size_t i = 13; i < 24; i++) {
          policy_target_position_[i] = default_position_[i];
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

#endif  // !POLICY_CTR_LEG_WAIST_HPP
