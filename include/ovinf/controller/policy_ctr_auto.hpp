#ifndef POLICY_CTR_AUTO_HPP
#define POLICY_CTR_AUTO_HPP

#include <chrono>

#include "filter/filter_mean.hpp"
#include "policy_controller_base.hpp"
#include "utils/history_buffer.hpp"

namespace ovinf {

class PolicyCtrAuto : public PolicyControllerBase {
 public:
  using Ptr = std::shared_ptr<PolicyCtrAuto>;

  PolicyCtrAuto() = delete;
  ~PolicyCtrAuto() = default;

  PolicyCtrAuto(RobotBase<float>::RobotPtr robot, YAML::Node const& config)
      : PolicyControllerBase(robot, config) {
    action_size_ = config["inference"]["action_size"].as<size_t>();

    for (size_t i = 0; i < action_size_; i++) {
      std::string joint_name =
          config["inference"]["policy_joint_names"][i].as<std::string>();
      size_t joint_idx = robot_->joint_names_.at(joint_name);
      policy_joint_idx_map_[i] = joint_idx;
    }

    if (config["num_delay"].IsDefined()) {
      num_delay_ = config["num_delay"].as<size_t>();
    } else {
      num_delay_ = 0;
    }
    delay_buffer_ = std::make_shared<HistoryBuffer<float>>(robot_->joint_size_,
                                                           num_delay_ + 1);
  }

  virtual void WarmUp() final {
    if (counter_++ % decimation_ == 0) {
      VectorT pos_input = VectorT::Zero(action_size_);
      VectorT vel_input = VectorT::Zero(action_size_);
      for (size_t i = 0; i < action_size_; i++) {
        pos_input[i] =
            robot_->Observer()->JointActualPosition()[policy_joint_idx_map_[i]];
        vel_input[i] =
            robot_->Observer()->JointActualVelocity()[policy_joint_idx_map_[i]];
      }

      inference_net_->WarmUp({.command = command_,
                              .ang_vel = robot_->Observer()->AngularVelocity(),
                              .proj_gravity = robot_->Observer()->ProjGravity(),
                              .joint_pos = pos_input,
                              .joint_vel = vel_input,
                              .euler_angles = robot_->Observer()->EulerRpy(),
                              .scan = robot_->Observer()->Depth()});
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
      VectorT pos_input = VectorT::Zero(action_size_);
      VectorT vel_input = VectorT::Zero(action_size_);
      for (size_t i = 0; i < action_size_; i++) {
        pos_input[i] =
            robot_->Observer()->JointActualPosition()[policy_joint_idx_map_[i]];
        vel_input[i] =
            robot_->Observer()->JointActualVelocity()[policy_joint_idx_map_[i]];
      }

      auto err = inference_net_->InferUnsync(
          {.command = command_,
           .ang_vel = robot_->Observer()->AngularVelocity(),
           .proj_gravity = robot_->Observer()->ProjGravity(),
           .joint_pos = pos_input,
           .joint_vel = vel_input,
           .euler_angles = robot_->Observer()->EulerRpy(),
           .scan = robot_->Observer()->Depth()});
    }

    if (set_target) {
      auto target_pos = inference_net_->GetResult();
      if (target_pos.has_value()) {
        policy_target_position_ = default_position_;
        for (size_t i = 0; i < action_size_; i++) {
          policy_target_position_[policy_joint_idx_map_[i]] =
              target_pos.value()[i];
        }
      } else {
        // std::cout << "target pos is empty" << std::endl;
      }

      delay_buffer_->AddObservation(
          target_pos_filter_->Filter(policy_target_position_));
      robot_->Executor()->JointTargetPosition() =
          delay_buffer_->GetObservation(num_delay_);

      ComputeJointPd();

      robot_->Executor()->JointTargetPosition() =
          delay_buffer_->GetObservation(0);
    }
  }

  virtual void Stop() final {
    command_ = VectorT::Zero(3);
    ready_ = false;
  }

 private:
  size_t action_size_ = 0;
  std::map<size_t, size_t> policy_joint_idx_map_;

  // Delay test
  std::shared_ptr<HistoryBuffer<float>> delay_buffer_ = nullptr;
  size_t num_delay_ = 0;
};

}  // namespace ovinf

#endif  // !POLICY_CTR_AUTO_HPP
