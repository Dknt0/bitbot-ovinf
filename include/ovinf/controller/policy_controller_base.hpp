#ifndef POLICY_CONTROLLER_BASE_HPP
#define POLICY_CONTROLLER_BASE_HPP

#include <chrono>

#include "controller/controller_base.hpp"
#include "filter/filter_factory.hpp"
#include "ovinf/ovinf_factory.hpp"

namespace ovinf {

class PolicyControllerBase : public ControllerBase<float> {
 public:
  using Ptr = std::shared_ptr<PolicyControllerBase>;

  PolicyControllerBase() = delete;
  ~PolicyControllerBase() = default;

  PolicyControllerBase(RobotBase<float>::RobotPtr robot,
                       YAML::Node const& config)
      : ControllerBase<float>(robot, config),
        decimation_(config["decimation"].as<int>()) {
    p_gains_ = VectorT::Zero(robot_->joint_size_);
    d_gains_ = VectorT::Zero(robot_->joint_size_);
    default_position_ = VectorT::Zero(robot_->joint_size_);

    target_pos_filter_ =
        FilterFactory::CreateFilter(config["target_pos_filter"]);

    for (auto const& pair : robot_->joint_names_) {
      p_gains_(pair.second) = config["p_gains"][pair.first].as<float>();
      d_gains_(pair.second) = config["d_gains"][pair.first].as<float>();
      default_position_(pair.second) =
          config["default_position"][pair.first].as<float>();
    }

    command_ = VectorT::Zero(3);
    counter_ = 0;

    inference_net_ = ovinf::PolicyFactory::CreatePolicy(config["inference"]);
  }

  VectorT& GetCommand() { return command_; }

 protected:
  const int decimation_;
  bool perception_enabled_ = false;

  VectorT command_;
  size_t counter_ = 0;
  VectorT default_position_;
  ovinf::BasePolicy<float>::BasePolicyPtr inference_net_;

  // Target filter
  FilterBase<VectorT>::Ptr target_pos_filter_;
  VectorT policy_target_position_;
};

}  // namespace ovinf

#endif  // !POLICY_CONTROLLER_BASE_HPP
