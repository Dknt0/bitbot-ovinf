#ifndef OVINF_POLICY_CONTROLLER_FACTORY_HPP
#define OVINF_POLICY_CONTROLLER_FACTORY_HPP

#include "policy_controller_base.hpp"
#include "policy_ctl_leg_only.hpp"
#include "policy_ctr_21dof.hpp"
#include "policy_ctr_leg_waist.hpp"

namespace ovinf {

class PolicyControllerFactory {
 public:
  template <typename T = float>
  static std::shared_ptr<ovinf::PolicyControllerBase> CreatePolicyController(
      RobotBase<T>::RobotPtr robot, YAML::Node const &config) {
    std::string controller_type = config["controller_type"].as<std::string>();
    if (controller_type == "LEG_ONLY") {
      return std::make_shared<ovinf::PolicyCtrLegOnly>(robot, config);
    } else if (controller_type == "LEG_WAIST") {
      return std::make_shared<ovinf::PolicyCtrLegWaist>(robot, config);
    } else if (controller_type == "21DOF") {
      return std::make_shared<ovinf::PolicyCtr21DoF>(robot, config);
    } else {
      throw std::invalid_argument("Unknown controller type: " +
                                  controller_type);
    }
  }
};

}  // namespace ovinf

#endif  // !OVINF_POLICY_CONTROLLER_FACTORY_HPP
