#ifndef ROBOT_HHFC_MJ_HPP
#define ROBOT_HHFC_MJ_HPP

#include <filesystem>
#include <vector>

#include "fc2_mj_common.h"
#include "filter/filter_factory.hpp"
#include "robot/base/robot_base.hpp"
#include "utils/csv_logger.hpp"

namespace ovinf {

class RobotFc2Mj : public RobotBase<float> {
  using VectorT = Eigen::Matrix<float, Eigen::Dynamic, 1>;

 public:
  using Ptr = std::shared_ptr<RobotFc2Mj>;

 private:
  class ObserverFc2Mj : public ObserverBase {
   public:
    ObserverFc2Mj() = delete;
    ObserverFc2Mj(RobotBase<float>* robot, const YAML::Node& config)
        : ObserverBase(robot, config) {
      // Create Filter
      motor_pos_filter_ =
          FilterFactory::CreateFilter(config["motor_pos_filter"]);
      motor_vel_filter_ =
          FilterFactory::CreateFilter(config["motor_vel_filter"]);
      ang_vel_filter_ = FilterFactory::CreateFilter(config["ang_vel_filter"]);
      acc_filter_ = FilterFactory::CreateFilter(config["acc_filter"]);
      eluer_rpy_filter_ = FilterFactory::CreateFilter(config["euler_filter"]);

      // Create Logger
      log_flag_ = config["log_data"].as<bool>();
      if (log_flag_) {
        CreateLog(config);
      }
    }

    virtual bool Update() final {
      auto robot_mj = dynamic_cast<RobotFc2Mj*>(robot_);

      for (size_t i = 0; i < motor_size_; ++i) {
        motor_actual_position_[i] = robot_mj->motors_[i]->GetActualPosition() *
                                    robot_->motor_direction_(i, 0);
        motor_actual_velocity_[i] = robot_mj->motors_[i]->GetActualVelocity() *
                                    robot_->motor_direction_(i, 0);
      }

      motor_actual_position_ =
          motor_pos_filter_->Filter(motor_actual_position_);
      motor_actual_velocity_ =
          motor_vel_filter_->Filter(motor_actual_velocity_);

      joint_actual_position_ = motor_actual_position_;
      joint_actual_velocity_ = motor_actual_velocity_;

      euler_rpy_ = eluer_rpy_filter_->Filter(
          Eigen::Vector3f(robot_mj->imu_->GetRoll(), robot_mj->imu_->GetPitch(),
                          robot_mj->imu_->GetYaw()));

      acceleration_ = acc_filter_->Filter(
          Eigen::Vector3f(robot_mj->imu_->GetAccX(), robot_mj->imu_->GetAccY(),
                          robot_mj->imu_->GetAccZ()));
      angular_velocity_ = ang_vel_filter_->Filter(Eigen::Vector3f(
          robot_mj->imu_->GetGyroX(), robot_mj->imu_->GetGyroY(),
          robot_mj->imu_->GetGyroZ()));

      Eigen::Matrix3f Rwb(
          Eigen::AngleAxisf(euler_rpy_[2], Eigen::Vector3f::UnitZ()) *
          Eigen::AngleAxisf(euler_rpy_[1], Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(euler_rpy_[0], Eigen::Vector3f::UnitX()));
      proj_gravity_ =
          VectorT(Rwb.transpose() * Eigen::Vector3f{0.0, 0.0, -1.0});

      if (log_flag_) {
        WriteLog();
      }
      return true;
    }

   private:
    inline void CreateLog(YAML::Node const& config);
    inline void WriteLog();

   private:
    FilterBase<VectorT>::Ptr motor_pos_filter_;
    FilterBase<VectorT>::Ptr motor_vel_filter_;
    FilterBase<VectorT>::Ptr ang_vel_filter_;
    FilterBase<VectorT>::Ptr acc_filter_;
    FilterBase<VectorT>::Ptr eluer_rpy_filter_;

    bool log_flag_ = false;
    CsvLogger::Ptr csv_logger_;
  };

  class ExecutorFc2Mj : public ExecutorBase {
   public:
    ExecutorFc2Mj() = delete;
    ExecutorFc2Mj(RobotBase<float>* robot, const YAML::Node& config)
        : ExecutorBase(robot, config) {}

    virtual bool ExecuteJointTorque() final {
      motor_target_position_ = joint_target_position_;
      motor_target_torque_ = joint_target_torque_;

      ExecuteMotorTorque();
      return true;
    }

    virtual bool ExecuteMotorTorque() final {
      auto robot_mj = dynamic_cast<RobotFc2Mj*>(robot_);
      for (size_t i = 0; i < motor_size_; ++i) {
        // Torque limit
        if (motor_target_torque_[i] > torque_limit_[i]) {
          motor_target_torque_[i] = torque_limit_[i];
        } else if (motor_target_torque_[i] < -torque_limit_[i]) {
          motor_target_torque_[i] = -torque_limit_[i];
        }

        // Position limit
        if (robot_->Observer()->MotorActualPosition()[i] >
            motor_upper_limit_[i]) {
          motor_target_torque_[i] = 0.0;
        } else if (robot_->Observer()->MotorActualPosition()[i] <
                   motor_lower_limit_[i]) {
          motor_target_torque_[i] = 0.0;
        }
      }

      // Set target
      for (size_t i = 0; i < motor_size_; ++i) {
        robot_mj->motors_[i]->SetTargetTorque(motor_target_torque_[i] *
                                              robot_->motor_direction_(i, 0));
        robot_mj->motors_[i]->SetTargetPosition(motor_target_position_[i] *
                                                robot_->motor_direction_(i, 0));
      }
      return true;
    }

    virtual bool ExecuteMotorCurrent() final {
      throw std::runtime_error(
          "ExecuteMotorCurrent is not supported in mujoco");
      return false;
    }

   private:
  };

 public:
  RobotFc2Mj() = delete;
  RobotFc2Mj(const YAML::Node& config) : RobotBase(config) {
    motors_.resize(motor_size_);
    this->observer_ = std::make_shared<ObserverFc2Mj>((RobotBase<float>*)this,
                                                      config["observer"]);
    this->executor_ = std::make_shared<ExecutorFc2Mj>((RobotBase<float>*)this,
                                                      config["executor"]);
  }

  inline void GetDevice(const KernelBus& bus);

  void SetExtraData(Kernel::ExtraData& extra_data) {
    extra_data_ = &extra_data;
  }

  virtual void PrintInfo() final {
    for (auto const& pair : motor_names_) {
      std::cout << "Motor id: " << pair.second << ", name: " << pair.first
                << std::endl;
      std::cout << "  - direction: " << motor_direction_(pair.second, 0)
                << std::endl;
      std::cout << "  - upper limit: "
                << Executor()->MotorUpperLimit()(pair.second, 0) << std::endl;
      std::cout << "  - lower limit: "
                << Executor()->MotorLowerLimit()(pair.second, 0) << std::endl;
      std::cout << "  - torque limit: "
                << Executor()->TorqueLimit()(pair.second, 0) << std::endl;
    }
    for (auto const& pair : joint_names_) {
      std::cout << "Joint id: " << pair.second << " name: " << pair.first
                << std::endl;
    }
  }

 private:
  std::vector<MotorPtr> motors_ = {};
  ImuPtr imu_;
  Kernel::ExtraData* extra_data_;
  // std::vector<AnklePtr> ankles_;
};

void RobotFc2Mj::GetDevice(const KernelBus& bus) {
  motors_[LHipYawMotor] = bus.GetDevice<MotorDevice>(0).value();
  motors_[LHipRollMotor] = bus.GetDevice<MotorDevice>(1).value();
  motors_[LHipPitchMotor] = bus.GetDevice<MotorDevice>(2).value();
  motors_[LKneeMotor] = bus.GetDevice<MotorDevice>(3).value();
  motors_[LAnklePitchMotor] = bus.GetDevice<MotorDevice>(4).value();
  motors_[LAnkleRollMotor] = bus.GetDevice<MotorDevice>(5).value();

  motors_[RHipYawMotor] = bus.GetDevice<MotorDevice>(6).value();
  motors_[RHipRollMotor] = bus.GetDevice<MotorDevice>(7).value();
  motors_[RHipPitchMotor] = bus.GetDevice<MotorDevice>(8).value();
  motors_[RKneeMotor] = bus.GetDevice<MotorDevice>(9).value();
  motors_[RAnklePitchMotor] = bus.GetDevice<MotorDevice>(10).value();
  motors_[RAnkleRollMotor] = bus.GetDevice<MotorDevice>(11).value();

  motors_[LShoulderPitchMotor] = bus.GetDevice<MotorDevice>(12).value();
  motors_[LShoulderRollMotor] = bus.GetDevice<MotorDevice>(13).value();
  motors_[LShoulderYawMotor] = bus.GetDevice<MotorDevice>(14).value();
  motors_[LElbowPitchMotor] = bus.GetDevice<MotorDevice>(15).value();

  motors_[RShoulderPitchMotor] = bus.GetDevice<MotorDevice>(16).value();
  motors_[RShoulderRollMotor] = bus.GetDevice<MotorDevice>(17).value();
  motors_[RShoulderYawMotor] = bus.GetDevice<MotorDevice>(18).value();
  motors_[RElbowPitchMotor] = bus.GetDevice<MotorDevice>(19).value();

  imu_ = bus.GetDevice<ImuDevice>(20).value();
}

void RobotFc2Mj::ObserverFc2Mj::CreateLog(YAML::Node const& config) {
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm* now_tm = std::localtime(&now_time);
  std::stringstream ss;
  ss << std::put_time(now_tm, "%Y-%m-%d-%H-%M-%S");
  std::string current_time = ss.str();

  std::string log_dir = config["log_dir"].as<std::string>();
  std::filesystem::path config_file_path(log_dir);
  if (config_file_path.is_relative()) {
    config_file_path = canonical(config_file_path);
  }

  std::string logger_file =
      config_file_path.string() + "/" + current_time + "_extra.csv";

  if (!exists(config_file_path)) {
    create_directories(config_file_path);
  }

  // Get headers
  std::vector<std::string> headers;

  // Motor actual pos
  for (size_t i = 0; i < motor_size_; ++i) {
    headers.push_back("motor_actual_pos_" + std::to_string(i));
  }

  // Motor actual vel
  for (size_t i = 0; i < motor_size_; ++i) {
    headers.push_back("motor_actual_vel_" + std::to_string(i));
  }

  // Joint actual pos
  for (size_t i = 0; i < joint_size_; ++i) {
    headers.push_back("joint_actual_pos_" + std::to_string(i));
  }

  // Joint actual vel
  for (size_t i = 0; i < joint_size_; ++i) {
    headers.push_back("joint_actual_vel_" + std::to_string(i));
  }

  // Motor target pos
  for (size_t i = 0; i < motor_size_; ++i) {
    headers.push_back("motor_target_pos_" + std::to_string(i));
  }

  // Motor target torque
  for (size_t i = 0; i < motor_size_; ++i) {
    headers.push_back("motor_target_torque_" + std::to_string(i));
  }

  // Joint target pos
  for (size_t i = 0; i < joint_size_; ++i) {
    headers.push_back("joint_target_pos_" + std::to_string(i));
  }

  // Joint target torque
  for (size_t i = 0; i < joint_size_; ++i) {
    headers.push_back("joint_target_torque_" + std::to_string(i));
  }

  // Acc
  headers.push_back("acc_x");
  headers.push_back("acc_y");
  headers.push_back("acc_z");

  // Ang vel
  headers.push_back("ang_vel_x");
  headers.push_back("ang_vel_y");
  headers.push_back("ang_vel_z");

  // Euler RPY
  headers.push_back("euler_roll");
  headers.push_back("euler_pitch");
  headers.push_back("euler_yaw");

  // Proj gravity
  headers.push_back("proj_gravity_x");
  headers.push_back("proj_gravity_y");
  headers.push_back("proj_gravity_z");

  csv_logger_ = std::make_shared<CsvLogger>(logger_file, headers);
}

void RobotFc2Mj::ObserverFc2Mj::WriteLog() {
  std::vector<CsvLogger::Number> datas;

  // Motor actual pos
  for (size_t i = 0; i < motor_size_; ++i) {
    datas.push_back(motor_actual_position_[i]);
  }

  // Motor actual vel
  for (size_t i = 0; i < motor_size_; ++i) {
    datas.push_back(motor_actual_velocity_[i]);
  }

  // Joint actual pos
  for (size_t i = 0; i < joint_size_; ++i) {
    datas.push_back(joint_actual_position_[i]);
  }

  // Joint actual vel
  for (size_t i = 0; i < joint_size_; ++i) {
    datas.push_back(joint_actual_velocity_[i]);
  }

  // Motor target pos
  for (size_t i = 0; i < motor_size_; ++i) {
    datas.push_back(robot_->Executor()->MotorTargetPosition()[i]);
  }

  // Motor target torque
  for (size_t i = 0; i < motor_size_; ++i) {
    datas.push_back(robot_->Executor()->MotorTargetTorque()[i]);
  }

  // Joint target pos
  for (size_t i = 0; i < joint_size_; ++i) {
    datas.push_back(robot_->Executor()->JointTargetPosition()[i]);
  }

  // Joint target torque
  for (size_t i = 0; i < joint_size_; ++i) {
    datas.push_back(robot_->Executor()->JointTargetTorque()[i]);
  }

  // Acc
  datas.push_back(acceleration_[0]);
  datas.push_back(acceleration_[1]);
  datas.push_back(acceleration_[2]);

  // Ang vel
  datas.push_back(angular_velocity_[0]);
  datas.push_back(angular_velocity_[1]);
  datas.push_back(angular_velocity_[2]);

  // Euler RPY
  datas.push_back(euler_rpy_[0]);
  datas.push_back(euler_rpy_[1]);
  datas.push_back(euler_rpy_[2]);

  // Proj gravity
  datas.push_back(proj_gravity_[0]);
  datas.push_back(proj_gravity_[1]);
  datas.push_back(proj_gravity_[2]);

  csv_logger_->Write(datas);
}

}  // namespace ovinf

#endif  // !ROBOT_HHFC_MJ_HPP
