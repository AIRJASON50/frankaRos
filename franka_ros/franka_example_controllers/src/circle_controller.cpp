// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/circle_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

bool CircleController::init(hardware_interface::RobotHW* robot_hw,
                           ros::NodeHandle& node_handle) {
  // 获取机器人ID参数
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CircleController: Could not read parameter arm_id");
    return false;
  }
  
  // 获取关节名称参数
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CircleController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  // 从参数服务器读取圆周运动参数
  if (!node_handle.getParam("circle_radius", circle_radius_)) {
    ROS_INFO_STREAM("CircleController: Using default circle radius: " << circle_radius_);
  }
  if (!node_handle.getParam("circle_frequency", circle_frequency_)) {
    ROS_INFO_STREAM("CircleController: Using default circle frequency: " << circle_frequency_);
  }

  // 获取机器人模型接口
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CircleController: Error getting model interface from hardware");
    return false;
  }
  
  // 尝试获取模型句柄
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CircleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // 获取机器人状态接口
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CircleController: Error getting state interface from hardware");
    return false;
  }
  
  // 尝试获取状态句柄
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CircleController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // 获取关节力矩控制接口
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CircleController: Error getting effort joint interface from hardware");
    return false;
  }
  
  // 获取每个关节的控制句柄
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CircleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // 设置笛卡尔空间阻抗控制参数（刚度矩阵）
  cartesian_stiffness_.setIdentity();
  // 设置位置刚度（前3x3矩阵）
  cartesian_stiffness_.topLeftCorner(3, 3) << 100.0 * Eigen::Matrix3d::Identity();
  // 设置方向刚度（后3x3矩阵）
  cartesian_stiffness_.bottomRightCorner(3, 3) << 10.0 * Eigen::Matrix3d::Identity();
  
  // 设置阻尼矩阵（用于稳定系统）
  cartesian_damping_.setIdentity();
  // 设置位置阻尼
  cartesian_damping_.topLeftCorner(3, 3) << 2.0 * sqrt(100.0) * Eigen::Matrix3d::Identity();
  // 设置方向阻尼
  cartesian_damping_.bottomRightCorner(3, 3) << 2.0 * sqrt(10.0) * Eigen::Matrix3d::Identity();

  return true;
}

void CircleController::starting(const ros::Time& /*time*/) {
  // 获取初始状态
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // 设置圆周中心为初始位置
  circle_center_ = initial_transform.translation();
  
  // 设置初始目标位置和方向
  position_d_ = circle_center_;
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  
  // 设置零空间平衡配置为初始关节角度
  q_d_nullspace_ = q_initial;
  
  // 重置时间
  elapsed_time_ = 0.0;
}

void CircleController::update(const ros::Time& /*time*/,
                             const ros::Duration& period) {
  // 更新已经过的时间
  elapsed_time_ += period.toSec();
  
  // 获取机器人状态
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // 转换为Eigen类型
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  // 计算圆周运动的目标位置
  // 在XY平面上作圆周运动，Z轴保持不变
  double angle = 2.0 * M_PI * circle_frequency_ * elapsed_time_;
  position_d_(0) = circle_center_(0) + circle_radius_ * cos(angle);
  position_d_(1) = circle_center_(1) + circle_radius_ * sin(angle);
  position_d_(2) = circle_center_(2);  // Z轴保持不变

  // 计算位置和方向误差
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // 方向误差
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.rotation() * error.tail(3);

  // 计算控制输入
  // 分配变量
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // 伪逆用于零空间处理
  // 运动学伪逆
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // 笛卡尔PD控制，阻尼比 = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // 零空间PD控制，阻尼比 = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // 期望力矩
  tau_d << tau_task + tau_nullspace + coriolis;
  // 饱和力矩率以避免不连续性
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }
}

Eigen::Matrix<double, 7, 1> CircleController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CircleController,
                       controller_interface::ControllerBase)
