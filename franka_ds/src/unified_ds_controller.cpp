// Copyright (c) 2024 Franka DS Project
// Use of this source code is governed by the Apache-2.0 license

#include <franka_ds/unified_ds_controller.h>
#include <franka_ds/ds_utils.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <cmath>  // 添加数学函数和常量支持

namespace franka_ds {

bool UnifiedDSController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  ROS_INFO("Unified DS-Impedance Controller initialization starting...");
  
  // 获取硬件接口
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR("UnifiedDSController: Error getting effort joint interface from hardware");
    return false;
  }
  
  auto* franka_state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (franka_state_interface == nullptr) {
    ROS_ERROR("UnifiedDSController: Could not get Franka state interface from hardware");
    return false;
  }
  
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR("UnifiedDSController: Could not get model interface from hardware");
    return false;
  }
  
  // 获取关节名称
  std::vector<std::string> joint_names;
  if (!controller_nh.getParam("joint_names", joint_names)) {
    ROS_ERROR("UnifiedDSController: Could not parse joint names");
    return false;
  }
  
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("UnifiedDSController: Wrong number of joint names, got " << joint_names.size() << " instead of 7 names!");
    return false;
  }
  
  // 获取关节句柄
  joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_[i] = effort_joint_interface->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("UnifiedDSController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  
  // 获取机器人状态和模型句柄
  std::string arm_id;
  if (!controller_nh.getParam("arm_id", arm_id)) {
    ROS_ERROR("UnifiedDSController: Could not read parameter arm_id");
    return false;
  }
  
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(franka_state_interface->getHandle(arm_id + "_robot"));
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("UnifiedDSController: Exception getting state handle: " << ex.what());
    return false;
  }
  
  // 加载DS-阻抗参数
  if (!loadDSImpedanceParameters(controller_nh)) {
    ROS_ERROR("Failed to load DS-Impedance parameters");
    return false;
  }
  ROS_INFO("DS-Impedance parameters loaded successfully");
  
  // 初始化能量罐管理器
  energy_tank_manager_ = std::make_unique<EnergyTankManager>(ds_impedance_params_.energy_tank_max_);
  ROS_INFO_STREAM("EnergyTankManager initialized: Max energy=" << ds_impedance_params_.energy_tank_max_ 
                  << "J, Initial energy=" << energy_tank_manager_->getCurrentEnergy() << "J");
  
  // 初始化机器人状态管理器
  // robot_state_manager_ = std::make_unique<DSRobotState>(); // Removed as per edit hint
  
  // 初始化ROS接口
  initializeROSInterface(controller_nh);
  ROS_INFO("ROS interface initialized");
  
  // 初始化控制状态
  current_phase_ = ControlPhase::CALIBRATION;
  user_start_command_received_ = false;
  force_sensor_calibrated_ = false;
  debug_print_counter_ = 0;
  
  // 初始化力传感器相关变量
  external_force_.setZero();
  contact_force_.setZero();
  baseline_force_z_ = 0.0;
  
  ROS_INFO("Unified DS-Impedance Controller initialization successful");
  ROS_INFO_STREAM("Parameters: Circle radius=" << ds_impedance_params_.circular_radius_ 
                  << "m, Linear velocity=" << ds_impedance_params_.linear_max_velocity_ 
                  << "m/s, Contact threshold=" << ds_impedance_params_.contact_force_threshold_ << "N");
  
  return true;
}

void UnifiedDSController::starting(const ros::Time& time) {
  ROS_INFO("Unified DS-Impedance Controller starting");
  
  // 重置能量罐
  energy_tank_manager_->resetEnergyTank();
  ROS_INFO_STREAM("Energy tank reset complete, current energy=" << energy_tank_manager_->getCurrentEnergy() << "J");
  
  // 获取初始机器人状态
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  
  // 设置初始位置和姿态
  initial_position_ = initial_transform.translation();
  initial_orientation_ = Eigen::Quaterniond(initial_transform.rotation());
  
  // 保存初始关节位置
  initial_joint_positions_ = initial_state.q;
  
  // 设置目标位置 - 使用配置文件中的正确目标位置
  target_position_ = ds_impedance_params_.target_position;  // 使用配置文件中的目标位置
  target_orientation_ = initial_orientation_; // 保持初始方向
  
  // 初始化控制状态
  current_phase_ = ControlPhase::CALIBRATION;
  controller_start_time_ = time;
  phase_start_time_ = time;
  
  ROS_INFO("DS-Impedance Controller ready, waiting for initialization");
  ROS_INFO_STREAM("Initial position: [" << initial_position_.transpose() << "], distance to target: " 
                  << (target_position_ - initial_position_).norm() << " m");
}

void UnifiedDSController::update(const ros::Time& time, const ros::Duration& period) {
  // 获取当前机器人状态
  franka::RobotState robot_state = state_handle_->getRobotState();
  
  // 获取动力学参数
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 49> mass_array = model_handle_->getMass();
  
  // 转换为Eigen格式
  Eigen::Map<Vector7d> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
  Eigen::Map<Vector7d> q(robot_state.q.data());
  Eigen::Map<Vector7d> dq(robot_state.dq.data());
  Eigen::Map<Vector7d> tau_J_d(robot_state.tau_J_d.data());
  
  // 获取当前末端执行器状态 - 使用预分配的缓存变量避免频繁分配
  transform_cache_ = Eigen::Affine3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  position_cache_ = transform_cache_.translation();
  orientation_cache_ = Eigen::Quaterniond(transform_cache_.rotation());
  
  // 计算当前笛卡尔速度 - 使用预分配的缓存变量
  cartesian_velocity_cache_ = jacobian * dq;
  
  // 更新控制阶段
  updateControlPhase(time.toSec(), position_cache_, external_force_);
  
  // 计算DS调制的期望速度 - 使用缓存的位置
  nominal_ds_velocity_cache_ = computeDSModulatedVelocity(position_cache_);
  
  // *** 计算la (调制增益) - 参照 SurfacePolishing.cpp::computeModulatedDS ***
  double d1 = ds_impedance_params_.ds_impedance_gain_;
  double gammap = ds_impedance_params_.force_modulation_gain_;
  double desired_force = computeDesiredContactForce(position_cache_);
  Eigen::Vector3d surface_normal(0.0, 0.0, 1.0); // 表面法向量

  double delta_val = std::pow(2.0 * surface_normal.dot(nominal_ds_velocity_cache_) * gammap * desired_force / d1, 2.0) +
                     4.0 * std::pow(nominal_ds_velocity_cache_.norm(), 4.0);
  
  double la;
  if (std::abs(nominal_ds_velocity_cache_.norm()) < 1e-6) {
    la = 0.0;
  } else {
    la = (-2.0 * surface_normal.dot(nominal_ds_velocity_cache_) * gammap * desired_force / d1 + std::sqrt(delta_val)) /
         (2.0 * std::pow(nominal_ds_velocity_cache_.norm(), 2.0));
  }

  // 如果能量过低且标称功率为负 (表示DS在消耗能量，帮助能量罐恢复)，则 la = 1.0
  // 这部分逻辑在 energy_tank_manager.cpp::updateScalarFunctions 中通过 beta 进行调整
  // 这里保持 la 的原始计算，让 energy_tank_manager 负责最终的无源性保障

  // 能量罐管理和无源性约束
  constrained_velocity_cache_ = nominal_ds_velocity_cache_;
  if (current_phase_ != ControlPhase::CALIBRATION && current_phase_ != ControlPhase::INITIALIZATION) {
    // *** 修复：能量罐力功率计算 ***
    // 使用真实的期望接触力，而不是恒定的0
    Eigen::Matrix3d damping_matrix = ds_impedance_params_.cartesian_damping_pos_;
    // double desired_force = computeDesiredContactForce(position_cache_);  // 已在上面计算
    // Eigen::Vector3d surface_normal = Eigen::Vector3d::UnitZ(); // 已在上面定义
    
    energy_tank_manager_->updateTankDynamics(cartesian_velocity_cache_.head(3), 
                                            nominal_ds_velocity_cache_,
                                            damping_matrix,
                                            desired_force,  // 不再恒为0
                                            surface_normal,
                                            la, // 传递la参数
                                            period.toSec());
    
    // 应用无源性约束
    constrained_velocity_cache_ = energy_tank_manager_->constrainVelocity(nominal_ds_velocity_cache_);
  }
  
  // 计算阻抗控制力矩 - 使用缓存变量
  tau_d_cache_ = computeImpedanceControl(constrained_velocity_cache_,
                                         transform_cache_,
                                         cartesian_velocity_cache_,
                                         jacobian);
  
  // 添加重力补偿和科里奥利力补偿
  tau_d_cache_ += coriolis;
  
  // 限制力矩变化率
  tau_d_cache_ = saturateTorqueRate(tau_d_cache_, tau_J_d);
  
  // 添加安全力矩限制，防止触发机器人安全反射
  const double MAX_JOINT_TORQUE = ds_impedance_params_.max_joint_torque_;  // 使用配置的关节力矩限制
  for (size_t i = 0; i < 7; ++i) {
    tau_d_cache_(i) = std::max(-MAX_JOINT_TORQUE, std::min(MAX_JOINT_TORQUE, tau_d_cache_(i)));
  }
  
  // 发送力矩命令到关节
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_cache_(i));
  }
  
  // 发布状态信息
  if (debug_print_counter_++ % kDebugPrintRate == 0) {
    publishControlStatus();
    publishEnergyStatus();
    
    // 详细调试输出
    ROS_INFO_STREAM("=== DS Controller Debug Info ===");
    ROS_INFO_STREAM("Phase: " << static_cast<int>(current_phase_) 
                    << " (" << (current_phase_ == ControlPhase::CIRCULAR_MOTION ? "CIRCULAR_MOTION" : 
                               current_phase_ == ControlPhase::ACCELERATION ? "ACCELERATION" : "OTHER") << ")");
    ROS_INFO_STREAM("Position: [" << position_cache_.transpose() << "]");
    ROS_INFO_STREAM("Target: [" << target_position_.transpose() << "]");
    ROS_INFO_STREAM("Distance to target: " << (target_position_ - position_cache_).norm() << " m");
    ROS_INFO_STREAM("Nominal DS velocity: [" << nominal_ds_velocity_cache_.transpose() << "], norm: " << nominal_ds_velocity_cache_.norm());
    ROS_INFO_STREAM("Constrained velocity: [" << constrained_velocity_cache_.transpose() << "], norm: " << constrained_velocity_cache_.norm());
    ROS_INFO_STREAM("Current velocity: [" << cartesian_velocity_cache_.head(3).transpose() << "], norm: " << cartesian_velocity_cache_.head(3).norm());
    ROS_INFO_STREAM("Torque command norm: " << tau_d_cache_.norm() << " Nm");
    ROS_INFO_STREAM("================================");
  }
}

void UnifiedDSController::stopping(const ros::Time& time) {
  ROS_WARN("Unified DS-Impedance Controller stopping");
  
  // 发送零力矩命令
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(0.0);
  }
}

Eigen::Vector3d UnifiedDSController::computeDSModulatedVelocity(const Eigen::Vector3d& current_position) {
  switch (current_phase_) {
    case ControlPhase::CALIBRATION:
    case ControlPhase::INITIALIZATION:
      return Eigen::Vector3d::Zero();
      
    case ControlPhase::ACCELERATION: {
      // 加速阶段：从零速度逐渐增加到圆周DS速度（不是线性DS）
      double acceleration_time = (ros::Time::now() - phase_start_time_).toSec();
      double progress = std::min(acceleration_time / ds_impedance_params_.acceleration_duration_, 1.0);
      
      // 计算标称DS速度和力调制项
      Eigen::Vector3d nominal_ds = computeCircularDS(current_position);
      Eigen::Vector3d force_modulated_ds = computeForceModulation(current_position);
      
      // 统一的DS速度场：f(x) + fn(x)
      Eigen::Vector3d unified_ds = nominal_ds + force_modulated_ds;
      
      return progress * unified_ds;
    }
    
    case ControlPhase::LINEAR_APPROACH: {
      // 此阶段现在被跳过，直接在加速后进入圆周运动
      current_phase_ = ControlPhase::CIRCULAR_MOTION;
      phase_start_time_ = ros::Time::now();
      ROS_INFO("Switching to CIRCULAR_MOTION phase");
      
      // 返回统一的DS速度场
      Eigen::Vector3d nominal_ds = computeCircularDS(current_position);
      Eigen::Vector3d force_modulated_ds = computeForceModulation(current_position);
      return nominal_ds + force_modulated_ds;
    }
    
    case ControlPhase::PROBE_DESCENT: {
      // 此阶段现在被跳过，直接进入圆周运动
      current_phase_ = ControlPhase::CIRCULAR_MOTION;
      phase_start_time_ = ros::Time::now();
      ROS_INFO("Switching to CIRCULAR_MOTION phase");
      
      // 返回统一的DS速度场
      Eigen::Vector3d nominal_ds = computeCircularDS(current_position);
      Eigen::Vector3d force_modulated_ds = computeForceModulation(current_position);
      return nominal_ds + force_modulated_ds;
    }
    
    case ControlPhase::CIRCULAR_MOTION: {
      // *** 核心修复：DS统一力-运动生成 ***
      // 按照论文公式：x˙d = f(x) + fn(x)
      // 其中 f(x) 是标称DS，fn(x) 是力调制项
      
      Eigen::Vector3d nominal_ds = computeCircularDS(current_position);
      Eigen::Vector3d force_modulated_ds = computeForceModulation(current_position);
      
      // 统一的DS速度场
      return nominal_ds + force_modulated_ds;
    }
    
    default:
      return Eigen::Vector3d::Zero();
  }
}

Vector7d UnifiedDSController::computeImpedanceControl(const Eigen::Vector3d& desired_velocity,
                                                      const Eigen::Affine3d& transform,
                                                      const Eigen::Matrix<double, 6, 1>& current_velocity,
                                                      const Eigen::Matrix<double, 6, 7>& jacobian) {
  // *** 修复：DS-阻抗控制律，移除分离的力控制 ***
  // 按照论文方程：Fc = d1*f(x) + d1*fn(x) - D(x)*x_dot
  // 由于力已经融入DS速度场，这里只需要实现速度跟踪控制
  
  // 获取当前线性和角速度
  Eigen::Vector3d current_linear_velocity = current_velocity.head(3);
  Eigen::Vector3d current_angular_velocity = current_velocity.tail(3);
  
  // DS阻抗增益 d1
  double d1 = ds_impedance_params_.ds_impedance_gain_;
  
  // 计算阻尼矩阵 D(x)
  Eigen::Matrix3d damping_matrix = Eigen::Matrix3d::Identity() * ds_impedance_params_.ds_damping_gain_;
  
  // *** DS控制律：线性部分 ***
  // 公式：Fc = d1 * x˙d - D(x) * x˙
  // 其中 x˙d 已经包含了 f(x) + fn(x)（标称DS + 力调制）
  Eigen::Vector3d ds_driving_force = d1 * desired_velocity;  // d1 * (f(x) + fn(x))
  Eigen::Vector3d damping_force = -damping_matrix * current_linear_velocity;  // -D(x) * x˙
  
  // 合成笛卡尔控制力（不再需要额外的力项）
  Eigen::Vector3d cartesian_force = ds_driving_force + damping_force;
  
  // *** 姿态控制 - 强制保持末端执行器垂直 ***
  Eigen::Quaterniond current_orientation(transform.rotation());
  
  // 目标姿态：末端执行器Z轴垂直向下
  Eigen::Vector3d desired_z_axis(0.0, 0.0, -1.0);
  Eigen::Vector3d current_z_axis = transform.rotation().col(2);
  
  // 计算旋转误差
  Eigen::Vector3d rotation_axis = current_z_axis.cross(desired_z_axis);
  double rotation_angle = acos(std::max(-1.0, std::min(1.0, current_z_axis.dot(desired_z_axis))));
  
  Eigen::Vector3d orientation_error = Eigen::Vector3d::Zero();
  if (rotation_axis.norm() > 1e-6) {
    rotation_axis.normalize();
    orientation_error = rotation_angle * rotation_axis;
  }
  
  // 姿态控制增益
  double orientation_stiffness = ds_impedance_params_.orientation_stiffness_;
  double orientation_damping = ds_impedance_params_.orientation_damping_;
  
  // 添加死区以减少小幅抖动
  if (orientation_error.norm() < 0.01) {
    orientation_error.setZero();
  }
  
  // 姿态控制力矩
  Eigen::Vector3d cartesian_torque = orientation_stiffness * orientation_error - 
                                     orientation_damping * current_angular_velocity;
  
  // 限制最大力矩
  double max_torque = ds_impedance_params_.max_orientation_torque_;
  if (cartesian_torque.norm() > max_torque) {
    cartesian_torque = cartesian_torque.normalized() * max_torque;
  }
  
  // 组合6D控制力矩
  Eigen::Matrix<double, 6, 1> cartesian_wrench;
  cartesian_wrench.head(3) = cartesian_force;
  cartesian_wrench.tail(3) = cartesian_torque;
  
  // 转换为关节力矩
  Vector7d tau_task = jacobian.transpose() * cartesian_wrench;
  
  // 简化的零空间控制
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<const Vector7d> q_current(robot_state.q.data());
  Eigen::Map<const Vector7d> dq_current(robot_state.dq.data());
  Eigen::Map<const Vector7d> q_initial(initial_joint_positions_.data());
  
  Vector7d tau_nullspace = ds_impedance_params_.nullspace_stiffness_ * (q_initial - q_current) - 
                          (2.0 * sqrt(ds_impedance_params_.nullspace_stiffness_)) * dq_current;
  
  // 计算零空间投影矩阵
  Eigen::Matrix<double, 6, 6> lambda = (jacobian * jacobian.transpose());
  
  // 检查矩阵是否可逆
  if (lambda.determinant() < 1e-6) {
    lambda += 1e-6 * Eigen::Matrix<double, 6, 6>::Identity();
  }
  
  Eigen::Matrix<double, 7, 6> jacobian_pinv = jacobian.transpose() * lambda.inverse();
  Eigen::Matrix<double, 7, 7> nullspace_projector = Eigen::Matrix<double, 7, 7>::Identity() - jacobian_pinv * jacobian;
  
  // 合成最终力矩命令
  Vector7d tau_d = tau_task + nullspace_projector * tau_nullspace;
  
  return tau_d;
}

Eigen::Vector3d UnifiedDSController::computeLinearDS(const Eigen::Vector3d& current_position) {
  Eigen::Vector3d direction = target_position_ - current_position;
  double distance = direction.norm();
  
  if (distance < 1e-6) {
    return Eigen::Vector3d::Zero();
  }
  
  direction.normalize();
  double velocity_magnitude = ds_impedance_params_.linear_lambda_ * distance;
  velocity_magnitude = std::min(velocity_magnitude, ds_impedance_params_.linear_max_velocity_);
  
  return velocity_magnitude * direction;
}

Eigen::Vector3d UnifiedDSController::computeCircularDS(const Eigen::Vector3d& current_position) {
  // 修复：使用contact_surface_height作为圆周运动的目标高度
  Eigen::Vector3d attractor = ds_impedance_params_.target_position;
  attractor(2) = ds_impedance_params_.contact_surface_height_;  // 使用专门设置的接触面高度
  
  // 计算相对位置和距离
  Eigen::Vector3d relative_position = current_position - attractor;
  double distance_to_attractor = relative_position.norm();
  
  // 圆周运动参数
  double target_radius = ds_impedance_params_.circular_radius_;
  
  // === 1. 改进的接近场设计（参考文档原版DS） ===
  Eigen::Vector3d approach_velocity = Eigen::Vector3d::Zero();
  if (distance_to_attractor > 1e-6) {
    // 使用更强的接近增益，特别是对z方向
    double approach_gain_xy = ds_impedance_params_.approach_gain_;  // XY方向接近增益
    double approach_gain_z = 5.0 * ds_impedance_params_.approach_gain_;  // Z方向使用更强的增益
    
    approach_velocity(0) = -approach_gain_xy * relative_position(0);
    approach_velocity(1) = -approach_gain_xy * relative_position(1);
    approach_velocity(2) = -approach_gain_z * relative_position(2);  // 强化高度收敛
    
    // 限制接近速度幅值
    double max_approach_speed = ds_impedance_params_.linear_max_velocity_;
    if (approach_velocity.norm() > max_approach_speed) {
      approach_velocity = approach_velocity.normalized() * max_approach_speed;
    }
  }
  
  // === 2. 改进的圆周运动速度 ===
  Eigen::Vector3d circular_velocity = Eigen::Vector3d::Zero();
  
  // 水平面圆周运动（X-Y平面）
  double R_xy = sqrt(relative_position(0) * relative_position(0) + 
                     relative_position(1) * relative_position(1));
  
  if (R_xy > 1e-6) {
    // 径向分量：收缩到目标半径
    double radial_gain = ds_impedance_params_.radial_gain_;
    double radial_velocity = -radial_gain * (R_xy - target_radius);
    
    // 切向分量：固定切向速度，确保圆周运动均匀
    double constant_tangential_speed = ds_impedance_params_.constant_tangential_speed_;
    
    // 转换为笛卡尔坐标
    double cos_theta = relative_position(0) / R_xy;
    double sin_theta = relative_position(1) / R_xy;
    
    circular_velocity(0) = radial_velocity * cos_theta - constant_tangential_speed * sin_theta;
    circular_velocity(1) = radial_velocity * sin_theta + constant_tangential_speed * cos_theta;
  }
  
  // Z方向：使用专门的高度控制，确保在接触面高度进行圆周运动
  circular_velocity(2) = -5.0 * relative_position(2);  // 更强的高度控制
  
  // === 3. 原版DS系统的平滑混合策略（参考SurfacePolishing.cpp:409-424） ===
  double blend_distance = ds_impedance_params_.blend_distance_;
  
  // 计算表面法向量（这里简化为垂直向上，实际应该是动态计算的）
  Eigen::Vector3d surface_normal(0.0, 0.0, 1.0);
  
  // 计算法向距离（到接触面的距离）
  double normal_distance = current_position(2) - ds_impedance_params_.contact_surface_height_;
  
  // 原版DS系统的角度计算策略
  // 基于法向距离的tanh函数，实现平滑过渡
  double theta_max = M_PI / 3.0;  // 最大混合角度60度，增强混合效果
  
  // 改进的tanh混合函数，主要基于法向距离
  double normalized_normal_distance = std::abs(normal_distance) / blend_distance;
  
  // 使用原版DS的tanh函数形式：(1.0 - tanh(10*_normalDistance)) * angle
  // 这里的系数10来自原版DS代码
  double theta = (1.0 - std::tanh(10.0 * normalized_normal_distance)) * theta_max;
  
  // 额外考虑水平距离的影响
  double horizontal_distance = sqrt(relative_position(0) * relative_position(0) + 
                                   relative_position(1) * relative_position(1));
  
  // 当接近目标半径时，增强圆周运动的权重
  double radius_factor = 1.0;
  if (horizontal_distance < target_radius * 2.0) {
    radius_factor = 0.5 + 0.5 * (horizontal_distance / (target_radius * 2.0));
    theta *= radius_factor;  // 在目标半径附近减小混合角度，增强圆周运动
  }
  
  // 构建3D旋转矩阵R（原版DS的核心机制）
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  
  // 基于theta角度的旋转矩阵计算
  double cos_theta = std::cos(theta);
  double sin_theta = std::sin(theta);
  
  // 原版DS的旋转矩阵：绕表面法向量旋转
  // 这里简化为绕Z轴旋转（因为表面是水平的）
  R(0, 0) = cos_theta;
  R(0, 1) = -sin_theta;
  R(1, 0) = sin_theta;  
  R(1, 1) = cos_theta;
  R(2, 2) = 1.0;
  
  // 原版DS核心公式：_fx = R * v0
  // v0是基础速度场的组合
  Eigen::Vector3d v0;
  
  // 根据距离和状态动态调整基础速度场组合
  if (std::abs(normal_distance) > blend_distance * 0.8) {
    // 远离接触面时：主要是接近运动
    v0 = 0.9 * approach_velocity + 0.1 * circular_velocity;
  } else if (horizontal_distance > target_radius * 1.2) {
    // 在接触面附近但远离目标半径：混合接近和圆周
    v0 = 0.6 * approach_velocity + 0.4 * circular_velocity;
  } else {
    // 在目标半径附近：主要是圆周运动
    v0 = 0.2 * approach_velocity + 0.8 * circular_velocity;
  }
  
  // 应用旋转矩阵进行最终混合
  Eigen::Vector3d final_velocity = R * v0;
  
  // 特别强化z方向的收敛：当z偏差较大时，直接使用接近速度
  double z_error = std::abs(relative_position(2));
  if (z_error > 0.02) {  // 进一步降低阈值到2cm，更精确的高度控制
    // 保持XY方向的混合，但Z方向使用强化的接近控制
    double z_gain = 10.0;  // 强力Z方向增益
    final_velocity(2) = -z_gain * relative_position(2);
  }
  
  // === 5. 速度限制和归一化 ===
  double velocity_limit = ds_impedance_params_.max_velocity_;
  if (final_velocity.norm() > velocity_limit) {
    final_velocity = final_velocity.normalized() * velocity_limit;
  }
  
  // 确保最小速度，避免停滞
  double min_velocity = ds_impedance_params_.min_velocity_;
  if (final_velocity.norm() < min_velocity && distance_to_attractor > 1e-3) {
    final_velocity = final_velocity.normalized() * min_velocity;
  }
  
  return final_velocity;
}

Eigen::Vector3d UnifiedDSController::computeForceModulation(const Eigen::Vector3d& current_position) {
  // *** 核心修复：实现DS统一力调制机制 ***
  // 按照论文公式：fn(x) = Fd(x)/d1 * n(x)
  // 其中 Fd(x) 是期望力，n(x) 是表面法向量，d1 是阻抗增益
  
  // 1. 计算表面法向量（简化版：垂直向上，接触面为水平面）
  Eigen::Vector3d surface_normal(0.0, 0.0, 1.0);  // 表面法向量垂直向上
  
  // 2. 计算期望接触力 Fd(x)（动态调整，参考原版DS）
  double desired_force = computeDesiredContactForce(current_position);
  
  // 3. 获取DS阻抗增益 d1
  double d1 = ds_impedance_params_.ds_impedance_gain_;
  
  // 4. 计算力调制速度项：fn(x) = Fd(x)/d1 * n(x)
  Eigen::Vector3d force_modulation = (desired_force / d1) * surface_normal;
  
  return force_modulation;
}

double UnifiedDSController::computeDesiredContactForce(const Eigen::Vector3d& current_position) {
  // *** 动态力生成逻辑（参考原版DS SurfacePolishing.cpp:505-516） ***
  
  // 计算到接触面的距离
  double distance_to_surface = current_position(2) - ds_impedance_params_.contact_surface_height_;
  
  // 检查是否接触
  bool is_in_contact = (distance_to_surface <= 0.01) && (external_force_.norm() > ds_impedance_params_.contact_force_threshold_);
  
  if (!is_in_contact) {
    // 未接触时：期望力为0（自由空间运动）
    return 0.0;
  } else {
    // 接触时：根据当前法向力动态调整期望力
    double current_normal_force = std::abs(external_force_(2));  // 简化：使用z方向力作为法向力
    
    if (current_normal_force < 3.0) {
      // 如果当前力小于最小阈值，增加期望力
      return 5.0;  // 增加到目标力
    } else {
      // 如果当前力足够，使用目标力
      return ds_impedance_params_.desired_normal_force_;
    }
  }
}

Eigen::Vector3d UnifiedDSController::computeProbeDS(const Eigen::Vector3d& current_position) {
  // 垂直向下探测
  Eigen::Vector3d probe_velocity;
  probe_velocity << 0.0, 0.0, -ds_impedance_params_.exploration_speed_;
  return probe_velocity;
}

bool UnifiedDSController::detectContact(const Eigen::Vector3d& external_force) {
  double force_magnitude = external_force.norm();
  return force_magnitude > ds_impedance_params_.contact_force_threshold_;
}

Vector7d UnifiedDSController::saturateTorqueRate(const Vector7d& tau_d_calculated, const Vector7d& tau_J_d) {
  Vector7d tau_d_saturated;
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

void UnifiedDSController::updateControlPhase(double current_time, 
                                              const Eigen::Vector3d& current_position, 
                                              const Eigen::Vector3d& external_force) {
  switch (current_phase_) {
    case ControlPhase::CALIBRATION: {
      // 等待力传感器校准完成
      if (force_sensor_calibrated_) {
        current_phase_ = ControlPhase::INITIALIZATION;
        phase_start_time_ = ros::Time::now();
        ROS_INFO("Calibration complete. Send 'start' command to begin motion.");
        ROS_INFO_STREAM("Initial position: [" << current_position.transpose() 
                       << "], distance to target: " << (target_position_ - current_position).norm() << " m");
        ROS_INFO_STREAM("Baseline force: " << external_force.norm() << " N");
      }
      break;
    }
    
    case ControlPhase::INITIALIZATION: {
      // *** 关键修复：等待用户"start"命令，不自动进入下一阶段 ***
      if (user_start_command_received_) {
        current_phase_ = ControlPhase::ACCELERATION;
        phase_start_time_ = ros::Time::now();
        user_start_command_received_ = false;  // 重置标志
        ROS_INFO_STREAM("Starting motion to target position [" << target_position_.transpose() << "]");
        ROS_INFO_STREAM("Distance to target: " << (target_position_ - current_position).norm() 
                       << " m, Acceleration duration: " << ds_impedance_params_.acceleration_duration_ << " seconds");
      }
      // 否则保持在INITIALIZATION阶段，等待用户命令
      break;
    }
    
    case ControlPhase::ACCELERATION: {
      double acceleration_time = (ros::Time::now() - phase_start_time_).toSec();
      if (acceleration_time >= ds_impedance_params_.acceleration_duration_) {
        current_phase_ = ControlPhase::CIRCULAR_MOTION;
        phase_start_time_ = ros::Time::now();
        ROS_INFO("Acceleration complete, switching directly to CIRCULAR_MOTION phase");
      }
      break;
    }
    
    case ControlPhase::LINEAR_APPROACH: {
      // 此阶段现在被跳过，直接在加速后进入圆周运动
      current_phase_ = ControlPhase::CIRCULAR_MOTION;
      phase_start_time_ = ros::Time::now();
      ROS_INFO("Switching to CIRCULAR_MOTION phase");
      break;
    }
    
    case ControlPhase::PROBE_DESCENT: {
      // 此阶段现在被跳过，直接进入圆周运动
      current_phase_ = ControlPhase::CIRCULAR_MOTION;
      phase_start_time_ = ros::Time::now();
      ROS_INFO("Switching to CIRCULAR_MOTION phase");
      break;
    }
    
    case ControlPhase::CIRCULAR_MOTION: {
      // 持续圆形运动，直到外部停止
      break;
    }
  }
}

bool UnifiedDSController::loadDSImpedanceParameters(ros::NodeHandle& nh) {
  // 目标位置参数
  std::vector<double> target_pos;
  if (nh.getParam("target_position", target_pos) && target_pos.size() == 3) {
    ds_impedance_params_.target_position << target_pos[0], target_pos[1], target_pos[2];
  } else {
    ds_impedance_params_.target_position << 0.483835, 0.089474, 0.193932;
    ROS_WARN("Using default target position: [0.483835, 0.089474, 0.193932]");
  }
  
  // DS参数
  nh.param("linear_lambda", ds_impedance_params_.linear_lambda_, 2.0);
  nh.param("linear_max_velocity", ds_impedance_params_.linear_max_velocity_, 0.05);
  nh.param("circular_radius", ds_impedance_params_.circular_radius_, 0.02);  // 修复：默认值改为0.02匹配配置文件
  // circular_omega 参数已删除 - 代码中实际使用 constant_tangential_speed
  nh.param("exploration_speed", ds_impedance_params_.exploration_speed_, 0.01);
  
  // DS控制核心参数
  nh.param("ds_impedance_gain", ds_impedance_params_.ds_impedance_gain_, 150.0);
  nh.param("ds_damping_gain", ds_impedance_params_.ds_damping_gain_, 25.0);
  nh.param("approach_gain", ds_impedance_params_.approach_gain_, 2.0);
  nh.param("radial_gain", ds_impedance_params_.radial_gain_, 1.0);
  nh.param("constant_tangential_speed", ds_impedance_params_.constant_tangential_speed_, 0.3);  // 修复：默认值改为0.3匹配配置文件
  nh.param("blend_distance", ds_impedance_params_.blend_distance_, 0.1);
  nh.param("min_velocity", ds_impedance_params_.min_velocity_, 0.01);
  
  // 姿态控制参数
  nh.param("orientation_stiffness", ds_impedance_params_.orientation_stiffness_, 50.0);
  nh.param("orientation_damping", ds_impedance_params_.orientation_damping_, 15.0);
  nh.param("max_orientation_torque", ds_impedance_params_.max_orientation_torque_, 15.0);
  
  // 接触力控制参数
  nh.param("desired_normal_force", ds_impedance_params_.desired_normal_force_, 5.0);
  nh.param("contact_surface_height", ds_impedance_params_.contact_surface_height_, 0.18);
  nh.param("filtered_force_gain", ds_impedance_params_.filtered_force_gain_, 0.9);
  nh.param("force_modulation_gain", ds_impedance_params_.force_modulation_gain_, 1.0);  // 添加gamma_p参数加载
  
  // 安全力矩限制
  nh.param("max_joint_torque", ds_impedance_params_.max_joint_torque_, 15.0);
  
  // 阻抗参数
  std::vector<double> pos_stiffness, ori_stiffness;
  if (nh.getParam("cartesian_stiffness_pos", pos_stiffness) && pos_stiffness.size() == 3) {
    ds_impedance_params_.cartesian_stiffness_pos_ = Eigen::Vector3d(pos_stiffness.data()).asDiagonal();
  } else {
    ds_impedance_params_.cartesian_stiffness_pos_ = 300.0 * Eigen::Matrix3d::Identity();
  }
  
  if (nh.getParam("cartesian_stiffness_ori", ori_stiffness) && ori_stiffness.size() == 3) {
    ds_impedance_params_.cartesian_stiffness_ori_ = Eigen::Vector3d(ori_stiffness.data()).asDiagonal();
  } else {
    ds_impedance_params_.cartesian_stiffness_ori_ = 30.0 * Eigen::Matrix3d::Identity();
  }
  
  // 计算临界阻尼
  ds_impedance_params_.cartesian_damping_pos_ = 2.0 * ds_impedance_params_.cartesian_stiffness_pos_.cwiseSqrt();
  ds_impedance_params_.cartesian_damping_ori_ = 2.0 * ds_impedance_params_.cartesian_stiffness_ori_.cwiseSqrt();
  
  // 零空间刚度
  nh.param("nullspace_stiffness", ds_impedance_params_.nullspace_stiffness_, 20.0);
  
  // 接触检测参数
  nh.param("contact_force_threshold", ds_impedance_params_.contact_force_threshold_, 0.5);
  nh.param("max_contact_force", ds_impedance_params_.max_contact_force_, 10.0);
  
  // 能量罐参数
  nh.param("energy_tank_max", ds_impedance_params_.energy_tank_max_, 4.0);
  
  // 安全参数
  nh.param("max_velocity", ds_impedance_params_.max_velocity_, 0.1);
  nh.param("max_acceleration", ds_impedance_params_.max_acceleration_, 0.5);
  nh.param("velocity_safety_factor", ds_impedance_params_.velocity_safety_factor_, 0.5);
  
  // 加速阶段参数
  nh.param("acceleration_duration", ds_impedance_params_.acceleration_duration_, 3.0);
  
  // 打印加载的关键参数用于调试
  ROS_INFO("Loaded DS-Impedance Parameters:");
  ROS_INFO_STREAM("  circular_radius: " << ds_impedance_params_.circular_radius_ << " m");
  // circular_omega 调试输出已删除
  ROS_INFO_STREAM("  constant_tangential_speed: " << ds_impedance_params_.constant_tangential_speed_ << " m/s");
  ROS_INFO_STREAM("  ds_impedance_gain: " << ds_impedance_params_.ds_impedance_gain_ << " N*s/m");
  ROS_INFO_STREAM("  contact_surface_height: " << ds_impedance_params_.contact_surface_height_ << " m");
  ROS_INFO_STREAM("  target_position: [" << ds_impedance_params_.target_position.transpose() << "] m");
  ROS_INFO_STREAM("  approach_gain: " << ds_impedance_params_.approach_gain_ << " 1/s");
  ROS_INFO_STREAM("  radial_gain: " << ds_impedance_params_.radial_gain_ << " 1/s");
  ROS_INFO_STREAM("  blend_distance: " << ds_impedance_params_.blend_distance_ << " m");
  
  return true;
}

void UnifiedDSController::initializeROSInterface(ros::NodeHandle& nh) {
  // 订阅用户命令
  sub_user_command_ = nh.subscribe("/unified_ds/user_command", 1, 
                                   &UnifiedDSController::userCommandCallback, this);
  
  // 订阅力传感器数据 - 修复话题名称匹配force_sensor_reader
  sub_force_data_ = nh.subscribe("/force_sensor/wrench", 1, 
                                 &UnifiedDSController::forceDataCallback, this);
  
  // 发布控制状态 - 修复话题名称匹配UI期望
  pub_control_status_ = nh.advertise<std_msgs::String>("/unified_ds/status", 1);
  pub_control_phase_ = nh.advertise<std_msgs::String>("/unified_ds/control_phase", 1);
  
  // 发布能量罐状态 - 修复话题名称匹配UI期望
  pub_energy_status_ = nh.advertise<std_msgs::Float64>("/unified_ds/energy_tank", 1);
  
  // 发布调试速度信息
  pub_velocity_debug_ = nh.advertise<geometry_msgs::Vector3>("/unified_ds/ds_velocity", 1);
}

void UnifiedDSController::userCommandCallback(const std_msgs::String::ConstPtr& msg) {
  if (msg->data == "start" && current_phase_ == ControlPhase::INITIALIZATION) {
    user_start_command_received_ = true;
    ROS_INFO("User command received: starting motion sequence");
  }
}

void UnifiedDSController::forceDataCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  // *** 修复：添加原版DS系统的力数据滤波机制 ***
  // 原始力数据
  Eigen::Vector3d raw_force(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
  
  // 原版DS低通滤波：_filteredWrench = _filteredForceGain*_filteredWrench+(1.0f-_filteredForceGain)*_wrench
  double filtered_force_gain = ds_impedance_params_.filtered_force_gain_;  // 滤波增益，0.9表示90%保留历史值，10%使用新值
  
  // 首次使用时初始化滤波值
  static bool first_force_data = true;
  if (first_force_data) {
    external_force_ = raw_force;
    first_force_data = false;
  } else {
    // 应用低通滤波
    external_force_ = filtered_force_gain * external_force_ + (1.0 - filtered_force_gain) * raw_force;
  }
  
  // 在校准阶段计算基线力（使用滤波后的数据）
  if (current_phase_ == ControlPhase::CALIBRATION) {
    baseline_force_z_ = external_force_(2);
    force_sensor_calibrated_ = true;
  }
  
  // 计算相对于基线的接触力（使用滤波后的数据）
  contact_force_ = external_force_;
  contact_force_(2) -= baseline_force_z_;
}

void UnifiedDSController::publishControlStatus() {
  // 发布简单的状态消息供UI使用
  std_msgs::String status_msg;
  std_msgs::String phase_msg;
  
  // 根据当前阶段发布对应的状态消息
  switch (current_phase_) {
    case ControlPhase::CALIBRATION:
      status_msg.data = "Initializing...";
      phase_msg.data = "CALIBRATION";
      break;
    case ControlPhase::INITIALIZATION:
      status_msg.data = "CALIBRATION_COMPLETE";  // UI期望的消息
      phase_msg.data = "INITIALIZATION";
      break;
    case ControlPhase::ACCELERATION:
      status_msg.data = "MOTION_STARTED";
      phase_msg.data = "ACCELERATION";
      break;
    case ControlPhase::LINEAR_APPROACH:
      status_msg.data = "MOTION_STARTED";
      phase_msg.data = "LINEAR_APPROACH";
      break;
    case ControlPhase::PROBE_DESCENT:
      status_msg.data = "MOTION_STARTED";
      phase_msg.data = "PROBE_DESCENT";
      break;
    case ControlPhase::CIRCULAR_MOTION:
      status_msg.data = "MOTION_STARTED";
      phase_msg.data = "CIRCULAR_MOTION";
      break;
    default:
      status_msg.data = "Unknown";
      phase_msg.data = "UNKNOWN";
      break;
  }
  
  // 发布状态消息
  pub_control_status_.publish(status_msg);
  pub_control_phase_.publish(phase_msg);
}

void UnifiedDSController::publishEnergyStatus() {
  std_msgs::Float64 energy_msg;
  energy_msg.data = energy_tank_manager_->getCurrentEnergy();
  pub_energy_status_.publish(energy_msg);
}

} // namespace franka_ds

// 注册控制器插件
PLUGINLIB_EXPORT_CLASS(franka_ds::UnifiedDSController, controller_interface::ControllerBase) 