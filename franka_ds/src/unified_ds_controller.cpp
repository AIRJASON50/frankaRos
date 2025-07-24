// Copyright (c) 2024 Franka DS Project
// Use of this source code is governed by the Apache-2.0 license

#include <franka_ds/unified_ds_controller.h>
#include <franka_ds/ds_utils.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

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
  
  // 设置目标位置 - 使用contact_control的正确目标位置
  target_position_ << 0.483835, 0.089474, 0.193932;
  target_orientation_ = initial_orientation_; // 保持初始姿态
  
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
  
  // 获取当前末端执行器状态 - 直接使用Franka接口
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());
  
  // 计算当前笛卡尔速度 - 直接使用Franka提供的数据
  Vector6d cartesian_velocity = jacobian * dq;
  
  // 更新控制阶段
  updateControlPhase(time.toSec(), position, external_force_);
  
  // 计算DS调制的期望速度 - 使用当前位置
  Eigen::Vector3d nominal_ds_velocity = computeDSModulatedVelocity(position);
  
  // 使用能量罐进行无源性约束
  Eigen::Vector3d constrained_velocity = nominal_ds_velocity;
  if (current_phase_ != ControlPhase::CALIBRATION && current_phase_ != ControlPhase::INITIALIZATION) {
    // 更新能量罐动力学
    Eigen::Matrix3d damping_matrix = ds_impedance_params_.cartesian_damping_pos_;
    double desired_force = 0.0;  // 自由空间运动时期望力为0
    Eigen::Vector3d surface_normal = Eigen::Vector3d::UnitZ();  // 默认表面法向量
    
    energy_tank_manager_->updateTankDynamics(cartesian_velocity.head(3), 
                                            nominal_ds_velocity,
                                            damping_matrix,
                                            desired_force,
                                            surface_normal,
                                            period.toSec());
    
    // 应用无源性约束
    constrained_velocity = energy_tank_manager_->constrainVelocity(nominal_ds_velocity);
  }
  
  // 计算阻抗控制力矩
  Vector7d tau_d = computeImpedanceControl(constrained_velocity, transform, cartesian_velocity, jacobian);
  
  // 添加重力补偿和科里奥利力补偿
  tau_d += coriolis;
  
  // 限制力矩变化率
  tau_d = saturateTorqueRate(tau_d, tau_J_d);
  
  // 添加安全力矩限制，防止触发机器人安全反射
  const double MAX_JOINT_TORQUE = 15.0;  // 每个关节的最大安全力矩 [Nm]
  for (size_t i = 0; i < 7; ++i) {
    tau_d(i) = std::max(-MAX_JOINT_TORQUE, std::min(MAX_JOINT_TORQUE, tau_d(i)));
  }
  
  // 发送力矩命令到关节
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
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
    ROS_INFO_STREAM("Position: [" << position.transpose() << "]");
    ROS_INFO_STREAM("Target: [" << target_position_.transpose() << "]");
    ROS_INFO_STREAM("Distance to target: " << (target_position_ - position).norm() << " m");
    ROS_INFO_STREAM("Nominal DS velocity: [" << nominal_ds_velocity.transpose() << "], norm: " << nominal_ds_velocity.norm());
    ROS_INFO_STREAM("Constrained velocity: [" << constrained_velocity.transpose() << "], norm: " << constrained_velocity.norm());
    ROS_INFO_STREAM("Current velocity: [" << cartesian_velocity.head(3).transpose() << "], norm: " << cartesian_velocity.head(3).norm());
    ROS_INFO_STREAM("Torque command norm: " << tau_d.norm() << " Nm");
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
      Eigen::Vector3d circular_velocity = computeCircularDS(current_position);
      return progress * circular_velocity;
    }
    
    case ControlPhase::LINEAR_APPROACH: {
      // 此阶段现在被跳过，直接在加速后进入圆周运动
      current_phase_ = ControlPhase::CIRCULAR_MOTION;
      phase_start_time_ = ros::Time::now();
      ROS_INFO("Switching to CIRCULAR_MOTION phase");
      return computeCircularDS(current_position);
    }
    
    case ControlPhase::PROBE_DESCENT: {
      // 此阶段现在被跳过，直接进入圆周运动
      current_phase_ = ControlPhase::CIRCULAR_MOTION;
      phase_start_time_ = ros::Time::now();
      ROS_INFO("Switching to CIRCULAR_MOTION phase");
      return computeCircularDS(current_position);
    }
    
    case ControlPhase::CIRCULAR_MOTION: {
      return computeCircularDS(current_position);
    }
    
    default:
      return Eigen::Vector3d::Zero();
  }
}

Vector7d UnifiedDSController::computeImpedanceControl(const Eigen::Vector3d& desired_velocity,
                                                      const Eigen::Affine3d& transform,
                                                      const Eigen::Matrix<double, 6, 1>& current_velocity,
                                                      const Eigen::Matrix<double, 6, 7>& jacobian) {
  // DS-阻抗控制律 (论文方程3): Fc = d1*f(x) + d1*fn(x) - D(x)*x_dot
  // 这是纯速度控制，不依赖位置误差！
  
  // 获取当前线性和角速度
  Eigen::Vector3d current_linear_velocity = current_velocity.head(3);
  Eigen::Vector3d current_angular_velocity = current_velocity.tail(3);
  
  // 计算DS阻抗增益 d1 (根据force_based_ds_modulation设置)
  double d1 = 150.0;  // 降低DS-阻抗控制器增益，确保力矩在安全范围内
  
  // 计算阻尼矩阵 D(x) - 简化为对角矩阵
  Eigen::Matrix3d damping_matrix = Eigen::Matrix3d::Identity() * 30.0;  // 降低阻尼
  
  // DS控制律：线性部分
  // 第一项: d1 * f(x) - 沿期望DS的驱动力
  Eigen::Vector3d ds_driving_force = d1 * desired_velocity;
  
  // 第二项: d1 * fn(x) - 法向力调制项（接触时生成法向力）
  Eigen::Vector3d normal_force_term = Eigen::Vector3d::Zero();
  if (current_phase_ == ControlPhase::CIRCULAR_MOTION && external_force_.norm() > ds_impedance_params_.contact_force_threshold_) {
    // 计算表面法向量（简化：假设垂直向上）
    Eigen::Vector3d surface_normal(0.0, 0.0, 1.0);
    double desired_normal_force = 5.0;  // 期望接触力 [N]
    normal_force_term = d1 * (desired_normal_force / d1) * surface_normal;
  }
  
  // 第三项: -D(x) * x_dot - 阻尼力
  Eigen::Vector3d damping_force = -damping_matrix * current_linear_velocity;
  
  // 合成笛卡尔控制力
  Eigen::Vector3d cartesian_force = ds_driving_force + normal_force_term + damping_force;
  
  // 姿态控制 - 强制保持末端执行器垂直（参考force_based_ds_modulation）
  Eigen::Quaterniond current_orientation(transform.rotation());
  
  // 目标姿态：末端执行器Z轴垂直向下
  Eigen::Vector3d desired_z_axis(0.0, 0.0, -1.0);  // 垂直向下
  Eigen::Vector3d current_z_axis = transform.rotation().col(2);
  
  // 计算旋转误差（使用Rodrigues公式，参考force_based_ds_modulation）
  Eigen::Vector3d rotation_axis = current_z_axis.cross(desired_z_axis);
  double rotation_angle = acos(std::max(-1.0, std::min(1.0, current_z_axis.dot(desired_z_axis))));
  
  Eigen::Vector3d orientation_error = Eigen::Vector3d::Zero();
  if (rotation_axis.norm() > 1e-6) {
    rotation_axis.normalize();
    orientation_error = rotation_angle * rotation_axis;
  }
  
  // 调整姿态控制增益（降低以减少抖动）
  double orientation_stiffness = 50.0;  // 降低刚度减少抖动
  double orientation_damping = 15.0;    // 适当的阻尼
  
  // 添加死区以减少小幅抖动
  if (orientation_error.norm() < 0.01) {  // 小于0.01弧度的误差忽略
    orientation_error.setZero();
  }
  
  // 姿态控制力矩（添加限制以防止过大的力矩）
  Eigen::Vector3d cartesian_torque = orientation_stiffness * orientation_error - 
                                     orientation_damping * current_angular_velocity;
  
  // 限制最大力矩以防止剧烈运动
  double max_torque = 10.0;  // 最大力矩限制
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
  
  // 计算零空间投影矩阵 - 简化版本避免维度错误
  // 使用伪逆计算：J# = J^T * (J*J^T)^(-1)
  Eigen::Matrix<double, 6, 6> lambda = (jacobian * jacobian.transpose());
  
  // 检查矩阵是否可逆
  if (lambda.determinant() < 1e-6) {
    // 如果接近奇异，使用正则化
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
  // 增强版复合DS：强恢复能力 + 完整圆周运动
  // 使用contact_control的目标位置作为吸引子
  static Eigen::Vector3d attractor(0.483835, 0.089474, 0.193932);  // contact_control的目标点
  
  // 计算相对位置和距离
  Eigen::Vector3d relative_position = current_position - attractor;
  double distance_to_attractor = relative_position.norm();
  
  // 圆周运动参数
  double target_radius = ds_impedance_params_.circular_radius_;  // 0.05m
  double omega = 2.0 * M_PI;  // 增加角频率，使圆周运动更明显
  
  // === 1. 强力接近速度（恢复能力）===
  Eigen::Vector3d approach_velocity = Eigen::Vector3d::Zero();
  if (distance_to_attractor > 1e-6) {
    // 使用强力接近速度，确保能拉回机器人
    double approach_gain = 2.0;  // 接近增益
    approach_velocity = -approach_gain * relative_position;  // 指向吸引子
    
    // 限制接近速度幅值
    double max_approach_speed = ds_impedance_params_.linear_max_velocity_;
    if (approach_velocity.norm() > max_approach_speed) {
      approach_velocity = approach_velocity.normalized() * max_approach_speed;
    }
  }
  
  // === 2. 纯圆周运动速度 ===
  Eigen::Vector3d circular_velocity = Eigen::Vector3d::Zero();
  
  // 水平面圆周运动（X-Y平面）
  double R_xy = sqrt(relative_position(0) * relative_position(0) + 
                     relative_position(1) * relative_position(1));
  
  if (R_xy > 1e-6) {
    // 径向分量：收缩到目标半径
    double radial_gain = 1.0;
    double radial_velocity = -radial_gain * (R_xy - target_radius);
    
    // 切向分量：圆周运动
    double tangential_velocity = omega * R_xy;
    
    // 转换为笛卡尔坐标
    double cos_theta = relative_position(0) / R_xy;
    double sin_theta = relative_position(1) / R_xy;
    
    circular_velocity(0) = radial_velocity * cos_theta - tangential_velocity * sin_theta;
    circular_velocity(1) = radial_velocity * sin_theta + tangential_velocity * cos_theta;
  }
  
  // Z方向：稳定在目标高度
  circular_velocity(2) = -2.0 * relative_position(2);  // 强力高度控制
  
  // === 3. 距离自适应混合 ===
  double blend_distance = 0.1;  // 混合距离阈值 [m]
  double blend_ratio;
  
  if (distance_to_attractor > blend_distance) {
    // 远离时：主要是接近运动
    blend_ratio = 0.1;  // 10%圆周，90%接近
  } else {
    // 接近时：主要是圆周运动
    double normalized_distance = distance_to_attractor / blend_distance;
    blend_ratio = 0.9 * (1.0 - normalized_distance) + 0.1;  // 逐渐增加圆周比例
  }
  
  // === 4. 最终混合速度 ===
  Eigen::Vector3d final_velocity = (1.0 - blend_ratio) * approach_velocity + 
                                   blend_ratio * circular_velocity;
  
  // === 5. 速度限制和归一化 ===
  double velocity_limit = ds_impedance_params_.max_velocity_;
  if (final_velocity.norm() > velocity_limit) {
    final_velocity = final_velocity.normalized() * velocity_limit;
  }
  
  // 确保最小速度，避免停滞
  double min_velocity = 0.01;  // 最小速度 [m/s]
  if (final_velocity.norm() < min_velocity && distance_to_attractor > 1e-3) {
    final_velocity = final_velocity.normalized() * min_velocity;
  }
  
  return final_velocity;
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
    ds_impedance_params_.target_position << 0.5, 0.0, 0.3;
    ROS_WARN("Using default target position: [0.5, 0.0, 0.3]");
  }
  
  // DS参数
  nh.param("linear_lambda", ds_impedance_params_.linear_lambda_, 2.0);
  nh.param("linear_max_velocity", ds_impedance_params_.linear_max_velocity_, 0.05);
  nh.param("circular_radius", ds_impedance_params_.circular_radius_, 0.05);
  nh.param("circular_omega", ds_impedance_params_.circular_omega_, 0.5);
  nh.param("exploration_speed", ds_impedance_params_.exploration_speed_, 0.01);
  
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
  external_force_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
  
  // 在校准阶段计算基线力
  if (current_phase_ == ControlPhase::CALIBRATION) {
    baseline_force_z_ = external_force_(2);
    force_sensor_calibrated_ = true;
  }
  
  // 计算相对于基线的接触力
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