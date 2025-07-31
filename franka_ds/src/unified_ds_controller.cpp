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
  
  // 初始化ROS接口
  initializeROSInterface(controller_nh);
  ROS_INFO("ROS interface initialized");
  
  // 初始化控制状态 - 简化为两个状态
  is_calibrated_ = false;
  motion_started_ = false;
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
  
  // 设置目标位置
  target_position_ = ds_impedance_params_.target_position;
  target_orientation_ = initial_orientation_;
  
  // 初始化状态
  is_calibrated_ = false;
  motion_started_ = false;
  controller_start_time_ = time;
  
  ROS_INFO("DS-Impedance Controller ready, starting calibration...");
  ROS_INFO_STREAM("Initial position: [" << initial_position_.transpose() << "], target position: [" 
                  << target_position_.transpose() << "], distance: " << (target_position_ - initial_position_).norm() << " m");
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
  
  // 获取当前末端执行器状态 - 使用预分配的缓存变量
  transform_cache_ = Eigen::Affine3d(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  position_cache_ = transform_cache_.translation();
  orientation_cache_ = Eigen::Quaterniond(transform_cache_.rotation());
  
  // 计算当前笛卡尔速度
  cartesian_velocity_cache_ = jacobian * dq;
  
  // 简化的状态管理
  updateSimpleControlState(time.toSec(), position_cache_, external_force_);
  
  // 计算期望速度
  if (!is_calibrated_ || !motion_started_) {
    // 校准阶段或等待启动：零速度
    nominal_ds_velocity_cache_ = Eigen::Vector3d::Zero();
  } else {
    // 运动阶段：统一速度场（接近场 + 力调制）
    nominal_ds_velocity_cache_ = computeUnifiedDS(position_cache_);
  }
  
  // 能量罐管理和无源性约束
  constrained_velocity_cache_ = nominal_ds_velocity_cache_;
  if (is_calibrated_ && motion_started_) {
    // 计算la (调制增益) - 参照 SurfacePolishing.cpp
    double d1 = ds_impedance_params_.ds_impedance_gain_;
    double gammap = ds_impedance_params_.force_modulation_gain_;
    double desired_force = computeDesiredContactForce(position_cache_);
    Eigen::Vector3d surface_normal(0.0, 0.0, 1.0);

    double delta_val = std::pow(2.0 * surface_normal.dot(nominal_ds_velocity_cache_) * gammap * desired_force / d1, 2.0) +
                       4.0 * std::pow(nominal_ds_velocity_cache_.norm(), 4.0);
    
    double la;
    if (std::abs(nominal_ds_velocity_cache_.norm()) < 1e-6) {
      la = 0.0;
    } else {
      la = (-2.0 * surface_normal.dot(nominal_ds_velocity_cache_) * gammap * desired_force / d1 + std::sqrt(delta_val)) /
           (2.0 * std::pow(nominal_ds_velocity_cache_.norm(), 2.0));
    }

    // 能量罐动力学更新
    Eigen::Matrix3d damping_matrix = ds_impedance_params_.cartesian_damping_pos_;
    energy_tank_manager_->updateTankDynamics(cartesian_velocity_cache_.head(3), 
                                            nominal_ds_velocity_cache_,
                                            damping_matrix,
                                            desired_force,
                                            surface_normal,
                                            la,
                                            period.toSec());
    
    // 应用无源性约束
    constrained_velocity_cache_ = energy_tank_manager_->constrainVelocity(nominal_ds_velocity_cache_);
  }
  
  // 计算阻抗控制力矩
  tau_d_cache_ = computeImpedanceControl(constrained_velocity_cache_,
                                         transform_cache_,
                                         cartesian_velocity_cache_,
                                         jacobian);
  
  // 添加重力补偿和科里奥利力补偿
  tau_d_cache_ += coriolis;
  
  // 限制力矩变化率
  tau_d_cache_ = saturateTorqueRate(tau_d_cache_, tau_J_d);
  
  // 安全力矩限制
  const double MAX_JOINT_TORQUE = ds_impedance_params_.max_joint_torque_;
  for (size_t i = 0; i < 7; ++i) {
    tau_d_cache_(i) = std::max(-MAX_JOINT_TORQUE, std::min(MAX_JOINT_TORQUE, tau_d_cache_(i)));
  }
  
  // 发送力矩命令到关节
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_cache_(i));
  }
  
  // 发布状态信息 (修复：改回200Hz中的500次打印，即约2Hz频率)
  if (debug_print_counter_++ % 500 == 0) {
    publishControlStatus();
    publishEnergyStatus();
    
    // 简化的状态输出（2Hz频率）
    if (!is_calibrated_) {
      ROS_INFO("=== Force Sensor Calibration in Progress ===");
    } else if (!motion_started_) {
      ROS_INFO("=== Ready to Start - Waiting for User Command ===");
      ROS_INFO_STREAM("Calibration complete, current position: [" << position_cache_.transpose() << "]");
    } else {
      // 只在运动状态时显示详细信息
      ROS_INFO_STREAM("=== Motion Active ===");
      ROS_INFO_STREAM("Position: [" << position_cache_.transpose() << "]");
      ROS_INFO_STREAM("Target: [" << target_position_.transpose() << "]");
      ROS_INFO_STREAM("Distance: " << (target_position_ - position_cache_).norm() << " m");
      ROS_INFO_STREAM("DS velocity norm: " << nominal_ds_velocity_cache_.norm() << " m/s");
      ROS_INFO_STREAM("DS velocity: [" << nominal_ds_velocity_cache_.transpose() << "] m/s");
      ROS_INFO_STREAM("Actual velocity norm: " << cartesian_velocity_cache_.head(3).norm() << " m/s");
      ROS_INFO_STREAM("Energy tank: " << energy_tank_manager_->getCurrentEnergy() << "/" << ds_impedance_params_.energy_tank_max_ << " J");
      ROS_INFO_STREAM("===================");
    }
  }
}

void UnifiedDSController::stopping(const ros::Time& time) {
  ROS_WARN("Unified DS-Impedance Controller stopping");
  
  // 发送零力矩命令
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(0.0);
  }
}

// 新的统一速度场计算函数
Eigen::Vector3d UnifiedDSController::computeUnifiedDS(const Eigen::Vector3d& current_position) {
  // 统一速度场 = 接近场 + 力调制
  // 不再区分圆周运动和线性接近，而是根据位置自动混合
  
  // 1. 计算基础接近/圆周混合速度场
  Eigen::Vector3d base_velocity = computeBaseVelocityField(current_position);
  
  // 2. 计算力调制项
  Eigen::Vector3d force_modulation = computeForceModulation(current_position);
  
  // 3. 统一速度场：基础速度场 + 力调制
  Eigen::Vector3d unified_velocity = base_velocity + force_modulation;
  
  // 4. 速度限制
  double velocity_limit = ds_impedance_params_.max_velocity_;
  if (unified_velocity.norm() > velocity_limit) {
    unified_velocity = unified_velocity.normalized() * velocity_limit;
  }
  
  // 5. 确保最小速度，避免停滞
  double min_velocity = ds_impedance_params_.min_velocity_;
  double distance_to_target = (target_position_ - current_position).norm();
  if (unified_velocity.norm() < min_velocity && distance_to_target > 1e-3) {
    unified_velocity = unified_velocity.normalized() * min_velocity;
  }
  
  return unified_velocity;
}

// 基础速度场：接近场与圆周场的自动混合
Eigen::Vector3d UnifiedDSController::computeBaseVelocityField(const Eigen::Vector3d& current_position) {
  // 目标位置：使用配置的吸引子位置，但Z坐标使用接触面高度
  Eigen::Vector3d attractor = ds_impedance_params_.target_position;
  attractor(2) = ds_impedance_params_.contact_surface_height_;
  
  Eigen::Vector3d relative_position = current_position - attractor;
  double distance_to_attractor = relative_position.norm();
  
  if (distance_to_attractor < 1e-6) {
    return Eigen::Vector3d::Zero();
  }
  
  // 计算水平距离和高度误差
  double horizontal_distance = sqrt(relative_position(0) * relative_position(0) +
                                   relative_position(1) * relative_position(1));
  double height_error = std::abs(relative_position(2));
  double target_radius = ds_impedance_params_.circular_radius_;
  
  // === 改进：基于旋转矩阵的DS混合策略（参考SurfacePolishing.cpp:465-481） ===
  
  // 1. 计算接近速度方向（指向吸引子的直线接近）
  // 修复：v0应该是从当前位置指向吸引子的速度，而不是简单的法向速度
  Eigen::Vector3d approach_direction = -relative_position.normalized();  // 指向吸引子
  Eigen::Vector3d v0 = ds_impedance_params_.linear_max_velocity_ * approach_direction;  // 接近速度
  
  // 2. 计算圆周运动速度（在表面上的投影）
  Eigen::Vector3d circular_velocity_raw = computeCircularVelocity(relative_position, attractor);
  
  // 3. 将圆周运动投影到表面上（去除法向分量）
  Eigen::Vector3d surface_normal(0.0, 0.0, 1.0);  // 表面法向量
  Eigen::Matrix3d surface_projection = Eigen::Matrix3d::Identity() - surface_normal * surface_normal.transpose();
  Eigen::Vector3d vd_contact = surface_projection * circular_velocity_raw;
  
  // 4. 保持圆周速度的原始幅值，不要标准化
  if (vd_contact.norm() < 1e-6) {
    // 如果圆周速度为零，使用默认切向方向和切向速度
    vd_contact = Eigen::Vector3d(ds_impedance_params_.constant_tangential_speed_, 0.0, 0.0);
  }
  
  // 5. 计算混合角度：基于竖直方向距离（到接触面的距离）
  // 参考SurfacePolishing.cpp第467行: theta = (1.0f-std::tanh(10*_normalDistance))*angle
  double normal_distance = std::max(0.0, height_error);  // 竖直方向距离到接触面
  double angle = std::acos(std::max(-1.0, std::min(1.0, v0.normalized().dot(vd_contact.normalized()))));
  double theta = (1.0 - std::tanh(10.0 * normal_distance)) * angle;  // 参考SurfacePolishing.cpp
  
  // 6. 计算旋转轴
  Eigen::Vector3d rotation_axis = v0.normalized().cross(vd_contact.normalized());
  
  // 7. 构建旋转矩阵并应用混合
  Eigen::Vector3d final_velocity;
  
  if (rotation_axis.norm() < 1e-6) {
    // 如果没有旋转轴（向量平行），直接使用接近速度
    final_velocity = v0;
  } else {
    // 标准化旋转轴
    rotation_axis.normalize();
    
    // 使用Rodrigues旋转公式构建旋转矩阵
    // R = I + sin(theta)*K + (1-cos(theta))*K^2
    Eigen::Matrix3d K;
    K << 0.0, -rotation_axis(2), rotation_axis(1),
         rotation_axis(2), 0.0, -rotation_axis(0),
         -rotation_axis(1), rotation_axis(0), 0.0;
    
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + 
                        std::sin(theta) * K + 
                        (1.0 - std::cos(theta)) * K * K;
    
    // 应用旋转矩阵到接近速度的幅值，但旋转到圆周方向
    Eigen::Vector3d v0_magnitude_vd_direction = v0.norm() * vd_contact.normalized();
    final_velocity = R * v0;
  }
  
  // 8. Z方向独立控制：仅用于高度快速收敛，不影响XY平面运动
  // 这里只处理XY平面的混合，Z方向交给接触场处理
  Eigen::Vector3d xy_velocity = final_velocity;

  
  // 调试输出（每2秒输出一次）
  static int debug_counter = 0;
  if (++debug_counter % 1000 == 0) {
    ROS_INFO_STREAM("=== Base Velocity Field Debug ===");
    ROS_INFO_STREAM("Horizontal distance: " << horizontal_distance << " m");
    ROS_INFO_STREAM("Normal distance (height_error): " << height_error << " m");
    ROS_INFO_STREAM("Target radius: " << target_radius << " m");
    ROS_INFO_STREAM("Blend factor: " << normal_distance); // 使用normal_distance
    ROS_INFO_STREAM("Theta: " << theta << " rad");
    ROS_INFO_STREAM("Tangential speed from YAML: " << ds_impedance_params_.constant_tangential_speed_ << " m/s");
    ROS_INFO_STREAM("Approach velocity norm: " << v0.norm() << " m/s");
    ROS_INFO_STREAM("Circular velocity norm: " << vd_contact.norm() << " m/s");
    ROS_INFO_STREAM("Final XY velocity norm: " << xy_velocity.norm() << " m/s");
    ROS_INFO_STREAM("============================");
  }
  
  return xy_velocity;
}

// 接近速度场
Eigen::Vector3d UnifiedDSController::computeApproachVelocity(const Eigen::Vector3d& relative_position) {
  Eigen::Vector3d approach_velocity;
  
  // XY方向：线性接近
  double approach_gain_xy = ds_impedance_params_.approach_gain_;
  approach_velocity(0) = -approach_gain_xy * relative_position(0);
  approach_velocity(1) = -approach_gain_xy * relative_position(1);
  
  // Z方向：强化接近
  double approach_gain_z = 3.0 * ds_impedance_params_.approach_gain_;
  approach_velocity(2) = -approach_gain_z * relative_position(2);
  
  // 限制接近速度
  double max_approach_speed = ds_impedance_params_.linear_max_velocity_;
  if (approach_velocity.norm() > max_approach_speed) {
    approach_velocity = approach_velocity.normalized() * max_approach_speed;
  }
  
  return approach_velocity;
}

// 圆周速度场
Eigen::Vector3d UnifiedDSController::computeCircularVelocity(const Eigen::Vector3d& relative_position, 
                                                             const Eigen::Vector3d& attractor) {
  // === 参考SurfacePolishing.cpp:494-512的getCircularMotionVelocity实现 ===
  
  Eigen::Vector3d circular_velocity = Eigen::Vector3d::Zero();
  
  // 计算相对于吸引子的位置
  Eigen::Vector3d position_relative_to_attractor = relative_position;
  
  // Z方向：收敛到接触面（这部分将被接触场覆盖，这里保持简单）
  circular_velocity(2) = 0;
  
  // XY平面圆周运动：参考SurfacePolishing.cpp的实现
  double R_xy = sqrt(position_relative_to_attractor(0) * position_relative_to_attractor(0) + 
                     position_relative_to_attractor(1) * position_relative_to_attractor(1));
  
  if (R_xy > 1e-6) {
    // 计算极坐标
    double cos_theta = position_relative_to_attractor(0) / R_xy;
    double sin_theta = position_relative_to_attractor(1) / R_xy;
    
    // 获取圆周运动参数
    double target_radius = ds_impedance_params_.circular_radius_;  // 对应SurfacePolishing中的r
    double tangential_speed = ds_impedance_params_.constant_tangential_speed_;  // 使用YAML中的切向速度
    
    // 计算角速度：omega = v / r，其中v是切向速度，r是当前半径
    double omega = tangential_speed/target_radius;  // 直接使用切向速度作为圆周运动的驱动
    
    // 参考SurfacePolishing.cpp:508-509的公式实现
    // velocity(0) = -(R-r) * cos(T) - R * omega * sin(T);
    // velocity(1) = -(R-r) * sin(T) + R * omega * cos(T);
    
    circular_velocity(0) = -(R_xy - target_radius) * cos_theta - R_xy * omega * sin_theta;
    circular_velocity(1) = -(R_xy - target_radius) * sin_theta + R_xy * omega * cos_theta;
    
    // 这样实现的圆周运动特性：
    // 1. 径向分量：-(R_xy - target_radius) * [cos_theta, sin_theta] 
    //    当R_xy > target_radius时向内收缩，当R_xy < target_radius时向外扩张
    // 2. 切向分量：-R_xy * omega * [sin_theta, -cos_theta]  
    //    提供恒定角速度omega的圆周运动
    // 3. 当R_xy = target_radius时，只有切向运动，实现匀速圆周
  }
  
  return circular_velocity;
}

// 简化的状态更新
void UnifiedDSController::updateSimpleControlState(double current_time, 
                                                   const Eigen::Vector3d& current_position, 
                                                   const Eigen::Vector3d& external_force) {
  if (!is_calibrated_) {
    // 校准阶段：等待力传感器校准完成
    // 这里假设在forceDataCallback中已经设置了is_calibrated_标志
    return;
  }
  
  // 校准完成，等待用户启动命令
  // motion_started_标志在userCommandCallback中设置
}

Vector7d UnifiedDSController::computeImpedanceControl(const Eigen::Vector3d& desired_velocity,
                                                     const Eigen::Affine3d& transform,
                                                     const Eigen::Matrix<double, 6, 1>& current_velocity,
                                                     const Eigen::Matrix<double, 6, 7>& jacobian) {
  // DS-阻抗控制律：统一速度场跟踪
  // 公式：Fc = d1 * x˙d - D(x) * x˙
  
  // 获取当前线性和角速度
  Eigen::Vector3d current_linear_velocity = current_velocity.head(3);
  Eigen::Vector3d current_angular_velocity = current_velocity.tail(3);
  
  // DS阻抗增益 d1
  double d1 = ds_impedance_params_.ds_impedance_gain_;
  
  // 计算阻尼矩阵 D(x)
  Eigen::Matrix3d damping_matrix = Eigen::Matrix3d::Identity() * ds_impedance_params_.ds_damping_gain_;
  
  // DS控制律：线性部分
  Eigen::Vector3d ds_driving_force = d1 * desired_velocity;  // d1 * (接近场 + 力调制)
  Eigen::Vector3d damping_force = -damping_matrix * current_linear_velocity;  // -D(x) * x˙
  
  // 合成笛卡尔控制力
  Eigen::Vector3d cartesian_force = ds_driving_force + damping_force;
  
  // 姿态控制 - 保持末端执行器垂直
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
  
  // 零空间控制
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

// 旧的函数已删除，现在使用统一的速度场计算

Eigen::Vector3d UnifiedDSController::computeForceModulation(const Eigen::Vector3d& current_position) {
  // *** 接触场：专门用于生成接触力的Z轴速度 ***
  // 按照用户设想：2.2 接触场是用来生成接触力的z轴速度，在没有接触之前都是0，靠近接触面时慢速增大
  
  // 1. 计算表面法向量（简化版：垂直向上，接触面为水平面）
  Eigen::Vector3d surface_normal(0.0, 0.0, 1.0);  // 表面法向量垂直向上
  
  // 2. 计算期望接触力 Fd(x)（动态调整，参考原版DS）
  double desired_force = computeDesiredContactForce(current_position);
  
  // 3. 获取DS阻抗增益 d1
  double d1 = ds_impedance_params_.ds_impedance_gain_;
  
  // 4. 计算到接触面的距离，用于判断是否生成接触场
  double distance_to_surface = current_position(2) - ds_impedance_params_.contact_surface_height_;
  
  // 5. 接触场的核心逻辑：只在接近接触面时生成Z轴速度
  Eigen::Vector3d contact_field = Eigen::Vector3d::Zero();
  
  // 接触场的生成条件：
  // - 在接触面上方一定范围内才开始生成
  // - 距离越近，力越大，但永远不为零（持续生成力）
  double contact_activation_distance = ds_impedance_params_.blend_distance_;  // 激活距离
  
  if (distance_to_surface > 0.0 && distance_to_surface < contact_activation_distance) {
    // 在激活范围内：生成平滑增长的接触场
    
    // 使用平滑函数计算接触场强度，距离越近强度越大
    // 参考SurfacePolishing.cpp中力的平滑生成
    double contact_field_strength = 0.5 * (1.0 - std::tanh(5.0 * distance_to_surface / contact_activation_distance));
    
    // 接触场只在Z方向生成速度，用于产生接触力
    // 公式：fn(x) = Fd(x)/d1 * n(x) * 接触场强度
    contact_field = (desired_force / d1) * surface_normal * contact_field_strength;
    
  } else if (distance_to_surface <= 0.0) {
    // 已经在接触面下方：生成完整的接触力
    contact_field = (desired_force / d1) * surface_normal;
  }
  // else: 距离太远时，接触场为零（符合设想：没有接触之前都是0）
  
  return contact_field;
}

double UnifiedDSController::computeDesiredContactForce(const Eigen::Vector3d& current_position) {
  // 动态力生成逻辑（参考原版DS SurfacePolishing.cpp:505-516）
  // 改进：使用平滑函数生成连续的期望力
  
  // 计算到接触面的距离
  double distance_to_surface = current_position(2) - ds_impedance_params_.contact_surface_height_;
  double blend_distance = ds_impedance_params_.blend_distance_; // 使用混合距离作为平滑过渡范围
  
  // 定义期望力和接触阈值
  double desired_force_max = ds_impedance_params_.desired_normal_force_;
  double contact_threshold_force = ds_impedance_params_.contact_force_threshold_;

  // 计算力生成因子：当靠近接触面时，力从0平滑增加到desired_force_max
  // 使用 sigmoid 或 tanh 函数实现平滑过渡
  // 假设力从 contact_surface_height_ + blend_distance 开始平滑增加
  // 当距离为 contact_surface_height_ 时达到 desired_force_max
  
  // 使用 tanh 函数实现平滑的力生成，将距离映射到期望力
  // 调整 tanh 输入，使其在接触面附近平滑过渡
  // 当 distance_to_surface 远大于 0 时，force_factor 接近 0
  // 当 distance_to_surface 远小于 0 时，force_factor 接近 1
  double force_factor = 0.5 * (1.0 - std::tanh(10.0 * (distance_to_surface / blend_distance)));
  
  // 确保 force_factor 在0到1之间
  force_factor = std::max(0.0, std::min(1.0, force_factor));

  // 计算最终期望力
  double final_desired_force = desired_force_max * force_factor;

  // 额外考虑：如果当前外部力很小且机器人已在接触面下方，保持一个最小力
  // 避免在轻微接触时期望力突然变为0，导致脱离
  if (current_position(2) < ds_impedance_params_.contact_surface_height_ && external_force_.norm() < contact_threshold_force) {
      final_desired_force = std::max(final_desired_force, 0.5); // 保持一个最小力，例如0.5N
  }

  return final_desired_force;
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

// 旧的阶段控制函数已删除，现在使用简化的状态管理

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
  // nh.param("linear_lambda", ds_impedance_params_.linear_lambda_, 2.0); // 已删除
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
  // nh.param("max_contact_force", ds_impedance_params_.max_contact_force_, 10.0); // 已删除
  
  // 能量罐参数
  nh.param("energy_tank_max", ds_impedance_params_.energy_tank_max_, 4.0);
  
  // 安全参数
  nh.param("max_velocity", ds_impedance_params_.max_velocity_, 0.1);
  // nh.param("max_acceleration", ds_impedance_params_.max_acceleration_, 0.5); // 已删除
  // nh.param("velocity_safety_factor", ds_impedance_params_.velocity_safety_factor_, 0.5); // 已删除
  
  // 加速阶段参数
  // 加速持续时间参数已删除，现在使用统一速度场
  
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
  ROS_INFO_STREAM("=== Received user command: '" << msg->data << "' ===");
  ROS_INFO_STREAM("Current state: is_calibrated_=" << (is_calibrated_ ? "true" : "false") 
                  << ", motion_started_=" << (motion_started_ ? "true" : "false"));
  
  if (msg->data == "start" && is_calibrated_ && !motion_started_) {
    motion_started_ = true;
    ROS_INFO("=== USER COMMAND: MOTION STARTED ===");
    ROS_INFO("Successfully switched from calibration to motion phase");
    ROS_INFO_STREAM("Current position: [" << position_cache_.transpose() << "]");
    ROS_INFO_STREAM("Target position: [" << target_position_.transpose() << "]");
    ROS_INFO_STREAM("Distance to target: " << (target_position_ - position_cache_).norm() << " m");
    ROS_INFO("Unified velocity field (approach + circular + force modulation) activated");
  } else if (msg->data == "start" && !is_calibrated_) {
    ROS_WARN("Cannot start motion: Force sensor calibration not completed yet");
    ROS_WARN_STREAM("Current force magnitude: " << external_force_.norm() << " N");
  } else if (msg->data == "start" && motion_started_) {
    ROS_INFO("Motion already started, ignoring duplicate start command");
  } else {
    ROS_WARN_STREAM("Unknown command or invalid state for command: '" << msg->data << "'");
  }
  
  ROS_INFO("=== Command processing complete ===");
}

void UnifiedDSController::forceDataCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  // 力传感器数据处理和低通滤波
  Eigen::Vector3d raw_force(msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
  
  // 实现一阶低通滤波器以减少噪声
  double filtered_force_gain = ds_impedance_params_.filtered_force_gain_;
  external_force_ = filtered_force_gain * external_force_ + (1.0 - filtered_force_gain) * raw_force;
  
  // 在校准阶段进行力传感器调零检测
  if (!is_calibrated_) {
    static ros::Time calibration_start_time = ros::Time::now();
    static bool calibration_started = false;
    
    // 检查力值是否小于0.5N
    double force_magnitude = external_force_.norm();
    
    if (force_magnitude < 0.5) {
      if (!calibration_started) {
        calibration_start_time = ros::Time::now();
        calibration_started = true;
        ROS_INFO("Force sensor calibration started - force below 0.5N, waiting 3 seconds...");
      }
      
      double elapsed_time = (ros::Time::now() - calibration_start_time).toSec();
      if (elapsed_time >= 3.0) {
        is_calibrated_ = true;
        baseline_force_z_ = external_force_(2);
        ROS_INFO("===== FORCE SENSOR CALIBRATION COMPLETED =====");
        ROS_INFO_STREAM("Force stable below 0.5N for 3 seconds. Final force: " << force_magnitude << " N");
        ROS_INFO("System ready - frontend start button can now be clicked");
      }
    } else {
      // 如果力值超过阈值，重新开始计时
      if (calibration_started) {
        calibration_started = false;
        ROS_INFO_STREAM("Force too high (" << force_magnitude << " N), restarting calibration timer");
      }
    }
  }
}

void UnifiedDSController::publishControlStatus() {
  // 发布简单的状态消息供UI使用
  std_msgs::String status_msg;
  std_msgs::String phase_msg;
  
  // 根据当前阶段发布对应的状态消息
  if (!is_calibrated_) {
    status_msg.data = "CALIBRATING";
    phase_msg.data = "CALIBRATION";
  } else if (!motion_started_) {
    status_msg.data = "CALIBRATION_COMPLETE";  // UI界面期望的消息
    phase_msg.data = "READY";
  } else {
    status_msg.data = "MOTION_STARTED";
    phase_msg.data = "MOTION";
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