// Copyright (c) 2024 Franka DS Project
// Use of this source code is governed by the Apache-2.0 license

#include <franka_ds/force_guided_controller.h>
#include <cmath>
#include <memory>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_ds {

bool ForceGuidedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {
  ROS_INFO("Force Guided Controller initialization starting...");
  
  // 获取机器人arm_id和关节名称
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ForceGuidedController: Could not read parameter arm_id");
    return false;
  }
  
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR("ForceGuidedController: Invalid or no joint_names parameters provided");
    return false;
  }

  // 获取硬件接口
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR("ForceGuidedController: Error getting model interface from hardware");
    return false;
  }
  
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("ForceGuidedController: Exception getting model handle: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("ForceGuidedController: Error getting state interface from hardware");
    return false;
  }
  
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("ForceGuidedController: Exception getting state handle: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR("ForceGuidedController: Error getting effort joint interface from hardware");
    return false;
  }
  
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ForceGuidedController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // 加载控制参数
  if (!loadControlParameters(node_handle)) {
    ROS_ERROR("Failed to load control parameters");
    return false;
  }

  // 初始化ROS接口
  initializeROSInterface(node_handle);

  // 初始化控制状态
  position_d_.setZero();
  position_d_target_.setZero();
  
  external_force_.setZero();
  force_data_received_ = false;
  force_guided_mode_ = true;  // 默认开启力引导模式
  last_position_.setZero();

  ROS_INFO("Force Guided Controller initialization successful");
  return true;
}

void ForceGuidedController::starting(const ros::Time& /*time*/) {
  ROS_INFO("Force Guided Controller starting");
  
  // 获取初始状态
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  
  // 记录初始位置（仅用于参考）
  position_d_ = initial_transform.translation();
  position_d_target_ = position_d_;
  last_position_ = position_d_;

  // 启用被动模式
  force_guided_mode_ = true;

  ROS_INFO_STREAM("Initial position: [" << position_d_.transpose() << "]");
  ROS_INFO("Passive force following mode enabled - robot is now compliant");
  ROS_INFO("You can manually move the robot to desired positions");
}

void ForceGuidedController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  // 获取机器人状态
  franka::RobotState robot_state = state_handle_->getRobotState();
  
  // 转换为Eigen格式
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  
  // 当前关节状态
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

  // 计算当前末端位姿
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  // **核心修改：被动控制策略**
  // 1. 去掉所有主动位置/姿态控制
  // 2. 只施加最小阻尼以保持系统稳定性
  // 3. 外力通过机械臂的顺应性自然传递
  
  Eigen::Matrix<double, 7, 1> tau_d;
  
  if (force_guided_mode_) {
    // 被动模式：只施加轻微阻尼防止振荡
    // 阻尼矩阵设置得很小，只是为了系统稳定
    double passive_damping = 5.0;  // 很小的阻尼系数
    tau_d = -passive_damping * dq + coriolis;  // 关节空间阻尼 + 补偿重力/科氏力
    
    ROS_DEBUG_THROTTLE(1.0, "Passive mode: minimal damping control");
  } else {
    // 非被动模式：保持当前位置
    Eigen::Matrix<double, 6, 1> error;
    {
      std::lock_guard<std::mutex> position_lock(position_target_mutex_);
      error.head(3) << position - position_d_;
      error.tail(3) = Eigen::Vector3d::Zero();  // 不控制姿态
    }

    // 只进行位置控制，不控制姿态
    Eigen::Matrix<double, 6, 6> stiffness_matrix = Eigen::Matrix<double, 6, 6>::Zero();
    stiffness_matrix.topLeftCorner(3, 3) = params_.translational_stiffness_ * Eigen::Matrix3d::Identity();
    // 姿态刚度设为零，让姿态自由
    
    Eigen::Matrix<double, 6, 6> damping_matrix = Eigen::Matrix<double, 6, 6>::Zero();
    damping_matrix.topLeftCorner(3, 3) = params_.translational_damping_ * Eigen::Matrix3d::Identity();
    // 姿态阻尼也设为零

    tau_d = jacobian.transpose() * (-stiffness_matrix * error - damping_matrix * (jacobian * dq)) + coriolis;
  }

  // 限制力矩变化率
  tau_d = saturateTorqueRate(tau_d, tau_J_d);
  
  // 安全力矩限制
  for (size_t i = 0; i < 7; ++i) {
    tau_d(i) = std::max(-params_.max_joint_torque_, 
                       std::min(params_.max_joint_torque_, tau_d(i)));
  }

  // 发送力矩命令
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // 更新期望位置为当前位置（被动跟随）
  if (force_guided_mode_) {
    std::lock_guard<std::mutex> position_lock(position_target_mutex_);
    position_d_ = position;  // 期望位置始终跟随当前位置
  }
  
  // 发布当前位姿
  geometry_msgs::PoseStamped current_pose;
  current_pose.header.stamp = ros::Time::now();
  current_pose.header.frame_id = "panda_link0";
  current_pose.pose.position.x = position(0);
  current_pose.pose.position.y = position(1);
  current_pose.pose.position.z = position(2);
  current_pose.pose.orientation.x = orientation.x();
  current_pose.pose.orientation.y = orientation.y();
  current_pose.pose.orientation.z = orientation.z();
  current_pose.pose.orientation.w = orientation.w();
  pub_current_pose_.publish(current_pose);
  
  // 更新上一次位置
  last_position_ = position;
}

void ForceGuidedController::stopping(const ros::Time& /*time*/) {
  ROS_WARN("Force Guided Controller stopping");
  
  // 发送零力矩命令
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(0.0);
  }
}

Eigen::Matrix<double, 7, 1> ForceGuidedController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {
  Eigen::Matrix<double, 7, 1> tau_d_saturated;
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + 
        std::max(std::min(difference, params_.delta_tau_max_), -params_.delta_tau_max_);
  }
  return tau_d_saturated;
}

bool ForceGuidedController::loadControlParameters(ros::NodeHandle& nh) {
  // 阻抗参数
  nh.param("translational_stiffness", params_.translational_stiffness_, 150.0);
  nh.param("rotational_stiffness", params_.rotational_stiffness_, 10.0);
  nh.param("translational_damping", params_.translational_damping_, 2.0 * sqrt(150.0));
  nh.param("rotational_damping", params_.rotational_damping_, 2.0 * sqrt(10.0));
  nh.param("nullspace_stiffness", params_.nullspace_stiffness_, 20.0);
  
  // 力引导参数
  nh.param("force_threshold", params_.force_threshold_, 2.0);
  nh.param("force_gain", params_.force_gain_, 0.01);
  nh.param("max_force_velocity", params_.max_force_velocity_, 0.1);
  
  // 安全参数
  nh.param("max_joint_torque", params_.max_joint_torque_, 12.0);
  nh.param("delta_tau_max", params_.delta_tau_max_, 1.0);
  
  // 滤波参数
  nh.param("filter_params", params_.filter_params_, 0.005);
  
  // 吸引子和接触面参数
  std::vector<double> attractor_pos;
  if (nh.getParam("attractor_position", attractor_pos) && attractor_pos.size() == 3) {
    params_.attractor_position_ << attractor_pos[0], attractor_pos[1], attractor_pos[2];
  } else {
    params_.attractor_position_ << 0.5, 0.0, 0.2;
  }
  nh.param("contact_surface_height", params_.contact_surface_height_, 0.18);
  
  // 打印加载的参数
  ROS_INFO("Force Guided Controller Parameters:");
  ROS_INFO("  Translational stiffness: %.1f N/m", params_.translational_stiffness_);
  ROS_INFO("  Rotational stiffness: %.1f Nm/rad", params_.rotational_stiffness_);
  ROS_INFO("  Force threshold: %.1f N", params_.force_threshold_);
  ROS_INFO("  Force gain: %.3f m/(N*s)", params_.force_gain_);
  ROS_INFO("  Attractor position: [%.3f, %.3f, %.3f]", 
           params_.attractor_position_(0), 
           params_.attractor_position_(1), 
           params_.attractor_position_(2));
  
  return true;
}

void ForceGuidedController::initializeROSInterface(ros::NodeHandle& nh) {
  // 订阅器
  sub_force_data_ = nh.subscribe("/force_sensor/wrench", 1, 
                                 &ForceGuidedController::forceDataCallback, this);
  sub_attractor_position_ = nh.subscribe("/force_guided/set_attractor", 1,
                                         &ForceGuidedController::attractorPositionCallback, this);
  sub_command_ = nh.subscribe("/force_guided/command", 1,
                             &ForceGuidedController::commandCallback, this);
  
  // 发布器
  pub_current_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/force_guided/current_pose", 1);
  pub_control_status_ = nh.advertise<std_msgs::String>("/force_guided/status", 1);
  
  ROS_INFO("Force Guided Controller ROS interface initialized");
}

void ForceGuidedController::forceDataCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  std::lock_guard<std::mutex> force_lock(force_data_mutex_);
  external_force_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z;
  force_data_received_ = true;
}

void ForceGuidedController::attractorPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  std::lock_guard<std::mutex> position_lock(position_target_mutex_);
  params_.attractor_position_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  params_.contact_surface_height_ = msg->pose.position.z;
  
  ROS_INFO("Updated attractor position: [%.3f, %.3f, %.3f], contact height: %.3f", 
           params_.attractor_position_(0), 
           params_.attractor_position_(1), 
           params_.attractor_position_(2),
           params_.contact_surface_height_);
}

void ForceGuidedController::commandCallback(const std_msgs::String::ConstPtr& msg) {
  if (msg->data == "enable_force_guide") {
    force_guided_mode_ = true;
    ROS_INFO("Force guided mode enabled");
  } else if (msg->data == "disable_force_guide") {
    force_guided_mode_ = false;
    ROS_INFO("Force guided mode disabled");
  }
}

Eigen::Vector3d ForceGuidedController::computeForceGuidedVelocity(const Eigen::Vector3d& external_force) {
  // 检查力是否超过阈值
  double force_magnitude = external_force.norm();
  if (force_magnitude < params_.force_threshold_) {
    return Eigen::Vector3d::Zero();  // 无外力时保持当前位置
  }
  
  // 计算力引导速度
  Eigen::Vector3d force_direction = external_force.normalized();
  double velocity_magnitude = std::min(params_.force_gain_ * force_magnitude, 
                                      params_.max_force_velocity_);
  
  return velocity_magnitude * force_direction;
}

Eigen::Vector3d ForceGuidedController::computeDesiredPosition(const Eigen::Vector3d& current_position, 
                                                              const Eigen::Vector3d& force_velocity, 
                                                              double dt) {
  // 基于力引导速度更新期望位置
  Eigen::Vector3d new_position = current_position + force_velocity * dt;
  
  // 限制移动范围（安全考虑）
  const double max_displacement = 0.5;  // 最大偏移量 [m]
  Eigen::Vector3d displacement = new_position - last_position_;
  if (displacement.norm() > max_displacement) {
    displacement = displacement.normalized() * max_displacement;
    new_position = last_position_ + displacement;
  }
  
  return new_position;
}

}  // namespace franka_ds

// 注册控制器插件
PLUGINLIB_EXPORT_CLASS(franka_ds::ForceGuidedController, controller_interface::ControllerBase) 