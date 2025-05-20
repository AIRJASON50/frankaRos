/**
 * @file circle_controller.cpp
 * @brief 集成力控制和轨迹跟踪的圆周运动控制器
 *
 * @架构：
 * - CircleController类：继承自controller_interface::ControllerBase
 * - 整合了轨迹生成、力控制和软接触模型
 *
 * @数据流：
 * 输入：机器人状态、力传感器数据 -> 
 * 处理：根据控制阶段（接近、接触、轨迹）生成运动轨迹和力控制命令 -> 
 * 输出：关节力矩命令和状态数据
 *
 * @概述：
 * 1. 三阶段控制流程：接近阶段、接触阶段、轨迹运动阶段
 * 2. 结合位置控制和力控制的混合控制策略
 * 3. 利用软接触模型实现稳定的接触力控制
 * 4. 支持多种轨迹类型的力控接触运动
 * 5. 提供数据记录和状态可视化功能
 */
#include <franka_example_controllers/circle_controller.h>

#include <cmath>
#include <memory>
#include <chrono>
#include <ctime>
#include <random>
#include <deque>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

// 在停止控制器时关闭日志文件
void CircleController::stopping(const ros::Time& time) {
  // 记录结束时间并关闭日志文件
  log_generator_.closeLogFile(time);
}

bool CircleController::init(hardware_interface::RobotHW* robot_hw,
                           ros::NodeHandle& node_handle) {
  // 保存节点句柄
  node_handle_ = node_handle;
  
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

  // 初始化轨迹生成器
  trajectory_generator_ = std::make_unique<TrajectoryGenerator>();
  
  // 读取轨迹参数
  node_handle.param<double>("circle_frequency", circle_frequency_, 0.3);
  node_handle.param<double>("circle_radius", circle_radius_, 0.1);
  node_handle.param<std::string>("circle_plane", circle_plane_, "xy");
  ROS_INFO_STREAM("CircleController: Circle parameters: frequency=" << circle_frequency_ 
                 << "Hz, radius=" << circle_radius_ << "m, plane=" << circle_plane_);
  
  // 初始化路径发布器
  path_pub_ = node_handle.advertise<nav_msgs::Path>("trajectory_path", 1);
  
  // 初始化轨迹生成器
  if (!trajectory_generator_->init(node_handle, path_pub_)) {
    ROS_ERROR_STREAM("CircleController: Failed to initialize trajectory generator");
    return false;
  }
  
  // 读取软接触模型参数
  ros::NodeHandle soft_contact_nh(node_handle, "contact_model");
  contact_params_.young_modulus = 1000.0;  // 默认值：1kPa
  contact_params_.poisson_ratio = 0.45;    // 默认值：近似不可压缩
  contact_params_.friction_coef = 0.3;     // 默认值：中等摩擦系数
  contact_params_.contact_radius = 0.01;   // 默认值：1cm半径的探头
  contact_params_.path_radius = 0.1;       // 默认值：10cm圆周路径半径
  contact_params_.damping = 50.0;          // 默认值：中等阻尼
  contact_params_.force_threshold = 3.0;   // 默认值：3N接触力阈值
  contact_params_.depth_threshold = 0.001; // 默认值：1mm接触深度阈值
  
  soft_contact_nh.getParam("young_modulus", contact_params_.young_modulus);
  soft_contact_nh.getParam("poisson_ratio", contact_params_.poisson_ratio);
  soft_contact_nh.getParam("friction_coef", contact_params_.friction_coef);
  soft_contact_nh.getParam("contact_radius", contact_params_.contact_radius);
  soft_contact_nh.getParam("path_radius", contact_params_.path_radius);
  soft_contact_nh.getParam("damping", contact_params_.damping);
  soft_contact_nh.getParam("force_threshold", contact_params_.force_threshold);
  soft_contact_nh.getParam("depth_threshold", contact_params_.depth_threshold);
  
  // 加载软体块参数
  if (!node_handle.getParam("soft_block/position_x", soft_block_x_) ||
      !node_handle.getParam("soft_block/position_y", soft_block_y_) ||
      !node_handle.getParam("soft_block/position_z", soft_block_z_)) {
    ROS_ERROR_STREAM("CircleController: Failed to get soft block position parameters");
    return false;
  }
  
  // 计算软块中心位置向量
  soft_block_position_ = Eigen::Vector3d(soft_block_x_, soft_block_y_, soft_block_z_);
  
  // 加载软块表面高度
  if (!node_handle.getParam("soft_block/surface_z", soft_block_surface_z_)) {
    ROS_ERROR_STREAM("CircleController: Failed to get soft block surface height parameter");
    return false;
  }
  
  // 加载探头长度参数
  if (!node_handle.getParam("probe/length", probe_length_)) {
    ROS_ERROR_STREAM("CircleController: Failed to get probe length parameter");
    return false;
  }
  
  // 打印详细的接触控制信息，用于调试
  ROS_INFO("-------------------------- CircleController Initialization Parameters --------------------------");
  ROS_INFO("Soft Block Position Info:");
  ROS_INFO("  - Center Position: [%.3f, %.3f, %.3f] m", soft_block_x_, soft_block_y_, soft_block_z_);
  ROS_INFO("  - Surface Height: %.3f m", soft_block_surface_z_);
  ROS_INFO("  - Surface-Center Distance: %.3f m", soft_block_surface_z_ - soft_block_z_);
  ROS_INFO("Probe Info:");
  ROS_INFO("  - Probe Length: %.3f m", probe_length_);
  ROS_INFO("Contact Control Parameters:");
  ROS_INFO("  - Target Contact Depth: %.2f mm", target_depth_ * 1000);
  ROS_INFO("  - Depth Threshold: %.2f mm", contact_params_.depth_threshold * 1000);
  ROS_INFO("  - Phase Duration: %.1f s", phase_duration_);
  ROS_INFO("------------------------------------------------------------------------------");
  
  // 在init函数中添加控制模式读取
  int control_mode_param = 0;
  if (node_handle.getParam("control_mode", control_mode_param)) {
    if (control_mode_param == 1) {
      control_mode_ = FORCE_CONTROL_MODE;
      ROS_INFO_STREAM("CircleController: Using force control mode");
    } else {
      control_mode_ = DEPTH_CONTROL_MODE;
      ROS_INFO_STREAM("CircleController: Using depth control mode");
    }
  } else {
    // 尝试读取字符串格式的控制模式（向后兼容）
    std::string control_mode_str;
    if (node_handle.getParam("control_mode", control_mode_str)) {
      if (control_mode_str == "force") {
        control_mode_ = FORCE_CONTROL_MODE;
        ROS_INFO_STREAM("CircleController: Using force control mode (string parameter)");
      } else {
        control_mode_ = DEPTH_CONTROL_MODE;
        ROS_INFO_STREAM("CircleController: Using depth control mode (string parameter)");
      }
    } else {
      // 默认使用深度控制模式
      control_mode_ = DEPTH_CONTROL_MODE;
      ROS_INFO_STREAM("CircleController: No control mode specified, using default depth control mode");
    }
  }

  // 读取深度控制参数
  ros::NodeHandle depth_nh(node_handle, "depth_controller");
  depth_nh.getParam("target_depth", target_depth_);
  depth_nh.getParam("p_gain", depth_p_gain_);
  depth_nh.getParam("i_gain", depth_i_gain_);
  
  // 读取力控制参数
  ros::NodeHandle force_nh(node_handle, "force_controller");
  force_nh.getParam("target_force", target_force_);
  force_nh.getParam("p_gain", force_p_gain_);
  force_nh.getParam("i_gain", force_i_gain_);
  force_nh.getParam("d_gain", force_d_gain_);
  force_nh.getParam("max_force", max_force_);
  
  // 同步力控制参数副本
  force_kp_ = force_p_gain_;
  force_ki_ = force_i_gain_;
  force_kd_ = force_d_gain_;
  
  // 读取相位持续时间
  node_handle.getParam("phase_duration", phase_duration_);
  
  // 初始化软接触模型
  contact_model_ = std::make_unique<franka_example_controllers::SoftContactModel>(
      node_handle, contact_params_);
  
  // 初始化发布器
  pose_pub_ = node_handle.advertise<geometry_msgs::PoseStamped>("equilibrium_pose", 10);
  contact_pub_ = node_handle.advertise<geometry_msgs::WrenchStamped>("contact_force", 10);
  phase_pub_ = node_handle.advertise<std_msgs::String>("control_phase", 10);

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

  // 设置笛卡尔阻抗控制参数（在开环控制中仅用于计算虚拟力矩）
  double translational_stiffness = 600.0;  // 位置刚度提高到600.0 N/m (原100.0)
  double rotational_stiffness = 50.0;      // 姿态刚度提高到50.0 Nm/rad (原20.0)
  
  cartesian_stiffness_.setZero();
  cartesian_stiffness_.topLeftCorner(3, 3)
      << translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_.bottomRightCorner(3, 3)
      << rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_.setZero();
  cartesian_damping_.topLeftCorner(3, 3)
      << 2.0 * sqrt(translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_ = 0.5;  // 零空间刚度：0.5

  // 初始化日志文件
  log_generator_.initLogFile(contact_params_, target_force_);

  // 初始化力轨迹生成器
  force_generator_ = std::make_unique<ForceGenerator>();
  if (!force_generator_->init(node_handle)) {
    ROS_ERROR_STREAM("CircleController: Failed to initialize force generator");
    return false;
  }
  
  // 保存噪声参数以供其他地方使用
  force_nh.getParam("force_noise_enable", force_noise_enable_);

  // 设置轨迹生成器的软块参数
  if (trajectory_generator_) {
    trajectory_generator_->setSoftBlockParameters(
      soft_block_surface_z_,  // 软块表面高度
      target_depth_           // 目标深度
    );
    // 默认需要接触
    trajectory_generator_->setContactRequired(true);
  }

  // 初始化成功
  return true;
}

void CircleController::starting(const ros::Time& time) {
  // 获取初始状态
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // 如果中心点未指定，则使用初始位置作为中心点
  if (circle_center_.norm() < 1e-6) {
  circle_center_ = initial_transform.translation();
    ROS_INFO_STREAM("CircleController: Using initial position as trajectory center: "
                    << circle_center_.transpose());
  } else {
    ROS_INFO_STREAM("CircleController: Using specified trajectory center: "
                    << circle_center_.transpose());
  }
  
  // 设置初始目标位置和方向
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  
  // 设置零空间平衡配置为初始关节角度
  q_d_nullspace_ = q_initial;
  
  // 重置时间
  elapsed_time_ = 0.0;
  
  // 重置控制阶段 - 始终从接近阶段开始
  control_phase_ = APPROACH;
  phase_start_time_ = time;
  
  // 重置力控制变量
  force_error_integral_ = 0.0;
  prev_force_error_ = 0.0;
  last_force_error_ = 0.0;
  
  // 重置接触状态
  contact_detected_ = false;
  contact_position_ = Eigen::Vector3d::Zero();
  contact_depth_ = 0.0;
  
  // 设置轨迹生成器参数
  trajectory_generator_->setCenter(position_d_);
  
  // 轨迹生成器配置 - 从接近阶段开始时禁用接触要求
  trajectory_generator_->setContactRequired(false);
  trajectory_generator_->setPreserveHeight(true);
  
  // 确保轨迹生成器知道软块位置
  trajectory_generator_->setSoftBlockParameters(soft_block_surface_z_, target_depth_);
  
  // 发布初始控制阶段
  std_msgs::String phase_msg;
  phase_msg.data = "APPROACH";
  phase_pub_.publish(phase_msg);
  
  // 输出初始设置信息
  ROS_INFO_STREAM("---------------------- Controller Started ----------------------");
  ROS_INFO_STREAM("Initial Phase: Approach");
  ROS_INFO_STREAM("Initial Position: [" << position_d_.transpose() << "]");
  ROS_INFO_STREAM("Soft Block Surface Height: " << soft_block_surface_z_ << " m");
  ROS_INFO_STREAM("Target Contact Depth: " << target_depth_ * 1000 << " mm");
  ROS_INFO_STREAM("Trajectory Type: " << trajectory_generator_->getTrajectoryTypeStr());
  if (trajectory_generator_->getTrajectoryType() == CIRCULAR) {
    ROS_INFO_STREAM("Circle Parameters: Plane=" << circle_plane_ << ", Radius=" << circle_radius_ << "m, Frequency=" << circle_frequency_ << "Hz");
  }
  ROS_INFO_STREAM("--------------------------------------------------------");
  
  // 重置计数器
  circular_counter_ = 0;
  
  // 重置姿态初始化标志
  desired_pose_initialized_ = false;
}

void CircleController::update(const ros::Time& time,
                             const ros::Duration& period) {
  // 获取机器人状态
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 16> O_T_EE_array = robot_state.O_T_EE;
  std::array<double, 49> mass_array = model_handle_->getMass();
  
  // 将机器人状态转换为Eigen矩阵
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 4, 4>> O_T_EE(O_T_EE_array.data());
  Eigen::Affine3d transform(O_T_EE);
  Eigen::Vector3d position = transform.translation();
  Eigen::Quaterniond orientation(transform.rotation());
  
  // 计算探头末端的真实位置（在z轴方向减去探头长度）
  // 注意：探头沿着末端坐标系的z轴负方向延伸，所以是减法
  Eigen::Vector3d end_position = position;
  end_position(2) -= probe_length_; // 探头末端位置 = 法兰位置 - 探头长度
  
  // 获取外部力估计
  const std::array<double, 6>& wrench_array = robot_state.K_F_ext_hat_K;
  Eigen::Map<const Eigen::Matrix<double, 6, 1>> wrench(wrench_array.data());
  Eigen::Vector3d force = wrench.head(3);
  external_force_ = force; // 更新外部力成员变量
  
  // 获取关节状态
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  
  // 获取当前末端位姿
  Eigen::Affine3d current_pose(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  
  // 从机器人状态获取外部力估计
  Eigen::Map<const Eigen::Matrix<double, 6, 1>> external_wrench(robot_state.O_F_ext_hat_K.data());
  Eigen::Vector3d measured_force = external_wrench.head(3);  // 仅使用力部分
  double force_magnitude = measured_force.norm();
  
  // 力值判断需要历史数据，添加力历史窗口
  static std::deque<double> force_history;
  force_history.push_back(force(2)); // 只记录z方向力
  if (force_history.size() > 10) {
    force_history.pop_front();
  }
  
  // 计算力变化最大值
  double max_force_change = 0.0;
  for (size_t i = 1; i < force_history.size(); ++i) {
    max_force_change = std::max(max_force_change, 
                               std::abs(force_history[i] - force_history[i-1]));
  }
  
  // 更新接触状态
  bool was_in_contact = contact_detected_;
  
  // 接触检测: 力突变 + 位置判断
  if (!contact_detected_) {
    // 判断是否首次接触: 力突变(大于2N) + 位置低于软块表面
    if (max_force_change > 2.0 && end_position(2) <= (soft_block_surface_z_ + 0.005)) {
      contact_detected_ = true;
      contact_position_ = end_position; // 保存接触检测位置，使用探头末端位置而非法兰位置
      ROS_INFO("Contact detection: Force change detected a contact! Force change: %.2f N at position: [%.4f, %.4f, %.4f]", 
               max_force_change, end_position(0), end_position(1), end_position(2));
               
      // 输出软块表面高度信息，便于调试
      ROS_INFO("Contact info: Soft block surface_z = %.5f, Contact position z = %.5f, Depth = %.5f mm", 
               soft_block_surface_z_, end_position(2), (soft_block_surface_z_ - end_position(2)) * 1000.0);
    }
  } else {
    // 根据当前控制阶段使用不同的接触丢失检测逻辑
    if (control_phase_ == TRAJECTORY) {
      // 轨迹阶段: 使用更大的阈值以防止轻微高度变化导致频繁状态切换
      if (end_position(2) > (soft_block_surface_z_ + 0.005)) { // 降低阈值到5mm，匹配软块实际厚度
        contact_detected_ = false;
        ROS_INFO("Contact lost: End-effector above soft block surface, height difference: %.3f mm", 
                 (end_position(2) - soft_block_surface_z_) * 1000.0);
      }
    } else {
      // 其他阶段: 使用较小的阈值
      if (end_position(2) > (soft_block_surface_z_ + 0.003)) { // 降低阈值到3mm，匹配软块实际厚度
        contact_detected_ = false;
        ROS_INFO("Contact lost: End-effector above soft block surface, height difference: %.3f mm", 
                 (end_position(2) - soft_block_surface_z_) * 1000.0);
      }
    }
  }
  
  // 计算接触深度 (如果在接触状态)
  if (contact_detected_) {
    // 接触深度 = 软块表面高度 - 当前探头末端Z坐标
    contact_depth_ = soft_block_surface_z_ - end_position(2);
    
    // 确保不会出现负深度 (可能由于数值误差导致)
    if (contact_depth_ < 0.0) {
      contact_depth_ = 0.0;
    }
    
    // 避免过大的深度值 (软块厚度为5mm，最大深度设为4mm)
    double max_allowed_depth = 0.004;
    if (contact_depth_ > max_allowed_depth) {
      contact_depth_ = max_allowed_depth;
    }
    
    // 更新日志生成器中的深度值
    log_generator_.setCurrentDepth(contact_depth_);
    
    // 记录接触位置与软块表面的关系，用于调试
    if (circular_counter_ % 300 == 0) {
      ROS_INFO("深度监控: 软块表面高度=%.5f, 当前位置z=%.5f, 深度=%.3fmm", 
               soft_block_surface_z_, end_position(2), contact_depth_ * 1000.0);
    }
  } else {
    contact_depth_ = 0.0; // 确保非接触状态下深度为0
    log_generator_.setCurrentDepth(0.0);
  }
  
  // 累计经过的时间（用于计算圆周轨迹）
  elapsed_time_ += period.toSec();
  
  // 动态更新目标力
  target_force_ = force_generator_->generateForceProfile(elapsed_time_);
  
  // 更新轨迹路径可视化 - 每10个周期更新一次，平衡性能和可视化效果
  if (circular_counter_ % 10 == 0) {
    trajectory_generator_->publishTrajectoryPath(elapsed_time_, soft_block_position_(2));
  }
  circular_counter_++;
  
  // =====================================================================
  // 1. 获取当前状态：传感器读数 - 已在函数开头获取，这里使用之前获取的数据
  // =====================================================================

  // 将Eigen矩阵类型修正为const类型，与下面的代码兼容
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_const(robot_state.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq_const(robot_state.dq.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

  // 计算末端速度
  Eigen::VectorXd v_ee = jacobian * dq;  // 笛卡尔空间速度
  Eigen::Vector3d position_d_dot = Eigen::Vector3d::Zero();  // 期望位置导数（速度）
  Eigen::Vector3d position_d_ddot = Eigen::Vector3d::Zero();  // 期望位置二阶导数（加速度）

  // =====================================================================
  // 2. 第一个周期：初始化期望姿态（只执行一次）
  // =====================================================================
  if (!desired_pose_initialized_) {
    // 记录当前姿态作为期望姿态（仅需记录一次）
    desired_pose_ = current_pose;
    
    // 确保z轴指向地面（探头朝下）
    Eigen::Vector3d z_axis = desired_pose_.rotation().col(2);  // 提取旋转矩阵的第3列，即z轴方向
    if (z_axis(2) > 0) {
      // 如果z轴向上（z分量为正），旋转180度使其向下
      Eigen::Matrix3d rotation_matrix = desired_pose_.rotation();
      // 创建180度绕x轴的旋转（使z轴翻转）
      Eigen::Matrix3d flip_rotation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
      // 应用旋转
      rotation_matrix = flip_rotation * rotation_matrix;
      // 更新期望姿态的旋转部分
      desired_pose_.linear() = rotation_matrix;
    }
    
    // 保存probe坐标系z轴作为圆周运动的法线（垂直于圆周平面）
    circle_normal_ = desired_pose_.rotation().col(2).normalized();
    
    // 计算圆周运动平面的基底向量（与法线垂直的两个单位向量）
    // 步骤1: 找到与z轴不平行的参考向量
    Eigen::Vector3d reference = (std::abs(circle_normal_(0)) < 0.9) ? 
        Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    // 步骤2: x平面向量 = 法线与参考向量的叉积（垂直于两者）
    circle_x_axis_ = circle_normal_.cross(reference).normalized();
    // 步骤3: y平面向量 = 法线与x平面向量的叉积（垂直于两者，形成右手系）
    circle_y_axis_ = circle_normal_.cross(circle_x_axis_).normalized();
    
    // 设置轨迹生成器的中心点和轴向
    trajectory_generator_->setCenter(circle_center_);
    trajectory_generator_->setAxis(circle_x_axis_, circle_y_axis_);
    
    // 记录初始关节角度（用于零空间控制）
    initial_q_ = q;
    
    // 初始位置作为起始点
    position_d_ = position;
    orientation_d_ = Eigen::Quaterniond(desired_pose_.rotation());
    
    // 初始化完成标记
    desired_pose_initialized_ = true;
    
    // 输出初始化信息
    ROS_INFO_STREAM("Desired pose initialized: \n" << desired_pose_.matrix());
    ROS_INFO_STREAM("Circle normal (z-axis): [" << circle_normal_.transpose() << "]");
    ROS_INFO_STREAM("Circle x-axis: [" << circle_x_axis_.transpose() << "]");
    ROS_INFO_STREAM("Circle y-axis: [" << circle_y_axis_.transpose() << "]");
    
    // 重置时间计数，确保从0开始
    elapsed_time_ = 0.0;
    return;
  }

  // =====================================================================
  // 3. 运动阶段控制
  // =====================================================================
  
  // 转换笛卡尔空间质量矩阵
  Eigen::Matrix3d mass_matrix_cart;
  mass_matrix_cart = (jacobian.topLeftCorner(3, 7) * mass.inverse() * jacobian.topLeftCorner(3, 7).transpose()).inverse();
  
  // 计算当前位置相对于软体块的深度
  double depth = soft_block_surface_z_ - end_position(2);
  bool in_contact = depth > contact_params_.depth_threshold * 0.5;
  
  // 更新接触状态
  ContactState current_contact_state = contact_model_->updateContact(
      position,
      v_ee.head(3),
      orientation,
      soft_block_surface_z_
  );
  
  // 更新深度值到日志生成器
  if (contact_detected_) {
    log_generator_.setCurrentDepth(contact_depth_);
  }
  
  // 发布接触状态
  geometry_msgs::WrenchStamped contact_wrench_msg;
  contact_wrench_msg.header.stamp = time;
  contact_wrench_msg.header.frame_id = "world";
  
  // 使用测量的力填充力反馈消息
  contact_wrench_msg.wrench.force.x = measured_force(0);
  contact_wrench_msg.wrench.force.y = measured_force(1);
  contact_wrench_msg.wrench.force.z = measured_force(2);
  
  // 设置力矩为零
  contact_wrench_msg.wrench.torque.x = 0.0;
  contact_wrench_msg.wrench.torque.y = 0.0;
  contact_wrench_msg.wrench.torque.z = 0.0;
  
  // 发布接触力信息
  contact_pub_.publish(contact_wrench_msg);
  
  // 发布估计力和测量力数据
  if (contact_depth_ > 0.0 || measured_force.norm() > 0.5) {
    // 直接处理接触状态（无需调用updateContactState）
    // 计算理论接触力
    double estimated_force = 0.0;
    if (contact_detected_ && contact_depth_ > 0.0) {
      estimated_force = contact_model_->computeNormalForce(contact_depth_);
    }
    
    // 直接发布力数据（无需调用publishForces）
    // 创建力信息消息
    geometry_msgs::WrenchStamped estimated_msg;
    estimated_msg.header.stamp = time;
    estimated_msg.header.frame_id = "world";
    
    // 设置估计力数据
    estimated_msg.wrench.force.x = 0.0;
    estimated_msg.wrench.force.y = 0.0;
    estimated_msg.wrench.force.z = estimated_force;
    estimated_msg.wrench.torque.x = 0.0;
    estimated_msg.wrench.torque.y = 0.0;
    estimated_msg.wrench.torque.z = 0.0;
    
    // 创建测量力信息消息
    geometry_msgs::WrenchStamped measured_msg;
    measured_msg.header = estimated_msg.header;
    measured_msg.wrench.force.x = measured_force(0);
    measured_msg.wrench.force.y = measured_force(1);
    measured_msg.wrench.force.z = measured_force(2);
    measured_msg.wrench.torque.x = 0.0;
    measured_msg.wrench.torque.y = 0.0;
    measured_msg.wrench.torque.z = 0.0;
    
    // 发布估计力和测量力
    if (external_force_pub_.getTopic().empty()) {
      external_force_pub_ = node_handle_.advertise<geometry_msgs::WrenchStamped>(
          "/circle_controller/estimated_force", 1);
    }
    if (measured_force_pub_.getTopic().empty()) {
      measured_force_pub_ = node_handle_.advertise<geometry_msgs::WrenchStamped>(
          "/circle_controller/measured_force", 1);
    }
    
    external_force_pub_.publish(estimated_msg);
    measured_force_pub_.publish(measured_msg);
    
    // 如果有接触，发布接触位姿
    if (contact_detected_) {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header = estimated_msg.header;
      pose_msg.pose.position.x = position_d_(0);
      pose_msg.pose.position.y = position_d_(1);
      pose_msg.pose.position.z = position_d_(2) - contact_depth_; // 调整位置到实际接触点
      pose_msg.pose.orientation.w = 1.0;
      pose_msg.pose.orientation.x = 0.0;
      pose_msg.pose.orientation.y = 0.0;
      pose_msg.pose.orientation.z = 0.0;
      
      // 发布接触位姿
      if (contact_pose_pub_.getTopic().empty()) {
        contact_pose_pub_ = node_handle_.advertise<geometry_msgs::PoseStamped>(
            "/circle_controller/contact_pose", 1);
      }
      contact_pose_pub_.publish(pose_msg);
    }
    }
  
  // =====================================================================
  // 4. 阶段控制逻辑
  // =====================================================================
  
  // 保存上一个阶段状态，用于检测状态变化
  ControlPhase previous_phase = control_phase_;
  
  // 阶段状态机转换逻辑
  switch (control_phase_) {
    case APPROACH:
      // 从接近阶段到接触检测阶段：当检测到接触力
      if (contact_detected_) {
      control_phase_ = CONTACT_DETECTION;
      phase_start_time_ = time;
        ROS_INFO("Phase transition: APPROACH -> CONTACT_DETECTION, contact detected");
      } else {
        // 接近阶段特定动作：向下移动直到接触
        // 使用较小的步长，以更平稳地接近表面
        position_d_[2] = std::max(position_d_[2] - 0.0003, soft_block_surface_z_ + 0.005);
    }
      break;
      
    case CONTACT_DETECTION:
      // 从接触检测阶段到深度控制阶段：接触稳定0.5秒后
      if ((time - phase_start_time_).toSec() > 0.5) {
        control_phase_ = DEPTH_CONTROL;
        phase_start_time_ = time;
        // 重置控制器积分项
        depth_error_integral_ = 0.0;
        force_error_integral_ = 0.0;
        ROS_INFO("Phase transition: CONTACT_DETECTION -> DEPTH_CONTROL, contact stabilized");
      }
      
      // 如果在接触检测期间失去接触，回到接近阶段
      if (!contact_detected_) {
        control_phase_ = APPROACH;
      phase_start_time_ = time;
        ROS_WARN("Phase transition: CONTACT_DETECTION -> APPROACH, contact lost during detection");
      }
      break;
      
    case DEPTH_CONTROL:
      // 从深度控制阶段到轨迹阶段：达到目标深度
      // 使用更宽松的判断标准，解决目标深度与实际深度差异大的问题
      {
        // 历史深度记录
        static std::deque<double> depth_history;
        depth_history.push_back(contact_depth_);
        if (depth_history.size() > 20) { // 减少窗口大小，提高响应速度
          depth_history.pop_front();
        }
        
        // 计算深度平均值和方差
        double depth_mean = 0.0;
        for (const auto& d : depth_history) {
          depth_mean += d;
        }
        depth_mean /= depth_history.size();
        
        double depth_variance = 0.0;
        for (const auto& d : depth_history) {
          depth_variance += (d - depth_mean) * (d - depth_mean);
        }
        depth_variance /= depth_history.size();
        
        // 1. 深度稳定性条件 - 放宽深度接近目标的要求，更关注稳定性
        // 由于实际深度与目标深度存在很大差异(目标1mm vs 实际22mm)，
        // 我们关注深度是否稳定，而不是深度是否接近目标值
        bool depth_stable = (depth_variance < 0.000008) && (depth_history.size() >= 15); 
        
        // 2. 时间条件 - 需要在稳定深度维持至少1.0秒
        static ros::Time target_depth_reached_time = ros::Time(0);
        if (depth_stable && !target_depth_reached_time.isValid()) {
          target_depth_reached_time = time;
          ROS_INFO("Depth stable, starting time count: %.3f mm, variance: %.6f mm²",
                  depth_mean * 1000.0, depth_variance * 1000000.0);
        } else if (!depth_stable) {
          if (target_depth_reached_time.isValid()) {
            ROS_INFO("Depth stability lost, resetting time count");
          }
          target_depth_reached_time = ros::Time(0); // 重置时间
        }
        
        // 检查时间条件
        bool time_condition = false;
        if (target_depth_reached_time.isValid()) {
          double stability_time = (time - target_depth_reached_time).toSec();
          time_condition = stability_time > 1.0; // 减少稳定时间要求到1.0秒
          
          // 每500个循环打印一次稳定时间进度
          if (circular_counter_ % 500 == 0 && target_depth_reached_time.isValid()) {
            ROS_INFO("Waiting for depth stability: %.1f/1.0 seconds elapsed", stability_time);
          }
        }
        
        // 达到稳定深度并且时间条件满足或者超过最大时间时进入轨迹阶段
        bool max_time_exceeded = (time - phase_start_time_).toSec() > 5.0; 
        
        // 状态转换逻辑
        if ((depth_stable && time_condition) || max_time_exceeded) {
          control_phase_ = TRAJECTORY;
          phase_start_time_ = time;
          // 重置轨迹时间
          elapsed_time_ = 0.0;
          // 重置深度控制相关变量
          depth_error_integral_ = 0.0;
          // 重置稳定时间计数器
          target_depth_reached_time = ros::Time(0);
          
          // 设置初始位置作为轨迹中心 - 确保在正确的深度开始轨迹
          // 使用当前稳定深度作为目标深度
          if (trajectory_generator_) {
            Eigen::Vector3d current_center = position;
            
            // 注意：使用当前实际位置的Z坐标，而不是目标深度
            // 这样可以避免轨迹生成器试图调整到不现实的深度
            double actual_depth = contact_depth_;
            current_center(2) = soft_block_surface_z_ - actual_depth;
            
            trajectory_generator_->setCenter(current_center);
            ROS_INFO("Using actual stable depth: %.3f mm instead of target: %.3f mm",
                     actual_depth * 1000.0, target_depth_ * 1000.0);
          }
          
          ROS_INFO("Phase transition: DEPTH_CONTROL -> TRAJECTORY, depth: %.3f mm (variance: %.6f mm², time stable: %s, max time: %s)", 
                  depth_mean * 1000.0, 
                  depth_variance * 1000000.0,
                  time_condition ? "yes" : "no",
                  max_time_exceeded ? "exceeded" : "not exceeded");
        }
      }
      
      // 如果在深度控制阶段失去接触，回到接近阶段
      if (!contact_detected_) {
        control_phase_ = APPROACH;
        phase_start_time_ = time;
        ROS_WARN("Phase transition: DEPTH_CONTROL -> APPROACH, contact lost during depth control");
      }
      break;
      
    case TRAJECTORY:
      // 轨迹阶段特定动作：生成XY平面轨迹点
      {
        // 生成轨迹位置（同时考虑深度控制）
        Eigen::Vector3d trajectory_position = trajectory_generator_->generateTrajectory(
          elapsed_time_,     // 当前轨迹时间
          contact_detected_, // 是否处于接触状态
          contact_depth_     // 当前接触深度
        );
        
        // 更新目标位置
        position_d_[0] = trajectory_position[0];
        position_d_[1] = trajectory_position[1];
        
        // 改进轨迹阶段的深度控制，使用实际深度而非目标深度
        // 这解决了机械臂尝试达到不切实际的1mm深度导致的弹跳问题
        if (contact_detected_) {
          // 使用静态变量记录进入轨迹阶段时的深度作为参考深度
          static double trajectory_reference_depth = contact_depth_;
          
          // 确保z坐标保持在参考深度附近，允许微小波动
          double target_z = soft_block_surface_z_ - trajectory_reference_depth;
          
          // 如果当前位置与目标位置相差较大，平滑过渡
          if (std::abs(position_d_[2] - target_z) > 0.0005) {
            // 使用平滑过渡（每次移动最多0.1mm）
            double max_step = 0.0001;
            double step = target_z - position_d_[2];
            
            // 限制调整步长
            if (std::abs(step) > max_step) {
              step = (step > 0) ? max_step : -max_step;
            }
            
            // 应用调整
            position_d_[2] += step;
          } else {
            // 深度接近目标深度，直接设置为目标深度
            position_d_[2] = target_z;
          }
        }
        
        // 每500个周期发布一次轨迹路径可视化（防止过度频繁刷新）
        if (circular_counter_ % 500 == 0) {
          // 根据接触状态设置路径显示高度
          double path_z = soft_block_surface_z_;
          
          // 如果处于接触状态，将路径设置在当前深度处
          if (contact_detected_) {
            path_z = position_d_[2];
          }
          
          // 发布轨迹路径可视化
          trajectory_generator_->publishTrajectoryPath(elapsed_time_, path_z);
        }
      }
      
      // 如果失去接触，返回接近阶段
      if (!contact_detected_) {
        control_phase_ = APPROACH;
      phase_start_time_ = time;
        ROS_WARN("Phase transition: TRAJECTORY -> APPROACH, contact lost during trajectory");
      }
      break;
  }
      
  // 阶段变化时发布消息
  if (previous_phase != control_phase_) {
      std_msgs::String phase_msg;
    switch (control_phase_) {
      case APPROACH: 
        phase_msg.data = "APPROACH";
        // 进入接近阶段时，禁用接触要求
        if (trajectory_generator_) {
          trajectory_generator_->setContactRequired(false);
          trajectory_generator_->setPreserveHeight(true);
        }
        break;
      case CONTACT_DETECTION: 
        phase_msg.data = "CONTACT_DETECTION"; 
        break;
      case DEPTH_CONTROL: 
      phase_msg.data = "DEPTH_CONTROL";
        break;
      case TRAJECTORY: 
        phase_msg.data = "TRAJECTORY"; 
        // 进入轨迹阶段时，启用接触要求
        if (trajectory_generator_) {
          trajectory_generator_->setContactRequired(true);
          trajectory_generator_->setPreserveHeight(false);
          
          // 确保使用当前位置作为轨迹中心
          Eigen::Vector3d current_center = position;
          // 修正Z坐标为目标深度
          current_center(2) = soft_block_surface_z_ - target_depth_;
          trajectory_generator_->setCenter(current_center);
          
          ROS_INFO("Trajectory center set to [%.4f, %.4f, %.4f] at target depth: %.3f mm",
                  current_center(0), current_center(1), current_center(2),
                  target_depth_ * 1000.0);
        }
        break;
    }
    phase_pub_.publish(phase_msg);
  }

  // 控制模式切换相关的状态发布
  static ControlMode last_control_mode = control_mode_;
  if (last_control_mode != control_mode_) {
    std_msgs::String mode_msg;
    mode_msg.data = (control_mode_ == DEPTH_CONTROL_MODE) ? "DEPTH_CONTROL_MODE" : "FORCE_CONTROL_MODE";
    phase_pub_.publish(mode_msg);
      
    // 输出控制模式切换信息
    ROS_INFO("Control mode switched: %s", mode_msg.data.c_str());
    
    // 重置控制器状态
    if (control_mode_ == DEPTH_CONTROL_MODE) {
      depth_error_integral_ = 0.0;
    } else {
      force_error_integral_ = 0.0;
      prev_force_error_ = 0.0;
    }
  
    // 更新上次控制模式
    last_control_mode = control_mode_;
  }
  
  // 计算理论接触力 - 如果在接触状态
    double theoretical_force = 0.0;
  if (contact_detected_ && contact_depth_ > 0.0) {
      theoretical_force = contact_model_->computeNormalForce(contact_depth_);
    }
    
  // 深度/力控制模式 - 仅适用于深度控制和轨迹阶段
  if (control_phase_ == DEPTH_CONTROL || control_phase_ == TRAJECTORY) {
    if (control_mode_ == DEPTH_CONTROL_MODE) {
      // 深度控制逻辑 - 改进版本，提高稳定性和精确性
      if (contact_detected_) {
        // 添加深度滤波，避免噪声影响
        static std::deque<double> depth_filter_buffer;
        depth_filter_buffer.push_back(contact_depth_);
        if (depth_filter_buffer.size() > 12) { // 增加滤波窗口到12个样本
          depth_filter_buffer.pop_front();
        }
        
        // 计算滤波后的深度值（排除极值后的均值滤波）
        std::vector<double> sorted_depths(depth_filter_buffer.begin(), depth_filter_buffer.end());
        std::sort(sorted_depths.begin(), sorted_depths.end());
        
        // 如果有足够的样本，排除最高和最低值
        double filtered_depth = 0.0;
        if (sorted_depths.size() >= 6) {
          // 去除最高和最低值
          for (size_t i = 1; i < sorted_depths.size() - 1; ++i) {
            filtered_depth += sorted_depths[i];
          }
          filtered_depth /= (sorted_depths.size() - 2);
    } else {
          // 样本不够多时使用所有值的平均
          for (const auto& d : depth_filter_buffer) {
            filtered_depth += d;
          }
          filtered_depth /= depth_filter_buffer.size();
        }
        
        double depth_error = target_depth_ - filtered_depth;
        
        // 更新积分项，应用积分窗口以限制长期累积
        depth_error_integral_ += depth_error * period.toSec();
        
        // 限制积分项以防止积分饱和
        double integral_limit = 0.002; // 减小积分限制为2mm
        depth_error_integral_ = std::max(std::min(depth_error_integral_, integral_limit), -integral_limit);
        
        // 计算位置调整量 - 使用PI-D控制
        // 当接近目标深度时使用更小的增益以防止过冲
        double adaptive_p_gain = depth_p_gain_;
        if (std::abs(depth_error) < 0.0003) { // 0.3mm内使用更小的增益
          adaptive_p_gain = depth_p_gain_ * 0.2; // 更强的减益
        } else if (std::abs(depth_error) < 0.0007) { // 0.7mm内使用中等增益
          adaptive_p_gain = depth_p_gain_ * 0.4;
        }
        
        // 计算微分项（速度反馈）用于阻尼
        static double prev_depth_error = 0.0;
        double depth_d_term = (depth_error - prev_depth_error) / period.toSec();
        
        // 对微分项应用低通滤波以减少噪声影响
        static double filtered_d_term = 0.0;
        filtered_d_term = 0.9 * filtered_d_term + 0.1 * depth_d_term;
        prev_depth_error = depth_error;
        
        // 添加阻尼项（PID中的D项），用于抑制震荡
        double damping_gain = 0.12; // 增加阻尼系数
        
        // 计算总的控制输出
        double position_adjustment = -(adaptive_p_gain * depth_error + 
                                      depth_i_gain_ * depth_error_integral_ + 
                                      damping_gain * filtered_d_term);
        
        // 限制单次位置调整量
        static double previous_adjustment = 0.0;
        double max_adjustment = 0.0003; // 最大单次调整为0.3mm（降低以提高稳定性）
        double raw_adjustment = std::max(std::min(position_adjustment, max_adjustment), -max_adjustment);
        
        // 使用非线性平滑函数进一步平滑控制输出
        double smoothing_factor = 0.85; // 增大平滑因子到0.85，更强的平滑效果
        double smoothed_adjustment = smoothing_factor * previous_adjustment + 
                                   (1.0 - smoothing_factor) * raw_adjustment;
        previous_adjustment = smoothed_adjustment;

        // 仅调整Z方向位置（深度方向）
        position_d_[2] += smoothed_adjustment;
        
        // 添加位置安全限制 - 防止z方向位置超出安全范围
        double min_allowed_z = soft_block_surface_z_ - 0.025; // 最大25mm深度
        double max_allowed_z = soft_block_surface_z_ + 0.05;  // 软块表面上方50mm
        position_d_[2] = std::max(std::min(position_d_[2], max_allowed_z), min_allowed_z);

        // 调试输出 - 使用英文消息
        if (circular_counter_ % 200 == 0) {
          ROS_INFO("Depth control: phase=%d, target=%.3f mm, current=%.3f mm, filtered=%.3f mm, error=%.3f mm, adjustment=%.3f mm",
                   static_cast<int>(control_phase_), 
                   target_depth_ * 1000.0, 
                   contact_depth_ * 1000.0, 
                   filtered_depth * 1000.0,
                   depth_error * 1000.0,
                   smoothed_adjustment * 1000.0);
        }
      } else {
        // 未接触状态下清除积分项，防止积分累积
        depth_error_integral_ = 0.0;
      }
    } else {
      // 力控制逻辑
      if (contact_detected_) {
        double force_error = target_force_ - measured_force.norm();
    
        // 更新积分项，应用积分窗口限制
        force_error_integral_ += force_error * period.toSec();
        force_error_integral_ = std::max(std::min(force_error_integral_, max_force_ * 0.5), -max_force_ * 0.5);
    
        // 自适应增益PID控制，在误差较小时减小增益
        double adaptive_p_gain = force_p_gain_;
        if (std::abs(force_error) < 1.0) { // 1N内使用更小的增益
          adaptive_p_gain = force_p_gain_ * 0.6;
        }
        
        // 计算力调整量 - 使用PID控制
        double force_adjustment = adaptive_p_gain * force_error + 
                                force_i_gain_ * force_error_integral_ +
                                force_d_gain_ * (force_error - prev_force_error_) / period.toSec();
        
        prev_force_error_ = force_error;
        
        // 将力调整转换为位置调整，限制单次调整量
        double position_adjustment = force_adjustment / 2000.0; // 假设刚度约为2000N/m
        double max_pos_adj = std::min(0.002, std::abs(force_error) * 0.0002); // 最大2mm，且与误差成比例
        position_adjustment = std::max(std::min(position_adjustment, max_pos_adj), -max_pos_adj);
      
        // 仅调整Z方向位置（力方向）
      position_d_[2] += position_adjustment;
      
        // 调试输出 - 使用英文消息
        if (circular_counter_ % 200 == 0) {
          ROS_INFO("Force control: phase=%d, target=%.2f N, current=%.2f N, error=%.2f N, adjustment=%.3f mm",
                   static_cast<int>(control_phase_), 
                   target_force_, 
                   measured_force.norm(), 
                   force_error,
                   position_adjustment * 1000.0);
    }
      } else {
        // 未接触状态下清除积分项
        force_error_integral_ = 0.0;
        prev_force_error_ = 0.0;
      }
    }
  }
  
  // 记录接触力和深度信息到日志
  if (contact_detected_ && contact_depth_ > 0.0) {
    // 当接触被检测到且深度为正值时记录数据
    log_generator_.logData(time, position, measured_force, static_cast<int>(control_phase_),
                          soft_block_surface_z_, target_force_, theoretical_force, 1);
    
    // 额外记录控制相关信息
    if (control_phase_ == DEPTH_CONTROL || control_phase_ == TRAJECTORY) {
      static int log_counter = 0;
      if (log_counter++ % 10 == 0) { // 每10次循环记录一次额外信息
        ROS_DEBUG("Control details: phase=%d, position_d=[%.4f, %.4f, %.4f], position=[%.4f, %.4f, %.4f], "
                 "depth=%.4f, z_error=%.4f, time=%.2f",
                 static_cast<int>(control_phase_),
                 position_d_(0), position_d_(1), position_d_(2),
                 position(0), position(1), position(2),
                 contact_depth_,
                 position_d_(2) - position(2),
                 elapsed_time_);
      }
    }
  } else if (measured_force.norm() > 0.5) {
    // 有力但未检测到接触时也记录数据（可能是接触抖动）
    log_generator_.logData(time, position, measured_force, static_cast<int>(control_phase_),
                          soft_block_surface_z_, target_force_, 0.0, 0);
  } else {
    // 未接触状态下记录最小力值
    Eigen::Vector3d min_force(0, 0, 0.1);
    log_generator_.logData(time, position, min_force, static_cast<int>(control_phase_),
                          soft_block_surface_z_, target_force_, 0.0, 0);
  }
  
  // 发布期望位姿
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = time;
  pose_msg.header.frame_id = "world";
  pose_msg.pose.position.x = position_d_(0);
  pose_msg.pose.position.y = position_d_(1);
  pose_msg.pose.position.z = position_d_(2);
  pose_msg.pose.orientation.x = orientation_d_.x();
  pose_msg.pose.orientation.y = orientation_d_.y();
  pose_msg.pose.orientation.z = orientation_d_.z();
  pose_msg.pose.orientation.w = orientation_d_.w();
  pose_pub_.publish(pose_msg);

  // =====================================================================
  // 5. 闭环控制计算
  // =====================================================================
  try {
    // 计算位置误差向量
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d_;  // 位置误差
    
    // 计算姿态误差
    // 确保四元数方向一致（避免绕远路旋转）
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    
    // 计算姿态误差四元数（表示从当前姿态到目标姿态的旋转）
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    
    // 提取误差四元数的虚部（表示旋转轴*sin(角度/2)）
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    
    // 将姿态误差转换到基坐标系
    error.tail(3) << -current_pose.rotation() * error.tail(3);
    
    // 限制误差幅度以避免控制不稳定
    for (int i = 0; i < 6; i++) {
      error(i) = std::max(std::min(error(i), 0.2), -0.2);
    }
    
    // 计算速度误差
    Eigen::Matrix<double, 6, 1> vel_error;
    vel_error.head(3) << v_ee.head(3) - position_d_dot;  // 线速度误差
    vel_error.tail(3) << v_ee.tail(3);  // 角速度误差（目标角速度为0）
    
    // 笛卡尔空间PD控制 (P: 刚度, D: 阻尼)
    Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

    // 计算任务空间力矩：-K*x - D*dx (K: 刚度, D: 阻尼, x: 位置误差, dx: 速度)
    tau_task << jacobian.transpose() *
                (-cartesian_stiffness_ * error - cartesian_damping_ * vel_error);
    
    // 零空间控制：保持初始的关节配置
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    // 零空间控制：使用阻尼比为1的PD控制保持初始关节角度
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                      (nullspace_stiffness_ * (initial_q_ - q_const) -
                          (2.0 * sqrt(nullspace_stiffness_)) * dq_const);
    
    // 最终控制输入：任务空间力 + 零空间力 + 重力补偿
    tau_d << tau_task + tau_nullspace + coriolis;
    
    // 限制力矩变化率，避免不连续性
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    
    // 应用控制输入
    for (size_t i = 0; i < 7; ++i) {
      joint_handles_[i].setCommand(tau_d(i));
    }
    
    // 监控输出（仅用于调试）
    static int motion_counter = 0;
    if (motion_counter++ % 1000 == 0) {  // 约1s打印一次
      ROS_INFO_STREAM("=== Motion Control Status ===");
      ROS_INFO_STREAM("Control phase: " << (control_phase_ == APPROACH ? "approaching" : 
                                    (control_phase_ == CONTACT_DETECTION ? "contact detection" : 
                                     (control_phase_ == DEPTH_CONTROL ? "depth control" : "trajectory"))));
      ROS_INFO_STREAM("Current position: [" << position.transpose() << "]");
      ROS_INFO_STREAM("Desired position: [" << position_d_.transpose() << "]");
      ROS_INFO_STREAM("Position error: [" << error.head(3).transpose() << "]");
      ROS_INFO_STREAM("Position error norm: " << error.head(3).norm());
      
      ROS_INFO_STREAM("Orientation error: [" << error.tail(3).transpose() << "]");
      ROS_INFO_STREAM("Orientation error norm: " << error.tail(3).norm());
      
      if (current_contact_state.in_contact) {
        ROS_INFO_STREAM("Contact state: In contact");
        ROS_INFO_STREAM("Contact position: [" << current_contact_state.position().transpose() << "]");
        ROS_INFO_STREAM("Measured force: [" << measured_force.transpose() << "]");
        ROS_INFO_STREAM("Measured force magnitude: " << measured_force.norm() << " N");
        ROS_INFO_STREAM("Contact depth: " << current_contact_state.depth() * 1000 << " mm");
        ROS_INFO_STREAM("Target force: " << target_force_ << " N");
        ROS_INFO_STREAM("Force error: " << target_force_ - (measured_force.norm() > 0.1 ? measured_force.norm() : 0.0) << " N");
      } else {
        ROS_INFO_STREAM("Contact state: Not in contact");
        ROS_INFO_STREAM("Distance to soft block: " << (end_position(2) - soft_block_surface_z_) << " m");
      }
      
      if (control_phase_ == TRAJECTORY) {
        double angle = 2.0 * M_PI * circle_frequency_ * elapsed_time_;
        ROS_INFO_STREAM("Circle angle: " << fmod(angle, 2.0 * M_PI) << " rad");
      }
      
      ROS_INFO_STREAM("Elapsed time: " << elapsed_time_ << " s");
      ROS_INFO_STREAM("===============================");
    }
  } catch (const std::exception& ex) {
    // 捕获并记录任何控制过程中的异常
    ROS_ERROR_STREAM("Exception during control process: " << ex.what());
  }
}

/**
 * @brief 限制力矩变化率，避免不连续性
 * 
 * @param tau_d_calculated 计算得到的期望力矩
 * @param tau_J_d 上一周期的期望力矩
 * @return Eigen::Matrix<double, 7, 1> 限制变化率后的力矩
 */
Eigen::Matrix<double, 7, 1> CircleController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    // 计算力矩变化量
    double difference = tau_d_calculated[i] - tau_J_d[i];
    // 限制变化量在 ±delta_tau_max_ 范围内
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CircleController,
                       controller_interface::ControllerBase)

