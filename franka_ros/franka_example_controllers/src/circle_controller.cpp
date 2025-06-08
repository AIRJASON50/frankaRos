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
    return false;
  }
  
  // 获取关节名称参数
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    return false;
  }

  // 创建轨迹生成器
  trajectory_generator_ = std::make_unique<TrajectoryGenerator>();
  
  // 初始化轨迹生成器
  if (!trajectory_generator_->init(node_handle, path_pub_)) {
    return false;
  }
  
  // 设置目标深度
  trajectory_generator_->setTargetDepth(target_depth_);
  
  // 读取轨迹参数
  node_handle.param<double>("circle_frequency", circle_frequency_, 0.3);
  node_handle.param<double>("circle_radius", circle_radius_, 0.1);
  node_handle.param<std::string>("circle_plane", circle_plane_, "xy");
  
  // 初始化路径发布器
  path_pub_ = node_handle.advertise<nav_msgs::Path>("trajectory_path", 1);
  
  // 读取软接触模型参数
  ros::NodeHandle soft_contact_nh(node_handle, "contact_model");
  contact_params_.young_modulus = 300000.0;  // 默认值：软块300kPa
  contact_params_.poisson_ratio = 0.45;    // 默认值：软块泊松比
  contact_params_.probe_young_modulus = 200000000000.0;  // 默认值：探头200GPa
  contact_params_.probe_poisson_ratio = 0.3;  // 默认值：探头泊松比
  contact_params_.friction_coef = 0.3;     // 默认值：中等摩擦系数
  contact_params_.contact_radius = 0.01;   // 默认值：1cm半径的探头
  contact_params_.path_radius = 0.1;       // 默认值：10cm圆周路径半径
  contact_params_.damping = 50.0;          // 默认值：中等阻尼
  contact_params_.force_threshold = 3.0;   // 默认值：3N接触力阈值
  contact_params_.depth_threshold = 0.001; // 默认值：1mm接触深度阈值
  
  soft_contact_nh.getParam("young_modulus", contact_params_.young_modulus);
  soft_contact_nh.getParam("poisson_ratio", contact_params_.poisson_ratio);
  soft_contact_nh.getParam("probe_young_modulus", contact_params_.probe_young_modulus);
  soft_contact_nh.getParam("probe_poisson_ratio", contact_params_.probe_poisson_ratio);
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
    return false;
  }
  
  // 计算软块中心位置向量
  soft_block_position_ = Eigen::Vector3d(soft_block_x_, soft_block_y_, soft_block_z_);
  
  // 读取探头长度参数
  if (!node_handle.getParam("probe/length", probe_length_)) {
    ROS_WARN("Failed to read probe/length, using default 0.114");
    probe_length_ = 0.114;
  }
  
  // 在init函数中添加控制模式读取
  int control_mode_param = 0;
  if (node_handle.getParam("control_mode", control_mode_param)) {
    if (control_mode_param == 1) {
      control_mode_ = FORCE_CONTROL_MODE;
    } else {
      control_mode_ = DEPTH_CONTROL_MODE;
    }
  } else {
    // 尝试读取字符串格式的控制模式（向后兼容）
    std::string control_mode_str;
    if (node_handle.getParam("control_mode", control_mode_str)) {
      if (control_mode_str == "force") {
        control_mode_ = FORCE_CONTROL_MODE;
      } else {
        control_mode_ = DEPTH_CONTROL_MODE;
      }
    } else {
      // 默认使用深度控制模式
      control_mode_ = DEPTH_CONTROL_MODE;
    }
  }

  // 读取深度控制参数
  ros::NodeHandle depth_nh(node_handle, "depth_controller");
  depth_nh.getParam("target_depth", target_depth_);
  depth_nh.getParam("p_gain", depth_p_gain_);
  depth_nh.getParam("i_gain", depth_i_gain_);
  depth_nh.getParam("d_gain", depth_d_gain_);
  depth_nh.getParam("max_adjustment", depth_max_adjustment_);
  depth_nh.getParam("stability_threshold", depth_stability_threshold_);
  depth_nh.getParam("stability_duration", depth_stability_duration_);
  
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
  
  // 读取调零参数
  ros::NodeHandle calibration_nh(node_handle, "calibration");
  calibration_nh.getParam("mean_force_threshold", calibration_mean_force_threshold_);
  calibration_nh.getParam("variance_threshold", force_variance_threshold_);
  calibration_nh.getParam("stability_duration", calibration_stability_duration_);
  
  // 读取接触检测参数
  ros::NodeHandle contact_detection_nh(node_handle, "contact_detection");
  contact_detection_nh.getParam("force_threshold", force_threshold_);
  contact_detection_nh.getParam("confirmation_duration", contact_confirmation_duration_);
  
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
    return false;
  }
  
  // 尝试获取模型句柄
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    return false;
  }

  // 获取机器人状态接口
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    return false;
  }
  
  // 尝试获取状态句柄
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    return false;
  }

  // 获取关节力矩控制接口
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    return false;
  }
  
  // 获取每个关节的控制句柄
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
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
    return false;
  }
  
  // 保存噪声参数以供其他地方使用
  force_nh.getParam("force_noise_enable", force_noise_enable_);

  // 设置轨迹生成器的软块参数
  if (trajectory_generator_) {
    trajectory_generator_->setCenter(position_d_);
    trajectory_generator_->setContactRequired(false); // 校准和接近阶段不需要接触
    trajectory_generator_->setPreserveHeight(true);  // 校准和接近阶段保持高度
  }

  // 初始化能量罐监控器
  energy_tank_monitor_ = std::make_unique<EnergyTankMonitor>();
  if (!energy_tank_monitor_->init(node_handle)) {
    ROS_ERROR("Failed to initialize energy tank monitor");
    return false;
  }

  // 初始化成功
  return true;
}

void CircleController::starting(const ros::Time& time) {
  // 获取初始状态
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // 确保 initial_position_ 捕获的是机器人启动时的实际法兰位置
  initial_position_ = initial_transform.translation(); 
  circle_center_ = initial_position_; // 圆周中心也设为初始位置
  
  position_d_ = initial_position_; 
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  q_d_nullspace_ = q_initial;
  elapsed_time_ = 0.0;
  
  // 严格从CALIBRATION阶段开始
  control_phase_ = CALIBRATION;
  phase_start_time_ = time;
  
  // 重置力传感器调零相关变量
  force_offset_ = Eigen::Vector3d::Zero();
  calibration_completed_ = false;
  calibration_sample_count_ = 0;
  calibration_attempts_ = 0;
  force_samples_.clear();
  calibration_stable_since_ = ros::Time(0); // 初始化为无效时间
  calibration_criteria_met_ = false;
  
  // 重置深度控制相关变量
  depth_error_integral_ = 0.0;
  prev_depth_error_ = 0.0;
  depth_stable_since_ = ros::Time(0); // 初始化为无效时间
  depth_criteria_met_ = false;
  
  // 重置力控制变量
  force_error_integral_ = 0.0;
  prev_force_error_ = 0.0;
  last_force_error_ = 0.0;
  
  // 重置接触状态
  contact_detected_ = false;
  contact_position_ = Eigen::Vector3d::Zero();
  contact_depth_ = 0.0;
  contact_reference_z_ = 0.0; 
  
  if (trajectory_generator_) {
    trajectory_generator_->setCenter(position_d_);
    trajectory_generator_->setContactRequired(false); // 校准和接近阶段不需要接触
    trajectory_generator_->setPreserveHeight(true);  // 校准和接近阶段保持高度
  }
  
  std_msgs::String phase_msg;
  phase_msg.data = "CALIBRATION";
  phase_pub_.publish(phase_msg);
  
  circular_counter_ = 0;
  desired_pose_initialized_ = false; // 将在第一次update中初始化
  
  // 重置能量罐监控器
  if (energy_tank_monitor_) {
    energy_tank_monitor_->reset();
    ROS_INFO("Energy tank monitor reset");
  }
}

void CircleController::update(const ros::Time& time,
                             const ros::Duration& period) {
  // 获取机器人状态
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 16> O_T_EE_array = robot_state.O_T_EE;
  std::array<double, 49> mass_array = model_handle_->getMass();
  
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(O_T_EE_array.data()));
  
  // 更新通用位置和力信息
  current_flange_position_ = current_transform.translation();
  current_probe_tip_position_ = current_flange_position_;
  current_probe_tip_position_(2) -= probe_length_;

  Eigen::Map<const Eigen::Matrix<double, 6, 1>> external_wrench_raw(robot_state.O_F_ext_hat_K.data());
  Eigen::Vector3d measured_force_raw = external_wrench_raw.head(3);
  
  // 总是使用校准后的Z轴力值，即使在校准过程中
  Eigen::Vector3d current_calibrated_force_full = measured_force_raw;
  if (calibration_completed_) {
    // 如果校准完成，使用保存的偏移值
    current_calibrated_force_full -= force_offset_;
  } else if (calibration_sample_count_ > 0 && !force_samples_.empty()) {
    // 在校准进行中，使用当前计算的偏移估计值
    Eigen::Vector3d current_offset = Eigen::Vector3d::Zero();
    for (const auto& sample : force_samples_) {
      current_offset += sample;
    }
    current_offset /= static_cast<double>(force_samples_.size());
    current_calibrated_force_full -= current_offset;
  }
  
  // 更新校准后的Z轴力值，供状态机和其他功能使用
  current_calibrated_force_z_ = current_calibrated_force_full(2);

  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

  // 第一次update时初始化期望姿态和轨迹生成器的轴
  if (!desired_pose_initialized_) {
    desired_pose_ = current_transform;
    // ... (确保Z轴向下, 计算circle_normal_, circle_x_axis_, circle_y_axis_ 的逻辑保持不变)
    Eigen::Vector3d z_axis = desired_pose_.rotation().col(2);
    if (z_axis(2) > 0) {
      Eigen::Matrix3d rotation_matrix = desired_pose_.rotation();
      Eigen::Matrix3d flip_rotation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
      rotation_matrix = flip_rotation * rotation_matrix;
      desired_pose_.linear() = rotation_matrix;
    }
    circle_normal_ = desired_pose_.rotation().col(2).normalized();
    Eigen::Vector3d reference = (std::abs(circle_normal_(0)) < 0.9) ? 
        Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    circle_x_axis_ = circle_normal_.cross(reference).normalized();
    circle_y_axis_ = circle_normal_.cross(circle_x_axis_).normalized();
    
    if (trajectory_generator_) {
        trajectory_generator_->setCenter(circle_center_); // circle_center_ 已在starting()中设为initial_position_
    trajectory_generator_->setAxis(circle_x_axis_, circle_y_axis_);
    }
    initial_q_ = q; // 保存初始关节角
    orientation_d_ = Eigen::Quaterniond(desired_pose_.rotation()); // 设置初始期望方向
    desired_pose_initialized_ = true;
  }
  
  elapsed_time_ += period.toSec(); // 全局时间累加

  // 阶段状态机
  ControlPhase previous_phase = control_phase_;

  switch (control_phase_) {
    case CALIBRATION:
      position_d_ = initial_position_; // 保持在初始法兰位置
      orientation_d_ = Eigen::Quaterniond(desired_pose_.rotation()); // 保持初始方向

      if (!calibration_completed_) {
        if (calibration_sample_count_ < calibration_required_samples_) {
          force_samples_.push_back(measured_force_raw); // 使用原始力进行校准
          calibration_sample_count_++;
          
          // 显示校准样本收集进度
          if (calibration_sample_count_ % 100 == 0 || calibration_sample_count_ == calibration_required_samples_) {
            ROS_INFO("Calibration progress: collected %d/%d samples", 
                     calibration_sample_count_, calibration_required_samples_);
          }
        } else {
          // 计算力传感器零点偏移
          force_offset_ = Eigen::Vector3d::Zero();
          for (const auto& sample : force_samples_) {
            force_offset_ += sample;
          }
          force_offset_ /= static_cast<double>(force_samples_.size());
  
          // 计算校准后的力值 - 这才是我们关心的实际力值
          Eigen::Vector3d calibrated_force = measured_force_raw - force_offset_;
  
          // 计算校准后力值的方差，用于评估稳定性
          Eigen::Vector3d variance = Eigen::Vector3d::Zero();
          for (const auto& sample : force_samples_) {
            Eigen::Vector3d diff = sample - force_offset_; // 使用校准后的力值计算方差
            variance += diff.cwiseProduct(diff);
          }
          variance /= static_cast<double>(force_samples_.size());
    
          // 校准条件检查 - 使用配置文件中的阈值
          bool mean_z_ok = std::abs(calibrated_force(2)) < calibration_mean_force_threshold_;
          bool variance_ok = std::sqrt(variance(2)) < force_variance_threshold_;
          
          // 添加调试输出显示校准进度
          if (calibration_attempts_ % 50 == 0) { // 每50次输出一次调试信息
            ROS_INFO("Calibration progress: mean_force_z=%.3f (threshold=%.3f), variance=%.3f (threshold=%.3f)", 
                     std::abs(calibrated_force(2)), calibration_mean_force_threshold_, 
                     std::sqrt(variance(2)), force_variance_threshold_);
          }
          
          // 只有当校准后的Z轴力值足够小且稳定时，才认为校准成功
          if (mean_z_ok && variance_ok) {
            if (calibration_stable_since_.is_zero()) {
              calibration_stable_since_ = time;
    }
            // 调零稳定1秒后完成校准
            if ((time - calibration_stable_since_).toSec() >= calibration_stability_duration_) {
              calibration_completed_ = true;
              calibration_criteria_met_ = true; // 标记校准完成
              
              // 接近阶段开始
              ROS_INFO("Calibration completed, starting APPROACH phase");
              control_phase_ = APPROACH;
              phase_start_time_ = time;
              approach_start_movement_time_ = time; // 开始接近运动的时间记录
              initial_approach_position_ = current_flange_position_; // 记录接近开始时的位置
              // 删除基于软块高度的接近距离计算，改为匀速下降
              contact_detected_ = false;
              contact_reference_z_ = 0.0;
              ROS_INFO("Starting approach phase from position: [%.3f, %.3f, %.3f]", 
                       initial_approach_position_(0), initial_approach_position_(1), initial_approach_position_(2));
    }
          } else {
            calibration_stable_since_ = ros::Time(0); // 条件不满足，重置稳定计时器
            // 如果多次尝试后仍不稳定，简单实现：重置样本计数，重新采样
            if (calibration_attempts_++ % calibration_required_samples_ == 0) {
              force_samples_.clear();
              calibration_sample_count_ = 0;
    }
          }
        }
      } else {
        // 如果校准还没完成，继续校准
          initial_approach_position_ = current_flange_position_;
        // 删除基于软块高度的接近距离计算
        approach_start_movement_time_ = time;
        contact_detected_ = false;
        contact_reference_z_ = 0.0;
          ROS_INFO("Calibration was already completed, ensuring we enter APPROACH phase");
  }
      break;

    case APPROACH:
      // 接近阶段：匀速向下移动，直到检测到足够的力
      position_d_(0) = initial_approach_position_(0); // 保持XY位置不变
      position_d_(1) = initial_approach_position_(1);
    
      // 匀速向下移动 (速度为approach_speed_ m/s)
      position_d_(2) = initial_approach_position_(2) - approach_speed_ * (time - phase_start_time_).toSec();
    
      // 延迟检测：确保机械臂开始下降至少0.5秒后才检测力值，避免调零后的瞬态效应
      if ((time - approach_start_movement_time_).toSec() >= 0.5) {
        // 仅使用Z轴校准后的力值检测接触
        if (current_calibrated_force_z_ > 0.5 && calibration_completed_) { // Z轴校准后力值大于0.5N表示接触
          control_phase_ = CONTACT_CONFIRMATION; // 切换到接触确认阶段
          phase_start_time_ = time;
          contact_confirmation_start_time_ = time;
          contact_detected_ = true;
          position_d_ = current_flange_position_; // 保持当前位置，不再下压
          
          // 输出调试信息
          ROS_INFO("Contact detected with force: %f N, switching to CONTACT_CONFIRMATION phase", current_calibrated_force_z_);
        }
      }
      break;

    case CONTACT_CONFIRMATION:
      // 接触确认阶段：记录接触点作为深度零点，短暂停留后切换
      position_d_ = current_flange_position_; // 保持不动
      
      // 首次进入时记录接触参考点
      if (contact_reference_z_ == 0.0) {
        contact_reference_z_ = current_probe_tip_position_(2); // 记录探头末端位置作为深度零点
        contact_position_ = current_flange_position_; // 记录法兰位置
        ROS_INFO("Contact reference point set: z=%f, calibrated force=%f N", 
                 contact_reference_z_, current_calibrated_force_z_);
      }
      
      // 短暂停留0.5秒后切换到深度控制阶段
      if ((time - contact_confirmation_start_time_).toSec() >= contact_confirmation_duration_) {
      control_phase_ = DEPTH_CONTROL;
      phase_start_time_ = time;
        depth_error_integral_ = 0.0; // 重置PID控制器
        prev_depth_error_ = 0.0;
        depth_stable_since_ = ros::Time(0);
        depth_criteria_met_ = false;
        // 初始化深度控制的期望位置为当前法兰位置
        position_d_ = current_flange_position_;
        ROS_INFO("Contact confirmed, switching to DEPTH_CONTROL phase");
    }
      break;

    case DEPTH_CONTROL:
      // 深度控制阶段：下压到目标深度，稳定后切换
      
      // 计算深度：接触参考点与当前探头末端的距离
      // 移除负深度限制，允许深度为负（探头高于参考点时）
      contact_depth_ = contact_reference_z_ - current_probe_tip_position_(2);
      
      // 添加调试信息
      if (circular_counter_++ % 50 == 0) { // 每50次循环输出一次调试信息
        ROS_INFO("DEPTH_CONTROL: current_depth=%.4f mm, target=%.4f mm, error=%.4f mm", 
                 contact_depth_ * 1000, target_depth_ * 1000, (target_depth_ - contact_depth_) * 1000);
        ROS_INFO("  probe_tip_z=%.6f, contact_ref_z=%.6f, position_d_z=%.6f", 
                 current_probe_tip_position_(2), contact_reference_z_, position_d_(2));
      }
      
      // PID控制深度
      {
    double depth_error = target_depth_ - contact_depth_;
        depth_error_integral_ += depth_error * period.toSec();
    
        // 积分限幅
        double integral_limit = 0.001; // 增大积分限幅
        depth_error_integral_ = std::max(std::min(depth_error_integral_, integral_limit), -integral_limit);
        
        double depth_d_term = (period.toSec() > 1e-6) ? (depth_error - prev_depth_error_) / period.toSec() : 0.0;
        prev_depth_error_ = depth_error;
      
        // PID控制参数
        double p_gain = depth_p_gain_;
        double i_gain = depth_i_gain_;
        double d_gain = depth_d_gain_;
        
        double adjustment = p_gain * depth_error + i_gain * depth_error_integral_ + d_gain * depth_d_term;
        adjustment = std::max(std::min(adjustment, depth_max_adjustment_), -depth_max_adjustment_);
        
        // 向下调整为负方向（增加深度）
        // 当depth_error > 0时（当前深度小于目标深度），需要下压，position_d_(2)减小
        position_d_(2) -= adjustment;
        
        // 调试输出PID控制信息
        if (circular_counter_ % 50 == 0) {
          ROS_INFO("  PID: P=%.6f, I=%.6f, D=%.6f, adj=%.6f", 
                   p_gain * depth_error, i_gain * depth_error_integral_, d_gain * depth_d_term, adjustment);
        }
      }
      
      // 检查深度是否稳定在目标值附近
      if (std::abs(contact_depth_ - target_depth_) < depth_stability_threshold_) {
        if (depth_stable_since_.is_zero()) {
          depth_stable_since_ = time;
          ROS_INFO("Depth stabilizing at: %.3f mm, target: %.3f mm", contact_depth_ * 1000, target_depth_ * 1000);
    }
        // 稳定指定时间后切换到轨迹阶段
        if ((time - depth_stable_since_).toSec() >= depth_stability_duration_) {
          depth_criteria_met_ = true;
      control_phase_ = TRAJECTORY;
      phase_start_time_ = time;
          elapsed_time_ = 0.0; // 重置轨迹生成器时间
          
          if (trajectory_generator_) {
            trajectory_generator_->setCenter(current_flange_position_);
            trajectory_generator_->setContactRequired(true);
            trajectory_generator_->setPreserveHeight(false);
    }
          
          ROS_INFO("Depth stable for %.1f seconds, switching to TRAJECTORY phase", depth_stability_duration_);
        }
      } else {
        depth_stable_since_ = ros::Time(0); // 不稳定，重置计时器
      }
      break;

    case TRAJECTORY:
      // 轨迹阶段：按轨迹运动同时保持深度
      // 移除失去接触的返回处理
      
      // XY平面轨迹生成
      if (trajectory_generator_) {
        Eigen::Vector3d trajectory_xy_position = trajectory_generator_->generateTrajectory(
            elapsed_time_,
            contact_detected_,
            contact_depth_
        );
        position_d_(0) = trajectory_xy_position(0);
        position_d_(1) = trajectory_xy_position(1);
      }
      
      // 继续保持深度控制
      contact_depth_ = contact_reference_z_ - current_probe_tip_position_(2);
      // 移除负深度限制，允许深度为负（探头高于参考点时）
    
      // 与DEPTH_CONTROL阶段使用相同的PID控制逻辑
      {
    double depth_error = target_depth_ - contact_depth_;
        depth_error_integral_ += depth_error * period.toSec();
        double integral_limit = 0.0005;
        depth_error_integral_ = std::max(std::min(depth_error_integral_, integral_limit), -integral_limit);
        
        double depth_d_term = (period.toSec() > 1e-6) ? (depth_error - prev_depth_error_) / period.toSec() : 0.0;
        prev_depth_error_ = depth_error;
        
        double p_gain = depth_p_gain_;
        double i_gain = depth_i_gain_;
        double d_gain = depth_d_gain_;
        
        double adjustment = p_gain * depth_error + i_gain * depth_error_integral_ + d_gain * depth_d_term;
        adjustment = std::max(std::min(adjustment, depth_max_adjustment_), -depth_max_adjustment_);
        
        position_d_(2) -= adjustment;
      }
      break;
      }
      
  // 发布阶段变化信息
  if (previous_phase != control_phase_) {
    std_msgs::String phase_msg_out;
    switch (control_phase_) {
      case CALIBRATION: phase_msg_out.data = "CALIBRATION"; break;
      case APPROACH: phase_msg_out.data = "APPROACH"; break;
      case CONTACT_CONFIRMATION: phase_msg_out.data = "CONTACT_CONFIRMATION"; break;
      case DEPTH_CONTROL: phase_msg_out.data = "DEPTH_CONTROL"; break;
      case TRAJECTORY: phase_msg_out.data = "TRAJECTORY"; break;
    }
    phase_pub_.publish(phase_msg_out);
    // ROS_INFO("Switched from %d to %d", static_cast<int>(previous_phase), static_cast<int>(control_phase_));
    }
    
  // 记录日志数据
  // 获取理论力数据 - 修改：始终计算理论力，不管是否检测到接触
  double theoretical_force = 0.0;
  if (contact_model_) {
    // 修改：使用当前深度值计算理论力，即使深度为负或零
    double current_depth = contact_detected_ ? contact_depth_ : 0.0;
    // 如果深度大于0，计算理论力
    if (current_depth > 0) {
      theoretical_force = contact_model_->computeNormalForce(current_depth);
    }
    // 即使深度为0或负值，也发布数据（理论力为0）
  }
  
  // 使用校准后的完整力向量
  log_generator_.logData(time, current_flange_position_, current_calibrated_force_full, 
                        static_cast<int>(control_phase_),
                        contact_detected_ ? contact_reference_z_ : 0.0, theoretical_force, 
                        probe_length_, contact_reference_z_, measured_force_raw(2));
  // 确保在log_generator中深度是基于 contact_reference_z_ 和 current_probe_tip_position_ 计算或直接传递contact_depth_
  log_generator_.setCurrentDepth(contact_detected_ ? contact_depth_ : 0.0);

  
  // 发布期望位姿
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = time;
  pose_msg.header.frame_id = "world"; // 或者其他合适的frame
  pose_msg.pose.position.x = position_d_(0);
  pose_msg.pose.position.y = position_d_(1);
  pose_msg.pose.position.z = position_d_(2);
  pose_msg.pose.orientation.x = orientation_d_.x();
  pose_msg.pose.orientation.y = orientation_d_.y();
  pose_msg.pose.orientation.z = orientation_d_.z();
  pose_msg.pose.orientation.w = orientation_d_.w();
  pose_pub_.publish(pose_msg);

  // 闭环控制计算 (末端位置和姿态控制)
    Eigen::Matrix<double, 6, 1> error;
  error.head(3) << current_flange_position_ - position_d_; // 位置误差
    
  Eigen::Quaterniond current_orientation(current_transform.rotation());
  if (orientation_d_.coeffs().dot(current_orientation.coeffs()) < 0.0) {
    current_orientation.coeffs() << -current_orientation.coeffs();
    }
  Eigen::Quaterniond error_quaternion(current_orientation.inverse() * orientation_d_);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail(3) << -current_transform.rotation() * error.tail(3); // 转换到基坐标系
  
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d_final(7);
  // 速度误差，假设期望速度为0 （对于准静态运动）
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_map(robot_state.dq.data());
  Eigen::Matrix<double, 6, 1> velocity_error = jacobian * dq_map;


    tau_task << jacobian.transpose() *
              (-cartesian_stiffness_ * error - cartesian_damping_ * velocity_error);
    
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                    (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                     (2.0 * sqrt(nullspace_stiffness_)) * dq);
    
  tau_d_final << tau_task + tau_nullspace + coriolis;
  tau_d_final << saturateTorqueRate(tau_d_final, tau_J_d);
  
  // 能量罐安全监控和缩放
  double safety_scale_factor = 1.0;
  if (energy_tank_monitor_) {
    safety_scale_factor = energy_tank_monitor_->updateAndGetScaleFactor(
      tau_d_final, robot_state, jacobian, period);
    
    // 应用安全缩放因子
    tau_d_final *= safety_scale_factor;
    
    // 输出安全状态信息
    if (safety_scale_factor < 1.0) {
      auto safety_level = energy_tank_monitor_->getCurrentSafetyLevel();
      if (circular_counter_ % 100 == 0) { // 每100次循环输出一次
        ROS_WARN("Energy tank safety scaling active: factor=%.2f, level=%d, energy=%.2f J", 
                 safety_scale_factor, static_cast<int>(safety_level), 
                 energy_tank_monitor_->getEnergyLevel());
      }
    }
  }
    
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_final(i));
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

