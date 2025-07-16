/**
 * @file contact_controller.cpp
 * @brief 接触控制器实现，集成前进运动、圆周运动、力传感器监控（简化版）
 * 
 * 实现功能：
 * 1. 先向前运动到目标位置
 * 2. 然后进行半径2.5cm的圆周运动
 * 3. 实时力传感器数据获取和处理
 * 4. 能量罐安全监控
 */

#include <franka_example_controllers/contact_controller.h>
#include <franka_example_controllers/log_generator.h>
#include <franka_example_controllers/energy_tank_monitor.h>
#include <franka_example_controllers/soft_contact_model.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

bool ContactController::init(hardware_interface::RobotHW* robot_hw,
                           ros::NodeHandle& node_handle) {
  // Get robot ID parameter
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ContactController: Could not get parameter arm_id");
    return false;
  }
  
  // Get Cartesian position control interface
  cartesian_pose_interface_ = robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR("ContactController: Could not get Cartesian pose interface");
    return false;
  }
  
  // Get position control handle
  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("ContactController: Exception getting Cartesian handle: " << ex.what());
    return false;
  }

  // Get robot state interface
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("ContactController: Could not get state interface");
    return false;
  }
  
  // Get state handle
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("ContactController: Exception getting state handle: " << ex.what());
    return false;
  }

  // Check robot starting position (same as CartesianPoseExampleController)
  // ✅ 临时注释掉位置检查，允许从任何位置启动进行日志测试
  /*
  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.3) {  // 放宽容限到0.3弧度
        ROS_ERROR_STREAM(
            "ContactController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        ROS_ERROR_STREAM("Joint " << i << ": current=" << state_handle.getRobotState().q_d[i] 
                         << ", expected=" << q_start[i] << ", error=" 
                         << std::abs(state_handle.getRobotState().q_d[i] - q_start[i]));
        return false;
      }
    }
    ROS_INFO("ContactController: Starting position check passed");
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "ContactController: Exception getting state handle for position check: " << e.what());
    return false;
  }
  */
  ROS_INFO("ContactController: Starting position check bypassed for logging test");

  // Get robot model interface (for Jacobian calculation)
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR("ContactController: Could not get model interface");
    return false;
  }
  
  // Get model handle
    try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("ContactController: Exception getting model handle: " << ex.what());
      return false;
  }

  // Initialize ROS publishers and subscribers
  pose_pub_ = node_handle.advertise<geometry_msgs::PoseStamped>("equilibrium_pose", 10);
  phase_pub_ = node_handle.advertise<std_msgs::String>("control_phase", 10);
  energy_pub_ = node_handle.advertise<std_msgs::Float64>("current_energy", 10);
  
  // Subscribe to force sensor data
  force_sub_ = node_handle.subscribe<geometry_msgs::WrenchStamped>(
      "/force_sensor/wrench", 10, &ContactController::forceCallback, this);
  
  // Subscribe to raw force sensor data for future use
  raw_force_sub_ = node_handle.subscribe<geometry_msgs::WrenchStamped>(
      "/force_sensor/raw_wrench", 10, &ContactController::rawForceCallback, this);
  
  // Read motion parameters
  ros::NodeHandle motion_nh(node_handle, "motion_control");
  motion_nh.param("circle_radius", circle_radius_, 0.025);  // 2.5cm radius
  motion_nh.param("motion_frequency", motion_frequency_, 0.2);  // 0.2 Hz
  motion_nh.param("probe_length", probe_length_, 0.1);  // 10cm probe length

  // 设置目标位置（当前机械臂末端位置）
  target_position_ << 0.483835, 0.089474, 0.193932;
  
  // Initialize log generator (interface only, not used)
  log_generator_ = std::make_unique<LogGenerator>();
  
  // Initialize energy tank monitor
  energy_monitor_ = std::make_unique<EnergyTankMonitor>();
  if (!energy_monitor_->init(node_handle)) {
    ROS_ERROR("ContactController: Failed to initialize energy tank monitor");
    return false;
  }

  // Initialize state variables
  control_phase_ = CALIBRATION;
  force_data_available_ = false;
  
  // Initialize force sensor related variables
  current_force_.setZero();
  raw_force_.setZero();
  
  // Initialize force zeroing variables
  force_zeroing_complete_ = false;
  force_offset_.setZero();
  force_window_buffer_.clear();
  
  // ==== 优化后的力传感器滤波系统参数 ====
  force_ema_alpha_ = 0.4;               // 提高alpha值，减少延迟（从0.15提高到0.4）
  force_window_size_ = 5;               // 减小滑动窗口（从10减到5）
  contact_detection_alpha_ = 0.6;       // 接触检测时使用更快响应（从0.3提高到0.6）
  
  // 初始化滤波器状态
  force_filter_initialized_ = false;
  contact_filter_initialized_ = false;
  filtered_force_ = Eigen::Vector3d::Zero();
  contact_filtered_force_ = Eigen::Vector3d::Zero();
  force_history_buffer_.clear();
  
  ROS_INFO("ContactController: Enhanced force filtering initialized - EMA_alpha=%.3f, Window_size=%d",
           force_ema_alpha_, force_window_size_);
  // ========================================
  
  // ==== 初始化z轴力反馈控制系统 ====
  target_force_z_ = 0.0;
  force_feedback_enabled_ = true;         // 默认启用力反馈控制
  target_force_recorded_ = false;
  
  // PD控制器参数（可通过参数服务器配置）- 更保守的设置
  ros::NodeHandle force_control_nh(node_handle, "force_control");
  force_control_nh.param("kp", force_kp_, 0.0005);       // 比例增益：进一步降低（从0.001降至0.0005）
  force_control_nh.param("kd", force_kd_, 0.0002);       // 微分增益：进一步降低（从0.0005降至0.0002）
  force_control_nh.param("z_offset_max", z_offset_max_, 0.003);    // 最大上升3mm（从5mm降至3mm）
  force_control_nh.param("z_offset_min", z_offset_min_, -0.003);   // 最大下降3mm（从5mm降至3mm）
  
  // 新增平滑控制参数 - 更保守的设置
  force_control_nh.param("max_change_per_step", max_change_per_step_, 0.00005);  // 单次最大变化0.05mm（从0.2mm降至0.05mm）
  force_control_nh.param("force_deadzone", force_deadzone_, 0.1);               // 死区0.1N（从0.05N增至0.1N）
  force_control_nh.param("smooth_alpha", smooth_alpha_, 0.02);                  // 更强的平滑滤波（从0.05降至0.02）
  
  force_error_prev_ = 0.0;
  current_z_offset_ = 0.0;
  
  // ✅ 初始化力控制渐进启动参数
  force_control_gradual_startup_ = true;
  force_control_init_time_ = ros::Time(0);
  force_control_startup_duration_ = 3.0;  // 3秒渐进启动
  
  ROS_INFO("ContactController: Force feedback control initialized - Kp=%.6f, Kd=%.6f, Z_range=[%.3f, %.3f]m",
           force_kp_, force_kd_, z_offset_min_, z_offset_max_);
  ROS_INFO("ContactController: Smooth control - MaxChange=%.2fmm, Deadzone=%.3fN, Alpha=%.3f",
           max_change_per_step_ * 1000, force_deadzone_, smooth_alpha_);
  // =========================================
  
  // Initialize user input variables
  waiting_for_user_command_ = false;
  user_input_ready_ = false;
  user_input_thread_started_ = false;
  
  // Contact parameters structure (for future use)
  contact_params_.young_modulus = 10000.0;  // Pa
  contact_params_.poisson_ratio = 0.3;
  contact_params_.contact_radius = 0.005;   // 5mm
  contact_params_.depth_threshold = 0.001;  // 1mm
  target_force_ = 2.0;  // N
  
  // Initialize trajectory smoothing parameters
  vel_max_ = 0.05;           // Maximum velocity (rad/s) - used for circular motion
  acceleration_time_ = 2.0;  // Time to reach max velocity (s)
  vel_current_ = 0.0;        // Start from zero velocity
  angle_ = 0.0;              // Start angle or distance
  
  // Initialize motion phase parameters
  move_to_target_complete_ = false;
  move_start_time_ = ros::Time(0);
  pause_start_time_ = ros::Time(0);
  move_progress_ = 0.0;
  circle_center_position_.setZero();
  circle_angle_ = 0.0;
  pause_pose_saved_ = false;  // 初始化暂停姿态标记
  
  // 初始化下探阶段变量
  probe_start_time_ = ros::Time(0);
  probe_start_position_.setZero();
  probe_start_force_z_ = 0.0;
  probe_contact_detected_ = false;
  probe_contact_time_ = ros::Time(0);
  probe_pause_start_time_ = ros::Time(0);
  probe_start_pose_.fill(0.0);
  
  // ✅ 初始化日志记录频率控制参数
  log_frequency_ = 100.0;               // 100 Hz 日志记录频率
  log_period_ = 1.0 / log_frequency_;   // 0.01 秒周期
  last_log_time_ = ros::Time(0);        // 初始化为0
  
  ROS_INFO("ContactController: Log recording frequency set to %.1f Hz (%.3f s period)", 
           log_frequency_, log_period_);
  
  ROS_INFO("ContactController initialized successfully - Move to target + Circular motion mode");
  return true;
}

void ContactController::starting(const ros::Time& time) {
  // Get initial state - 使用期望位姿而不是当前位姿，确保连续性
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

  // Extract initial position
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_pose_.data()));
  initial_position_ = initial_transform.translation();
  
  // 确保初始命令与当前状态匹配，避免start pose invalid错误
  cartesian_pose_handle_->setCommand(initial_pose_);
  
  // Reset trajectory smoothing variables
  vel_current_ = 0.0;
  angle_ = 0.0;
  elapsed_time_ = ros::Duration(0.0);  // 确保elapsed_time_从0开始
  
  // Reset motion phase variables
  move_to_target_complete_ = false;
  move_start_time_ = ros::Time(0);
  pause_start_time_ = ros::Time(0);
  move_progress_ = 0.0;
  circle_center_position_.setZero();
  circle_angle_ = 0.0;
  pause_pose_saved_ = false;  // 初始化暂停姿态标记
  
  // ✅ 初始化圆周运动渐进启动变量
  circle_motion_started_ = false;
  circle_startup_duration_ = 2.0;
  
  // ✅ 初始化分阶段圆周运动控制变量
  circle_force_control_enabled_ = false;
  circle_force_control_delay_ = 10.0;  // 10秒后启用力控制（从5.0秒延长到10.0秒）
  circle_force_control_start_time_ = ros::Time(0);
  
  // 初始化下探阶段变量
  probe_start_time_ = ros::Time(0);
  probe_start_position_.setZero();
  probe_start_force_z_ = 0.0;
  probe_contact_detected_ = false;
  probe_contact_time_ = ros::Time(0);
  probe_pause_start_time_ = ros::Time(0);
  probe_start_pose_.fill(0.0);
  
  ROS_INFO("ContactController: Motion variables initialized");
  
  // Set control state to zeroing phase
  control_phase_ = CALIBRATION;  // Start with force sensor zeroing
  phase_start_time_ = time;
  elapsed_time_ = ros::Duration(0.0);
  
  // Initialize zeroing variables
  force_zeroing_complete_ = false;
  zeroing_start_time_ = time;
  waiting_for_user_command_ = false;
  force_offset_.setZero();
  force_window_buffer_.clear();
  
  ROS_INFO("ContactController: Zeroing variables initialized");
  
  // ==== 重置滤波器状态 ====
  force_filter_initialized_ = false;
  contact_filter_initialized_ = false;
  filtered_force_.setZero();
  contact_filtered_force_.setZero();
  force_history_buffer_.clear();
  
  ROS_INFO("ContactController: Force filtering system reset and ready");
  // ==============================
  
  // ==== 重置力反馈控制状态 ====
  target_force_recorded_ = false;
  target_force_z_ = 0.0;
  force_error_prev_ = 0.0;
  current_z_offset_ = 0.0;
  
  // ✅ 重置力控制渐进启动状态
  force_control_gradual_startup_ = true;
  force_control_init_time_ = ros::Time(0);
  
  ROS_INFO("ContactController: Force feedback control reset and ready");
  // ===============================
  
  // ✅ 初始化日志记录时间
  last_log_time_ = time;
  
  publishPhaseStatus("ZEROING");
  
  ROS_INFO("ContactController started - Beginning force sensor zeroing process");
  ROS_INFO("Initial pose set: [%.3f, %.3f, %.3f]", initial_position_(0), initial_position_(1), initial_position_(2));
  ROS_INFO("Target position: [%.3f, %.3f, %.3f]", target_position_(0), target_position_(1), target_position_(2));
  ROS_INFO("Please ensure no external forces are applied to the robot end-effector");
  ROS_INFO("Zeroing will take approximately 10-15 seconds...");
}

void ContactController::update(const ros::Time& time, const ros::Duration& period) {
  elapsed_time_ += period;
  
  // Get robot state
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  
  // ✅ 修正：使用期望位姿进行逻辑判断，避免跟踪误差导致的不连续
  Eigen::Affine3d current_desired_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE_d.data()));
  Eigen::Vector3d current_desired_position = current_desired_transform.translation();
  
  // 只有在需要监控实际位置时才使用实际位姿（如安全检查）
  Eigen::Affine3d current_actual_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d current_actual_position = current_actual_transform.translation();
  
  // Get current force sensor data
  Eigen::Vector3d current_force_raw;
  Eigen::Vector3d current_force_filtered;
  {
    std::lock_guard<std::mutex> lock(force_mutex_);
    current_force_raw = current_force_;
    current_force_filtered = current_force_;
  }
  
  // State machine for different control phases
  double energy_scale = 1.0;  // 声明能量缩放因子，默认值为1.0
  
  if (control_phase_ == CALIBRATION && !force_zeroing_complete_) {
    // === WAITING FOR FORCE SENSOR ZEROING ===
    cartesian_pose_handle_->setCommand(initial_pose_);
    
    if ((time - zeroing_start_time_).toSec() > 8.0) {  // Wait 8 seconds for external zeroing
      force_zeroing_complete_ = true;
      ROS_INFO("Force sensor external zeroing completed - controller ready");
    }
    
  } else if (control_phase_ == CALIBRATION && force_zeroing_complete_ && !waiting_for_user_command_) {
    // === WAITING FOR USER COMMAND PHASE ===
    cartesian_pose_handle_->setCommand(initial_pose_);
    
    waiting_for_user_command_ = true;
    publishPhaseStatus("WAITING_FOR_COMMAND");
    
    ROS_INFO("=== FORCE SENSOR ZEROING COMPLETED ===");
    ROS_INFO("System is ready for motion to target position + probe descent + circular motion.");
    ROS_INFO("Target position: [%.3f, %.3f, %.3f]", target_position_(0), target_position_(1), target_position_(2));
    ROS_INFO("Control flow: Move to target (40s) -> Pause (2s) -> Probe descent (1mm/s until contact) -> Pause (2s) -> Circular motion (2.5cm radius)");
    ROS_INFO("Enter 'start' to begin the complete sequence:");
      
    startUserInputMonitoring();
    
  } else if (control_phase_ == CALIBRATION && waiting_for_user_command_) {
    // === WAITING FOR USER INPUT ===
    cartesian_pose_handle_->setCommand(initial_pose_);
    
    if (checkUserInput()) {
      control_phase_ = TRAJECTORY;
        phase_start_time_ = time;
      elapsed_time_ = ros::Duration(0.0);
      waiting_for_user_command_ = false;
      
      vel_current_ = 0.0;
      angle_ = 0.0;
      
      publishPhaseStatus("TRAJECTORY");
      ROS_INFO("Starting motion to target position + circular motion (2.5cm radius)");
    }
    
  } else if (control_phase_ == TRAJECTORY) {
    // === MOVE TO TARGET POSITION PHASE ===
    
    // 获取实际的关节力矩和速度数据 - 大幅减少频率以降低计算负载
    static Eigen::Matrix<double, 7, 1> dq_vec, tau_J_vec;
    static int energy_update_counter = 0;
    if (++energy_update_counter % 20 == 0) {
      for (int i = 0; i < 7; ++i) {
        dq_vec(i) = robot_state.dq[i];
        tau_J_vec(i) = robot_state.tau_J[i];
      }
      energy_scale = energy_monitor_->updateAndGetScaleFactor(tau_J_vec, robot_state, jacobian, period);
    }
    
    if (move_start_time_.isZero()) {
      move_start_time_ = time;
      publishPhaseStatus("MOVE_TO_TARGET");
      ROS_INFO("Starting move to target position [%.3f, %.3f, %.3f]", 
               target_position_(0), target_position_(1), target_position_(2));
    }
    
    // 简化距离计算
    Eigen::Vector3d distance_to_target = target_position_ - current_desired_position;
    double distance_magnitude = distance_to_target.norm();
    
    const double distance_threshold = 0.008;  // 8mm
    
    static int trajectory_log_counter = 0;
    if (++trajectory_log_counter % 1000 == 0) {
      ROS_INFO("Motion progress: %.1f%%, distance to target: %.1f mm", 
               move_progress_ * 100, distance_magnitude * 1000);
    }
    
    if (distance_magnitude <= distance_threshold) {
      // 到达目标位置，切换到停顿阶段
      control_phase_ = PAUSE_AT_TARGET;
      pause_start_time_ = time;
      
      publishPhaseStatus("PAUSE_AT_TARGET");
      ROS_INFO("Reached target position (distance: %.3f mm). Starting 2s pause at [%.3f, %.3f, %.3f]",
               distance_magnitude * 1000, current_desired_position(0), current_desired_position(1), current_desired_position(2));
      
    } else {
      // 继续向目标位置运动 - 使用更平滑的轨迹规划
      static bool trajectory_initialized = false;
      static Eigen::Vector3d target_offset;
      static double motion_duration;
      static double max_velocity;
      
      if (!trajectory_initialized) {
        target_offset = target_position_ - initial_position_;
        double total_distance = target_offset.norm();
        
        // 使用更保守的速度设置，确保符合Franka安全要求
        max_velocity = 0.005;  // 降低到5mm/s，确保平滑性
        motion_duration = std::max(40.0, total_distance / max_velocity);  // 至少40秒
        
        trajectory_initialized = true;
        ROS_INFO("Trajectory planning: distance=%.3f m, duration=%.1f s, max_vel=%.3f m/s", 
                 total_distance, motion_duration, max_velocity);
      }
      
      // 使用五次多项式轨迹确保速度和加速度连续性
      double t = elapsed_time_.toSec();
      double T = motion_duration;
      
      double smoothed_progress;
      if (t <= 0.0) {
        smoothed_progress = 0.0;
      } else if (t >= T) {
        smoothed_progress = 1.0;
      } else {
        // 五次多项式：q(t) = 10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5
        // 确保 q(0)=0, q'(0)=0, q''(0)=0, q(T)=1, q'(T)=0, q''(T)=0
        double tau = t / T;
        smoothed_progress = 10.0 * std::pow(tau, 3) - 15.0 * std::pow(tau, 4) + 6.0 * std::pow(tau, 5);
      }
    
      // 应用能量缩放因子
      smoothed_progress = std::min(smoothed_progress * energy_scale, 1.0);
    
      Eigen::Vector3d current_offset = smoothed_progress * target_offset;
      std::array<double, 16> target_pose = initial_pose_;
      target_pose[12] = initial_position_(0) + current_offset(0);
      target_pose[13] = initial_position_(1) + current_offset(1);
      target_pose[14] = initial_position_(2) + current_offset(2);
      
      cartesian_pose_handle_->setCommand(target_pose);
      move_progress_ = smoothed_progress;
    }
    
  } else if (control_phase_ == PAUSE_AT_TARGET) {
    // === PAUSE AT TARGET POSITION PHASE ===
    // ✅ 修正：不在暂停期间重新获取位姿，而是在进入暂停阶段时就确定基准位姿
    
    static int pause_energy_counter = 0;
    if (++pause_energy_counter % 50 == 0) {
      static Eigen::Matrix<double, 7, 1> dq_vec, tau_J_vec;
      for (int i = 0; i < 7; ++i) {
        dq_vec(i) = robot_state.dq[i];
        tau_J_vec(i) = robot_state.tau_J[i];
      }
      energy_scale = energy_monitor_->updateAndGetScaleFactor(tau_J_vec, robot_state, jacobian, period);
    }
    
    // ✅ 修正：只在第一次进入暂停阶段时保存基准姿态（基于当前期望位姿）
    if (!pause_pose_saved_) {
      circle_base_pose_ = robot_state.O_T_EE_d;  // 使用当前期望位姿作为圆周运动基准
      
      // 提取基准位置用于圆心计算
      Eigen::Affine3d base_transform(Eigen::Matrix4d::Map(circle_base_pose_.data()));
      Eigen::Vector3d base_position = base_transform.translation();
      
      // 计算圆周运动中心：圆心在基准位置X坐标+2.5cm处
      circle_center_position_ = base_position;
      circle_center_position_(0) += 0.025;  // X坐标向前偏移2.5cm作为圆心
      
      // 计算初始角度：从圆心到基准位置的角度（在世界坐标系xy平面内）
      Eigen::Vector3d center_to_base = base_position - circle_center_position_;
      circle_angle_ = std::atan2(center_to_base(1), center_to_base(0));
      
      pause_pose_saved_ = true;
      ROS_INFO("Pause pose saved. Circle center at [%.3f, %.3f, %.3f], initial angle: %.1f°",
               circle_center_position_(0), circle_center_position_(1), circle_center_position_(2),
               circle_angle_ * 180.0 / M_PI);
    }
    
    double pause_duration = 2.0;
    double pause_elapsed = (time - pause_start_time_).toSec();
    
    static int pause_log_counter = 0;
    if (++pause_log_counter % 500 == 0) {
      ROS_INFO("Pause progress: %.1f/%.1f s", pause_elapsed, pause_duration);
    }
    
    if (pause_elapsed >= pause_duration) {
      // 停顿结束，开始下探阶段
      control_phase_ = PROBE_DESCENT;
      probe_start_time_ = time;
      elapsed_time_ = ros::Duration(0.0);  // 重置时间用于下探阶段
      
      // ✅ 修正：保存下探开始的期望姿态和位置（保证连续性）
      probe_start_pose_ = robot_state.O_T_EE_d;  // 使用期望位姿
      Eigen::Affine3d probe_transform(Eigen::Matrix4d::Map(probe_start_pose_.data()));
      probe_start_position_ = probe_transform.translation();
      
      // 保存下探开始时的z轴力值作为基准
      {
        std::lock_guard<std::mutex> lock(force_mutex_);
        probe_start_force_z_ = current_force_filtered(2);  // z轴分量
      }
      
      probe_contact_detected_ = false;
      
      publishPhaseStatus("PROBE_DESCENT");
      ROS_INFO("Pause completed. Starting probe descent at [%.3f, %.3f, %.3f], baseline force_z: %.3f N",
               probe_start_position_(0), probe_start_position_(1), probe_start_position_(2), probe_start_force_z_);
    }
    
    // ✅ 修正：暂停期间不发送新命令，保持当前期望位姿
    
  } else if (control_phase_ == PROBE_DESCENT) {
    // === 下探接触检测阶段 ===
    
    static int probe_energy_counter = 0;
    if (++probe_energy_counter % 50 == 0) {
      static Eigen::Matrix<double, 7, 1> dq_vec, tau_J_vec;
      for (int i = 0; i < 7; ++i) {
        dq_vec(i) = robot_state.dq[i];
        tau_J_vec(i) = robot_state.tau_J[i];
      }
      energy_scale = energy_monitor_->updateAndGetScaleFactor(tau_J_vec, robot_state, jacobian, period);
    }
    
    // 获取当前z轴力值并检测接触
    double current_force_z;
    {
      std::lock_guard<std::mutex> lock(force_mutex_);
      current_force_z = current_force_filtered(2);
    }
    
    double force_change = std::abs(current_force_z - probe_start_force_z_);
    const double force_threshold = 0.3;  // 0.5N的力值变化阈值
    
    if (!probe_contact_detected_ && force_change > force_threshold) {
      // 检测到接触，开始减速停止
      probe_contact_detected_ = true;
      probe_contact_time_ = time;
      
      // ==== 记录目标力值用于力反馈控制 ====
      if (!target_force_recorded_ && force_feedback_enabled_) {
        recordTargetForce(current_force_z);
      }
      // =================================
      
      ROS_INFO("Contact detected! Force change: %.3f N (threshold: %.3f N). Starting deceleration.",
               force_change, force_threshold);
      publishPhaseStatus("PROBE_CONTACT_DETECTED");
    }
    
    if (!probe_contact_detected_) {
      // 未检测到接触，继续下探运动
      double descent_velocity = 0.001;  // 1mm/s下降速度
      double descent_distance = descent_velocity * elapsed_time_.toSec();
      
      // 应用能量缩放因子
      descent_distance *= energy_scale;
      
      // 计算新的目标位置（只改变z坐标）
      std::array<double, 16> probe_pose = probe_start_pose_;
      probe_pose[14] = probe_start_position_(2) - descent_distance;  // z坐标下降
      
      cartesian_pose_handle_->setCommand(probe_pose);
      
      static int probe_log_counter = 0;
      if (++probe_log_counter % 1000 == 0) {
        ROS_INFO("Probing: descent=%.1f mm, force_z=%.3f N (baseline=%.3f N, change=%.3f N)",
                 descent_distance * 1000, current_force_z, probe_start_force_z_, force_change);
      }
      
    } else {
      // 已检测到接触，执行减速停止逻辑
      double decel_time = (time - probe_contact_time_).toSec();
      const double max_decel_time = 1.0;  // 1秒内减速到零
      
      if (decel_time < max_decel_time) {
        // 平滑减速：使用余弦函数实现平滑停止
        double decel_factor = 0.5 * (1.0 + std::cos(M_PI * decel_time / max_decel_time));
        
        double descent_velocity = 0.001;  // 1mm/s基础速度
        double remaining_velocity = descent_velocity * decel_factor;
        double additional_descent = remaining_velocity * period.toSec();
    
        // 应用能量缩放因子
        additional_descent *= energy_scale;
        
        // 获取当前命令姿态并继续微调下降
        std::array<double, 16> current_command = robot_state.O_T_EE_d;
        current_command[14] -= additional_descent;  // 继续轻微下降
    
        cartesian_pose_handle_->setCommand(current_command);
    
        static int decel_log_counter = 0;
        if (++decel_log_counter % 200 == 0) {
          ROS_INFO("Decelerating: time=%.2f/%.2f s, velocity_factor=%.3f",
                   decel_time, max_decel_time, decel_factor);
        }
        
      } else {
        // 减速完成，进入下探后暂停阶段
        control_phase_ = PROBE_PAUSE;
        probe_pause_start_time_ = time;
        
        publishPhaseStatus("PROBE_PAUSE");
        ROS_INFO("Deceleration completed. Starting probe pause phase for 2 seconds.");
      }
    }
    
  } else if (control_phase_ == PROBE_PAUSE) {
    // === 下探后暂停阶段 ===
    // 参考franka stopping函数逻辑，不发送新的运动指令，让机器人自然静止
    
    static int probe_pause_energy_counter = 0;
    if (++probe_pause_energy_counter % 50 == 0) {
      static Eigen::Matrix<double, 7, 1> dq_vec, tau_J_vec;
      for (int i = 0; i < 7; ++i) {
        dq_vec(i) = robot_state.dq[i];
        tau_J_vec(i) = robot_state.tau_J[i];
      }
      energy_scale = energy_monitor_->updateAndGetScaleFactor(tau_J_vec, robot_state, jacobian, period);
    }
    
    double probe_pause_duration = 2.0;
    double probe_pause_elapsed = (time - probe_pause_start_time_).toSec();
    
    static int probe_pause_log_counter = 0;
    if (++probe_pause_log_counter % 500 == 0) {
      ROS_INFO("Probe pause progress: %.1f/%.1f s", probe_pause_elapsed, probe_pause_duration);
    }
    
    if (probe_pause_elapsed >= probe_pause_duration) {
      // ✅ 修正：下探后暂停结束，使用当前期望位姿作为圆周运动基准
      circle_base_pose_ = robot_state.O_T_EE_d;  // 使用期望位姿而非实际位姿
      
      // 提取基准位置用于圆心计算
      Eigen::Affine3d base_transform(Eigen::Matrix4d::Map(circle_base_pose_.data()));
      Eigen::Vector3d base_position = base_transform.translation();
      
      // ✅ 改进：确保圆周运动从当前位置开始，完全消除位置跳变
      // 将当前位置设为圆周上的起始点，圆心相对于当前位置偏移
      circle_center_position_ = base_position;
      circle_center_position_(0) += 0.025;  // X坐标向前偏移2.5cm作为圆心
      
      // ✅ 改进：精确计算初始角度，确保第一个目标位置就是当前位置
      Eigen::Vector3d center_to_current = base_position - circle_center_position_;
      circle_angle_ = std::atan2(center_to_current(1), center_to_current(0));
      
      // ✅ 改进：验证第一个目标位置与当前位置的一致性
      double radius = 0.025;  // 2.5cm半径
      double verify_x = circle_center_position_(0) + radius * std::cos(circle_angle_);
      double verify_y = circle_center_position_(1) + radius * std::sin(circle_angle_);
      double position_error = std::sqrt(std::pow(verify_x - base_position(0), 2) + 
                                       std::pow(verify_y - base_position(1), 2));
      
      if (position_error > 0.001) {  // 1mm误差阈值
        ROS_WARN("Circular motion setup error: position mismatch %.3f mm, adjusting...", position_error * 1000);
        // 如果有误差，直接设置圆心使得当前位置在圆周上
        circle_center_position_(0) = base_position(0) - radius * std::cos(circle_angle_);
        circle_center_position_(1) = base_position(1) - radius * std::sin(circle_angle_);
      }
      
      // 进入圆周运动阶段
      control_phase_ = CIRCULAR_MOTION;
      elapsed_time_ = ros::Duration(0.0);  // 重置时间用于圆周运动
      
      // ✅ 新增：圆周运动启动标志，用于实现渐进启动
      circle_motion_started_ = false;
      circle_startup_duration_ = 2.0;  // 2秒渐进启动时间
      
      // ✅ 新增：设置分阶段力控制
      circle_force_control_enabled_ = false;  // 初始关闭力控制
      circle_force_control_start_time_ = time + ros::Duration(circle_force_control_delay_);  // 5秒后启用
      
      // ✅ 新增：初始化日志系统，从圆周运动开始记录数据
      ROS_INFO("ContactController: Initializing log system for circular motion data recording...");
      if (log_generator_) {
        log_generator_->initLogFile(contact_params_, target_force_);
        ROS_INFO("ContactController: Log system initialized - recording will start with circular motion");
      } else {
        ROS_WARN("ContactController: log_generator_ is null, cannot initialize logging");
      }
      
      publishPhaseStatus("CIRCULAR_MOTION");
      ROS_INFO("Probe pause completed. Starting circular motion (radius=2.5cm) with gradual startup");
      ROS_INFO("Circle center at [%.3f, %.3f, %.3f], initial angle: %.1f°",
               circle_center_position_(0), circle_center_position_(1), circle_center_position_(2),
               circle_angle_ * 180.0 / M_PI);
      ROS_INFO("Position verification: error=%.3f mm (acceptable: <1.0mm)", position_error * 1000);
      ROS_INFO("Force control will be enabled after %.1f seconds for z-axis adjustment", circle_force_control_delay_);
    }
    
    // ✅ 修正：下探后暂停期间不发送新命令，保持当前期望位姿
    
  } else if (control_phase_ == CIRCULAR_MOTION) {
    // === CIRCULAR MOTION PHASE ===
    // ✅ 改进：使用渐进启动确保速度和加速度连续性
    
    static int circle_energy_counter = 0;
    if (++circle_energy_counter % 30 == 0) {
      static Eigen::Matrix<double, 7, 1> dq_vec, tau_J_vec;
      for (int i = 0; i < 7; ++i) {
        dq_vec(i) = robot_state.dq[i];
        tau_J_vec(i) = robot_state.tau_J[i];
      }
      energy_scale = energy_monitor_->updateAndGetScaleFactor(tau_J_vec, robot_state, jacobian, period);
    }
    
    double current_time = elapsed_time_.toSec();
    
    // ✅ 第一阶段：渐进启动 (0-2秒)
    if (current_time < circle_startup_duration_) {
      // 渐进启动阶段：保持在起始位置，准备开始运动
      if (!circle_motion_started_) {
        // 第一次进入，设置初始命令为当前期望位姿，确保无跳变
        cartesian_pose_handle_->setCommand(circle_base_pose_);
        circle_motion_started_ = true;
        
        static int startup_log_counter = 0;
        if (++startup_log_counter % 500 == 0) {
          ROS_INFO("Circular motion startup: %.1f/%.1f s, holding at start position", 
                   current_time, circle_startup_duration_);
        }
      } else {
        // 启动期间：保持在圆周起始位置
        double radius = 0.025;  // 2.5cm半径
        std::array<double, 16> startup_pose = circle_base_pose_;
        
        // 确保位置精确在圆周上
        startup_pose[12] = circle_center_position_(0) + radius * std::cos(circle_angle_);
        startup_pose[13] = circle_center_position_(1) + radius * std::sin(circle_angle_);
        startup_pose[14] = circle_center_position_(2);  // z坐标保持不变
        
        cartesian_pose_handle_->setCommand(startup_pose);
        
        static int startup_hold_counter = 0;
        if (++startup_hold_counter % 1000 == 0) {
          ROS_INFO("Circular startup: holding position [%.3f, %.3f, %.3f] for %.1f more seconds",
                   startup_pose[12], startup_pose[13], startup_pose[14],
                   circle_startup_duration_ - current_time);
        }
      }
      
    } else {
      // ✅ 第二阶段：实际圆周运动 (2秒后开始)
      // 调整时间基准：从启动完成时刻开始计算圆周运动时间
      double motion_time = current_time - circle_startup_duration_;
      
      // 完整圆周运动：20秒完成一个完整的2π弧度圆周
      double time_period = 20.0;  // 20秒一个完整圆周
      
      // 计算当前应该走过的角度（0到2π），支持连续循环
      double target_angle_progress;
      
      // 使用模运算实现连续循环，每20秒重复一次
      double cycle_time = fmod(motion_time, time_period);
      
      // ✅ 改进：使用更平滑的启动曲线，确保在motion_time=0时速度为0
      double t_norm = cycle_time / time_period;  // 归一化时间 [0,1]
      
      // 平滑的角度进展函数：确保在t=0和t=1时角速度为0
      // 使用 3t² - 2t³ 函数（S曲线）
      double smooth_progress = 3.0 * t_norm * t_norm - 2.0 * t_norm * t_norm * t_norm;
      target_angle_progress = smooth_progress * 2.0 * M_PI;  // 完整圆周2π弧度
      
      // 应用能量缩放因子
      target_angle_progress *= energy_scale;
      
      // 计算当前角度位置（相对于初始角度）
      double current_angle = circle_angle_ + target_angle_progress;
      
      // 计算圆周位置（在世界坐标系xy平面内）
      double radius = 0.025;  // 2.5cm半径
      double delta_x = radius * std::cos(current_angle);
      double delta_y = radius * std::sin(current_angle);
      
      // 使用保存的基准姿态进行圆周运动
      std::array<double, 16> circular_pose = circle_base_pose_;
      circular_pose[12] = circle_center_position_(0) + delta_x;
      circular_pose[13] = circle_center_position_(1) + delta_y;
      circular_pose[14] = circle_center_position_(2);  // z坐标先保持不变
      
      // ✅ 改进：分阶段z轴力控制
      // 检查是否到了启用力控制的时间
      if (!circle_force_control_enabled_ && time >= circle_force_control_start_time_) {
        circle_force_control_enabled_ = true;
        
        // ✅ 重置力控制渐进启动状态
        force_control_gradual_startup_ = true;  // 启用渐进启动
        current_z_offset_ = 0.0;  // 重置偏移量
        force_error_prev_ = 0.0;  // 重置PD控制器状态
        
        ROS_INFO("=== FORCE CONTROL ACTIVATED ===");
        ROS_INFO("Starting z-axis force feedback control for circular motion");
        ROS_INFO("Target force: %.3f N, current phase: xy+z coordinated control", target_force_z_);
        ROS_INFO("Gradual startup initiated - conservative force control with %.1fs ramp-up", force_control_startup_duration_);
      }
      
      // ==== 应用z轴力反馈控制（仅在启用后） ====
      if (circle_force_control_enabled_ && force_feedback_enabled_ && target_force_recorded_) {
        // 获取当前z轴力值进行力反馈控制
        double current_force_z;
        {
          std::lock_guard<std::mutex> lock(force_mutex_);
          current_force_z = current_force_filtered(2);
        }
        
        // 应用力反馈控制调节z轴位置
        double controlled_z = applyForceControl(circle_center_position_(2), current_force_z, period.toSec());
        circular_pose[14] = controlled_z;
        
        // 记录力控制日志
        static int force_control_log_counter = 0;
        if (++force_control_log_counter % 1000 == 0) {
          double force_error = target_force_z_ - current_force_z;
          bool in_deadzone = std::abs(force_error) < force_deadzone_;
          ROS_INFO("Force control: Target_F=%.3f N, Current_F=%.3f N, Error=%.3f N, Deadzone=%s",
                   target_force_z_, current_force_z, force_error, in_deadzone ? "YES" : "NO");
          ROS_INFO("             Z_offset=%.3f mm, Z_pos=%.3f, MaxChange=%.2f mm/step",
                   current_z_offset_ * 1000, controlled_z, max_change_per_step_ * 1000);
        }
      } else if (!circle_force_control_enabled_) {
        // 力控制未启用阶段：z轴保持固定位置
        static int xy_only_log_counter = 0;
        if (++xy_only_log_counter % 2000 == 0) {
          double time_to_force_control = (circle_force_control_start_time_ - time).toSec();
          ROS_INFO("XY-only circular motion: %.1f seconds until force control activation", 
                   std::max(0.0, time_to_force_control));
        }
      }
      // ==============================
      
      cartesian_pose_handle_->setCommand(circular_pose);
      
      static int circle_log_counter = 0;
      if (++circle_log_counter % 2000 == 0) {
        double angle_degrees = (target_angle_progress * 180.0 / M_PI);
        int cycle_count = static_cast<int>(motion_time / time_period) + 1;
        double cycle_progress = (cycle_time / time_period) * 100.0;
        ROS_INFO("Circular motion: Cycle %d, %.1f%% in current cycle, angle=%.1f°, position=[%.3f, %.3f, %.3f]", 
                 cycle_count, cycle_progress, angle_degrees,
                 circular_pose[12], circular_pose[13], circular_pose[14]);
      }
    }
    
  } else {
    // Default: maintain current pose
    cartesian_pose_handle_->setCommand(initial_pose_);
  
    static int default_energy_counter = 0;
    if (++default_energy_counter % 20 == 0) {
      static Eigen::Matrix<double, 7, 1> dq_vec, tau_J_vec;
      for (int i = 0; i < 7; ++i) {
        dq_vec(i) = robot_state.dq[i];
        tau_J_vec(i) = robot_state.tau_J[i];
      }
      energy_scale = energy_monitor_->updateAndGetScaleFactor(tau_J_vec, robot_state, jacobian, period);
    }
  }
  
  // 获取当前能量级别
  double current_energy = energy_monitor_->getEnergyLevel();
  
  // Publish current energy
  std_msgs::Float64 energy_msg;
  energy_msg.data = current_energy;
  energy_pub_.publish(energy_msg);
  
  // ✅ 记录日志数据 - 只在圆周运动阶段记录数据，并控制100Hz频率
  if (log_generator_ && control_phase_ == CIRCULAR_MOTION) {
    // 检查是否达到日志记录间隔（100Hz = 0.01秒间隔）
    double time_since_last_log = (time - last_log_time_).toSec();
    
    if (time_since_last_log >= log_period_) {
      // 获取当前位置（期望位姿的位置部分）
      Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE_d.data()));
      Eigen::Vector3d current_position = current_transform.translation();
      
      // 获取当前力数据
      Eigen::Vector3d current_force_log, raw_force_log;
      {
        std::lock_guard<std::mutex> lock(force_mutex_);
        current_force_log = current_force_;
        raw_force_log = raw_force_;
      }
      
      // 计算理论力（在我们的场景中，主要是在下探阶段产生的接触力）
      double theoretical_force = 0.0;
      if (probe_contact_detected_) {
        // 使用记录的目标力作为理论参考
        theoretical_force = std::abs(target_force_z_);
      }
      
      // 计算接触参考点（下探阶段的初始z位置）
      double contact_reference_z = (probe_contact_detected_) ? probe_start_position_(2) : 0.0;
      
      // ✅ 记录数据 - 只在满足频率条件时记录
      log_generator_->logData(
        time,                           // 时间
        current_position,               // 当前位置
        current_force_log,              // 滤波后的力
        static_cast<int>(control_phase_), // 控制阶段
        contact_reference_z,            // 软体块表面z坐标（接触参考点）
        theoretical_force,              // 理论力
        probe_length_,                  // 探头长度
        contact_reference_z,            // 接触参考z坐标
        raw_force_log(2),              // 原始z轴力值
        current_energy,                 // 能量水平
        energy_scale                    // 能量缩放因子
      );
      
      // 更新上次记录时间
      last_log_time_ = time;
    }
  }
  
  // Publish desired pose (for visualization)
  publishVisualizationPose(time, robot_state);
}

void ContactController::stopping(const ros::Time& time) {
  // 确保机器人平滑停止到零速度
  // 设置当前位置为目标位置，这样机器人会自然停止运动
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 16> current_pose = robot_state.O_T_EE_d;  // 使用期望位姿确保连续性
  cartesian_pose_handle_->setCommand(current_pose);
  
  // ✅ 关闭日志文件
  if (log_generator_) {
    log_generator_->closeLogFile(time);
  }
  
  publishPhaseStatus("STOPPED");
  ROS_INFO("ContactController stopped");
}

void ContactController::forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(force_mutex_);
  
  // 这里接收的是已经调零后的数据（从/force_sensor/wrench话题）
  // 但仍需要在控制器层面进行额外的滤波处理以应对高频噪声和毛刺
  Eigen::Vector3d raw_force_input;
  raw_force_input(0) = msg->wrench.force.x;
  raw_force_input(1) = msg->wrench.force.y;
  raw_force_input(2) = msg->wrench.force.z;
  
  // 应用增强的滤波处理
  // 根据当前控制阶段选择合适的滤波策略
  bool use_contact_filter = (control_phase_ == PROBE_DESCENT || control_phase_ == CONTACT);
  Eigen::Vector3d filtered_force_result = applyForceFiltering(raw_force_input, use_contact_filter);
  
  // 保存滤波后的结果
  current_force_ = filtered_force_result;
  
  // 每100次回调打印一次滤波效果统计（用于调试）
  static int filter_debug_counter = 0;
  if (++filter_debug_counter % 100 == 0) {
    Eigen::Vector3d difference = raw_force_input - filtered_force_result;
    double noise_reduction = difference.norm();
    if (noise_reduction > 0.05) {  // 只在噪声明显时记录
      ROS_DEBUG("Force filtering: Raw[%.3f,%.3f,%.3f] -> Filtered[%.3f,%.3f,%.3f], Noise reduced: %.3f N",
               raw_force_input(0), raw_force_input(1), raw_force_input(2),
               filtered_force_result(0), filtered_force_result(1), filtered_force_result(2),
               noise_reduction);
    }
  }
  
  force_data_available_ = true;
}

// 添加新的原始数据回调函数
void ContactController::rawForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(force_mutex_);
  
  // 保存真正的原始数据用于日志记录
  raw_force_(0) = msg->wrench.force.x;
  raw_force_(1) = msg->wrench.force.y;
  raw_force_(2) = msg->wrench.force.z;
}

void ContactController::publishPhaseStatus(const std::string& phase_name) {
  std_msgs::String phase_msg;
  phase_msg.data = phase_name;
  phase_pub_.publish(phase_msg);
}

void ContactController::processForceZeroing(const ros::Time& time, const Eigen::Vector3d& raw_force) {
  // Add current force sample to window buffer
  ForceDataSample sample;
  sample.timestamp = time;
  sample.force = raw_force;
  force_window_buffer_.push_back(sample);
    
  // Maintain window size (keep last 5 seconds of data at 100Hz = 500 samples)
  const size_t max_window_size = 500;
  if (force_window_buffer_.size() > max_window_size) {
    force_window_buffer_.erase(force_window_buffer_.begin());
  }
  
  // Check if we have enough samples for zeroing analysis (at least 2 seconds of data)
  const size_t min_samples_for_analysis = 200;
  if (force_window_buffer_.size() < min_samples_for_analysis) {
    return;
  }
  
  // Calculate moving average for each force component
  Eigen::Vector3d force_sum = Eigen::Vector3d::Zero();
  for (const auto& sample : force_window_buffer_) {
    force_sum += sample.force;
  }
  Eigen::Vector3d force_mean = force_sum / force_window_buffer_.size();
  
  // Calculate standard deviation for stability check
  Eigen::Vector3d force_variance = Eigen::Vector3d::Zero();
  for (const auto& sample : force_window_buffer_) {
    Eigen::Vector3d diff = sample.force - force_mean;
    force_variance += diff.cwiseProduct(diff);
  }
  force_variance /= force_window_buffer_.size();
  Eigen::Vector3d force_std = force_variance.cwiseSqrt();
  
  // Check stability condition: 5 seconds of stable data with mean < 0.3N
  double stability_duration = 5.0;  // seconds
  ros::Duration required_duration(stability_duration);
  
  // Check if current window contains at least 5 seconds of data
  if (force_window_buffer_.size() >= static_cast<size_t>(stability_duration * 100)) {  // 100Hz
    // Check if all force components have mean < 0.3N and std < 0.1N
    bool is_stable = true;
    const double max_mean_force = 0.3;  // N
    const double max_std_force = 0.1;   // N
    
    for (int i = 0; i < 3; ++i) {
      if (std::abs(force_mean(i)) > max_mean_force || force_std(i) > max_std_force) {
        is_stable = false;
        break;
      }
    }
    
    if (is_stable) {
      // Zeroing completed successfully
      force_offset_ = force_mean;
      force_zeroing_complete_ = true;
      
      ROS_INFO("Force sensor zeroing completed successfully!");
      ROS_INFO("Final force offsets: [%.3f, %.3f, %.3f] N", 
               force_offset_(0), force_offset_(1), force_offset_(2));
      ROS_INFO("Standard deviations: [%.3f, %.3f, %.3f] N", 
               force_std(0), force_std(1), force_std(2));
    }
  }
}

void ContactController::startUserInputMonitoring() {
  if (!user_input_thread_started_) {
    user_input_ready_ = false;
    user_input_thread_started_ = true;
    
    // Start user input monitoring thread
    user_input_thread_ = std::thread([this]() {
      std::string input;
      while (waiting_for_user_command_ && ros::ok()) {
        std::cout << "Enter 'start' to begin complete sequence (move + probe descent + circular motion): ";
        std::getline(std::cin, input);
  
        // Convert to lowercase for case-insensitive comparison
        std::transform(input.begin(), input.end(), input.begin(), ::tolower);
        
        if (input == "start" || input == "s") {
          std::lock_guard<std::mutex> lock(user_input_mutex_);
          user_input_ready_ = true;
          ROS_INFO("User command received: starting complete sequence (move + probe descent + circular motion)...");
          break;
        } else if (input == "quit" || input == "exit" || input == "q") {
          ROS_INFO("User requested to quit the controller");
          ros::shutdown();
          break;
        } else {
          std::cout << "Invalid command. Please type 'start' to begin or 'quit' to exit." << std::endl;
        }
      }
    });
    
    // Detach thread to run independently
    user_input_thread_.detach();
  }
}

bool ContactController::checkUserInput() {
  std::lock_guard<std::mutex> lock(user_input_mutex_);
  return user_input_ready_;
}

void ContactController::publishVisualizationPose(const ros::Time& time, const franka::RobotState& robot_state) {
  // Publish desired pose (for visualization)
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = time;
  pose_msg.header.frame_id = "world";
  
  std::array<double, 16> current_command_pose;
  if (control_phase_ == TRAJECTORY) {
    // 运动到目标位置阶段的可视化
    current_command_pose = initial_pose_;
    
    Eigen::Vector3d target_offset = target_position_ - initial_position_;
    Eigen::Vector3d current_offset = move_progress_ * target_offset;
    
    current_command_pose[12] = initial_position_(0) + current_offset(0);
    current_command_pose[13] = initial_position_(1) + current_offset(1);
    current_command_pose[14] = initial_position_(2) + current_offset(2);
  } else if (control_phase_ == PAUSE_AT_TARGET) {
    // ✅ 修正：停顿阶段的可视化 - 使用期望位姿
    current_command_pose = initial_pose_;
    
    // 使用当前期望位置（停顿位置）
    Eigen::Affine3d current_desired_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE_d.data()));
    Eigen::Vector3d pause_position = current_desired_transform.translation();
    
    current_command_pose[12] = pause_position(0);
    current_command_pose[13] = pause_position(1);
    current_command_pose[14] = pause_position(2);
  } else if (control_phase_ == PROBE_DESCENT) {
    // 下探阶段的可视化
    current_command_pose = probe_start_pose_;
    
    if (!probe_contact_detected_) {
      // 下探运动中的可视化
      double descent_velocity = 0.001;  // 1mm/s下降速度
      double descent_distance = descent_velocity * elapsed_time_.toSec();
      
      current_command_pose[12] = probe_start_position_(0);
      current_command_pose[13] = probe_start_position_(1);
      current_command_pose[14] = probe_start_position_(2) - descent_distance;
    } else {
      // ✅ 修正：接触检测后减速阶段的可视化 - 使用期望位姿
      Eigen::Affine3d current_desired_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE_d.data()));
      Eigen::Vector3d current_position = current_desired_transform.translation();
      
      current_command_pose[12] = current_position(0);
      current_command_pose[13] = current_position(1);
      current_command_pose[14] = current_position(2);
    }
  } else if (control_phase_ == PROBE_PAUSE) {
    // ✅ 修正：下探后暂停阶段的可视化 - 使用期望位姿
    current_command_pose = initial_pose_;
    
    // 使用当前期望位置（下探后暂停位置）
    Eigen::Affine3d current_desired_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE_d.data()));
    Eigen::Vector3d probe_pause_position = current_desired_transform.translation();
    
    current_command_pose[12] = probe_pause_position(0);
    current_command_pose[13] = probe_pause_position(1);
    current_command_pose[14] = probe_pause_position(2);
  } else if (control_phase_ == CIRCULAR_MOTION) {
    // 圆周运动阶段的可视化 - 与主控制逻辑保持一致
    current_command_pose = initial_pose_;
    
    // 使用与主控制逻辑相同的完整圆周运动计算
    double time_period = 20.0;  // 20秒一个完整圆周
    double current_time = elapsed_time_.toSec();
    
    // 计算当前应该走过的角度（0到2π），支持连续循环
    double target_angle_progress;
    
    // 使用模运算实现连续循环，每20秒重复一次
    double cycle_time = fmod(current_time, time_period);
    
    // 使用平滑的角速度变化：开始和结束时速度为0，中间匀速
    // 使用三次多项式确保速度连续性
    double t_norm = cycle_time / time_period;  // 归一化时间 [0,1]
    
    // 平滑的角度进展函数：确保在t=0和t=1时角速度为0
    // 使用 3t² - 2t³ 函数（S曲线）
    double smooth_progress = 3.0 * t_norm * t_norm - 2.0 * t_norm * t_norm * t_norm;
    target_angle_progress = smooth_progress * 2.0 * M_PI;  // 完整圆周2π弧度
    
    double radius = 0.025;  // 2.5cm半径
    
    // 在世界坐标系xy平面内计算圆周位置
    double delta_x = radius * std::cos(circle_angle_ + target_angle_progress);
    double delta_y = radius * std::sin(circle_angle_ + target_angle_progress);
    
    current_command_pose[12] = circle_center_position_(0) + delta_x;
    current_command_pose[13] = circle_center_position_(1) + delta_y;
    current_command_pose[14] = circle_center_position_(2);  // z坐标保持不变
  } else {
    // 其他阶段使用初始姿态
    current_command_pose = initial_pose_;
  }
  
  Eigen::Affine3d target_transform(Eigen::Matrix4d::Map(current_command_pose.data()));
  Eigen::Vector3d target_position = target_transform.translation();
  Eigen::Quaterniond target_orientation(target_transform.rotation());
  
  pose_msg.pose.position.x = target_position(0);
  pose_msg.pose.position.y = target_position(1);
  pose_msg.pose.position.z = target_position(2);
  pose_msg.pose.orientation.x = target_orientation.x();
  pose_msg.pose.orientation.y = target_orientation.y();
  pose_msg.pose.orientation.z = target_orientation.z();
  pose_msg.pose.orientation.w = target_orientation.w();
  pose_pub_.publish(pose_msg);
}

// ==== 增强的力传感器滤波函数实现 ====

Eigen::Vector3d ContactController::applyForceFiltering(const Eigen::Vector3d& raw_force, bool use_contact_detection_filter) {
  // 简化的滤波策略：只使用EMA滤波，根据控制阶段选择参数
  // 移除滑动平均和异常值检测以减少延迟
  
  if (use_contact_detection_filter) {
    // 接触检测阶段：使用快速响应的滤波参数
    return applyEMAFilter(raw_force, contact_detection_alpha_, 
                         contact_filtered_force_, contact_filter_initialized_);
  } else {
    // 普通阶段：使用标准滤波参数
    return applyEMAFilter(raw_force, force_ema_alpha_, 
                         filtered_force_, force_filter_initialized_);
  }
}

Eigen::Vector3d ContactController::applyEMAFilter(const Eigen::Vector3d& raw_force, double alpha, 
                                                 Eigen::Vector3d& filtered_output, bool& is_initialized) {
  if (!is_initialized) {
    filtered_output = raw_force;
    is_initialized = true;
    return filtered_output;
  }
  
  // 指数滑动平均: y[n] = α * x[n] + (1-α) * y[n-1]
  filtered_output = alpha * raw_force + (1.0 - alpha) * filtered_output;
  return filtered_output;
}

// ==========================================

// ==== z轴力反馈控制函数实现 ====

void ContactController::recordTargetForce(double current_force_z) {
  target_force_z_ = current_force_z;
  target_force_recorded_ = true;
  force_error_prev_ = 0.0;  // 重置PD控制器状态
  
  ROS_INFO("Target force recorded: %.3f N for force feedback control", target_force_z_);
}

double ContactController::calculateForceControlOutput(double current_force_z, double dt) {
  if (!target_force_recorded_ || dt <= 0.0) {
    return 0.0;
  }
  
  // 计算力误差（期望力值 - 当前力值）
  // 注意：z轴力通常为负值（向下压力），我们关心的是力的大小
  double force_error = target_force_z_ - current_force_z;
  
  // 应用死区控制：小误差不进行调节
  if (std::abs(force_error) < force_deadzone_) {
    force_error = 0.0;
    force_error_prev_ = 0.0;  // 重置微分项
    return current_z_offset_;  // 返回当前偏移量，不进行调节
  }
  
  // PD控制器计算
  double proportional_term = force_kp_ * force_error;
  double derivative_term = force_kd_ * (force_error - force_error_prev_) / dt;
  
  // 控制输出（z轴位移调节量）
  double control_output = proportional_term + derivative_term;
  
  // 限制控制输出幅度
  control_output = std::max(z_offset_min_, std::min(z_offset_max_, control_output));
  
  // 更新上一次误差
  force_error_prev_ = force_error;
  
  return control_output;
}

double ContactController::applyForceControl(double base_z_position, double current_force_z, double dt) {
  if (!force_feedback_enabled_ || !target_force_recorded_) {
    return base_z_position;
  }
  
  // ✅ 检查是否刚启用力控制，如果是则启动渐进启动策略
  if (force_control_gradual_startup_) {
    force_control_init_time_ = ros::Time::now();
    force_control_gradual_startup_ = false;
    ROS_INFO("Force control gradual startup initiated - %.1f second ramp-up period", force_control_startup_duration_);
  }
  
  // 计算启动进度（0.0到1.0）
  double startup_elapsed = (ros::Time::now() - force_control_init_time_).toSec();
  double startup_factor = std::min(1.0, startup_elapsed / force_control_startup_duration_);
  
  // 计算PD控制器输出
  double control_output = calculateForceControlOutput(current_force_z, dt);
  
  // ✅ 在启动阶段应用渐进因子
  control_output *= startup_factor;
  
  // 计算期望的偏移量变化
  double desired_change = control_output - current_z_offset_;
  
  // ✅ 在启动阶段进一步限制变化量
  double max_change_limit = max_change_per_step_;
  if (startup_factor < 1.0) {
    max_change_limit *= (0.1 + 0.9 * startup_factor);  // 启动时最大变化量只有10%
  }
  
  // 限制单次变化量，避免突变
  if (std::abs(desired_change) > max_change_limit) {
    desired_change = (desired_change > 0) ? max_change_limit : -max_change_limit;
  }
  
  // 应用限制后的变化量
  double new_offset = current_z_offset_ + desired_change;
  
  // ✅ 在启动阶段使用更强的平滑滤波
  double current_smooth_alpha = smooth_alpha_;
  if (startup_factor < 1.0) {
    current_smooth_alpha *= (0.2 + 0.8 * startup_factor);  // 启动时平滑滤波更强
  }
  
  // 使用调整后的平滑滤波进行过渡
  current_z_offset_ = current_smooth_alpha * new_offset + (1.0 - current_smooth_alpha) * current_z_offset_;
  
  // 确保最终偏移量在允许范围内
  current_z_offset_ = std::max(z_offset_min_, std::min(z_offset_max_, current_z_offset_));
  
  // 应用z轴偏移
  double controlled_z_position = base_z_position + current_z_offset_;
  
  // ✅ 记录启动阶段的详细信息
  static int startup_log_counter = 0;
  if (startup_factor < 1.0 && ++startup_log_counter % 500 == 0) {
    ROS_INFO("Force control startup: progress=%.1f%%, factor=%.3f, max_change=%.3fmm, smooth_alpha=%.4f", 
             startup_factor * 100, startup_factor, max_change_limit * 1000, current_smooth_alpha);
  }
  
  return controlled_z_position;
}

// =============================

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ContactController,
                       controller_interface::ControllerBase)
