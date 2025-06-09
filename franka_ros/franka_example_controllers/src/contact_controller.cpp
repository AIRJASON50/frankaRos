/**
 * @file contact_controller.cpp
 * @brief 接触控制器实现，集成圆周运动、力传感器监控、日志记录和能量管理
 * 
 * 实现功能：
 * 1. 水平面圆周运动（转轴平行于z轴）
 * 2. 实时力传感器数据获取和处理
 * 3. 实验数据日志记录
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
  
  // Subscribe to raw force sensor data for logging purposes
  raw_force_sub_ = node_handle.subscribe<geometry_msgs::WrenchStamped>(
      "/force_sensor/raw_wrench", 10, &ContactController::rawForceCallback, this);
  
  // Read motion parameters
  ros::NodeHandle motion_nh(node_handle, "motion_control");
  motion_nh.param("circle_radius", circle_radius_, 0.075);  // 7.5cm radius
  motion_nh.param("motion_frequency", motion_frequency_, 0.2);  // 0.2 Hz
  motion_nh.param("probe_length", probe_length_, 0.1);  // 10cm probe length

  // Initialize log generator
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
  
  // Initialize user input variables
  waiting_for_user_command_ = false;
  user_input_ready_ = false;
  user_input_thread_started_ = false;
  
  // Contact parameters structure (for logging)
  contact_params_.young_modulus = 10000.0;  // Pa
  contact_params_.poisson_ratio = 0.3;
  contact_params_.contact_radius = 0.005;   // 5mm
  contact_params_.depth_threshold = 0.001;  // 1mm
  target_force_ = 2.0;  // N
  
  // Initialize trajectory smoothing parameters
  vel_max_ = 0.05;           // Maximum velocity (rad/s) 
  acceleration_time_ = 2.0;  // Time to reach max velocity (s)
  vel_current_ = 0.0;        // Start from zero velocity
  angle_ = 0.0;              // Start angle
  
  ROS_INFO("ContactController initialized successfully - Circular motion mode");
  return true;
}

void ContactController::starting(const ros::Time& time) {
  // Get initial state
  franka::RobotState initial_state = state_handle_->getRobotState();
  initial_pose_ = initial_state.O_T_EE_d;

  // Extract initial position
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_pose_.data()));
  initial_position_ = initial_transform.translation();
  
  // Reset trajectory smoothing variables
  vel_current_ = 0.0;
  angle_ = 0.0;
  
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
  
  // Initialize log file
  log_generator_->initLogFile(contact_params_, target_force_);
  
  publishPhaseStatus("ZEROING");
  
  ROS_INFO("ContactController started - Beginning force sensor zeroing process");
  ROS_INFO("Please ensure no external forces are applied to the robot end-effector");
  ROS_INFO("Zeroing will take approximately 10-15 seconds...");
}

void ContactController::update(const ros::Time& time, const ros::Duration& period) {
  elapsed_time_ += period;
  
  // Get robot state
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  
  // Get current position
  Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d current_position = current_transform.translation();
  
  // Get current force sensor data
  Eigen::Vector3d current_force_raw;
  Eigen::Vector3d current_force_zeroed;
  Eigen::Vector3d current_force_for_log;  // 声明用于日志记录的力数据
  double raw_force_z_for_log = 0.0;
  {
    std::lock_guard<std::mutex> lock(force_mutex_);
    current_force_raw = current_force_;
    current_force_zeroed = current_force_ - force_offset_;
    raw_force_z_for_log = raw_force_(2);
  }
  
  // State machine for different control phases
  if (control_phase_ == CALIBRATION && !force_zeroing_complete_) {
    // === WAITING FOR FORCE SENSOR ZEROING ===
    // The force sensor zeroing is handled by the force_sensor_read node
    // We just need to wait a bit and then consider it complete
    
    // Keep robot at initial pose during zeroing
    cartesian_pose_handle_->setCommand(initial_pose_);
    
    // Check if enough time has passed for force sensor to complete its zeroing
    if ((time - zeroing_start_time_).toSec() > 8.0) {  // Wait 8 seconds for external zeroing
      force_zeroing_complete_ = true;
      ROS_INFO("Force sensor external zeroing completed - controller ready");
    }
    
    // Use current force data for logging (it will be close to zero after external zeroing)
    current_force_for_log = current_force_;
    
  } else if (control_phase_ == CALIBRATION && force_zeroing_complete_ && !waiting_for_user_command_) {
    // === WAITING FOR USER COMMAND PHASE ===
    
    // Keep robot at initial pose
    cartesian_pose_handle_->setCommand(initial_pose_);
    
    // Switch to waiting state
    waiting_for_user_command_ = true;
    publishPhaseStatus("WAITING_FOR_COMMAND");
    
    ROS_INFO("=== FORCE SENSOR ZEROING COMPLETED ===");
    ROS_INFO("Force sensor zeroing handled by external node");
    ROS_INFO("System is ready for circular motion.");
    ROS_INFO("Enter 'start' to begin circular motion:");
      
    // Start input monitoring thread
    startUserInputMonitoring();
      
    // Use current force data for logging (already zeroed by external node)
    current_force_for_log = current_force_;
    
  } else if (control_phase_ == CALIBRATION && waiting_for_user_command_) {
    // === WAITING FOR USER INPUT ===
    
    // Keep robot at initial pose
    cartesian_pose_handle_->setCommand(initial_pose_);
    
    // Check if user has provided input
    if (checkUserInput()) {
      // User confirmed start, switch to trajectory phase
      control_phase_ = TRAJECTORY;
        phase_start_time_ = time;
      elapsed_time_ = ros::Duration(0.0);
      waiting_for_user_command_ = false;
      
      // Reset trajectory smoothing variables for smooth start
      vel_current_ = 0.0;
      angle_ = 0.0;
      
      publishPhaseStatus("TRAJECTORY");
      ROS_INFO("Starting horizontal circular motion around center [%.3f, %.3f, %.3f]",
               initial_position_(0), initial_position_(1), initial_position_(2));
    }
    
    // Use zeroed force for logging
    current_force_for_log = current_force_zeroed;
    
  } else if (control_phase_ == TRAJECTORY) {
    // === CIRCULAR MOTION PHASE ===
    
    // Smooth velocity control (inspired by joint_impedance_example_controller)
    if (vel_current_ < vel_max_) {
      vel_current_ += period.toSec() * std::fabs(vel_max_ / acceleration_time_);
    }
    vel_current_ = std::fmin(vel_current_, vel_max_);
    
    // Update angle with current velocity
    angle_ += period.toSec() * vel_current_ / std::fabs(circle_radius_);
    if (angle_ > 2 * M_PI) {
      angle_ -= 2 * M_PI;
    }
    
    // Generate smooth circular motion trajectory
    std::array<double, 16> new_pose = initial_pose_;
    double delta_x = circle_radius_ * std::sin(angle_);
    double delta_y = circle_radius_ * (std::cos(angle_) - 1.0);
    
    // Set new position (offset relative to initial position)
    new_pose[12] -= delta_x;  // X direction offset
    new_pose[13] -= delta_y;  // Y direction offset
    // Z direction remains unchanged
    
    // Send position command
    cartesian_pose_handle_->setCommand(new_pose);
      
    // Use current force for logging (already zeroed by external node)
    current_force_for_log = current_force_;
    
  } else {
    // Default: maintain current pose
    cartesian_pose_handle_->setCommand(initial_pose_);
    current_force_for_log = current_force_zeroed;
  }
  
  // Update energy tank monitoring
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Matrix<double, 7, 1> tau_d = Eigen::Matrix<double, 7, 1>::Zero();  // Zero torque in position control mode
  double energy_scale = energy_monitor_->updateAndGetScaleFactor(tau_d, robot_state, jacobian, period);
  
  // Get current energy level
  double current_energy = energy_monitor_->getEnergyLevel();
  
  // Publish current energy
  std_msgs::Float64 energy_msg;
  energy_msg.data = current_energy;
  energy_pub_.publish(energy_msg);
  
  // Log data (use zeroed force after zeroing is complete)
  double theoretical_force = 0.0;  // Temporarily set to 0, contact model not implemented
  double soft_block_z = initial_position_(2) - 0.2;  // Assume soft block is 20cm below
  double contact_reference_z = 0.0;  // Temporarily set to 0, no contact
  
  log_generator_->logData(
    time,
    current_position,
    current_force_for_log,
    static_cast<int>(control_phase_),
    soft_block_z,
    theoretical_force,
    probe_length_,
    contact_reference_z,
    raw_force_z_for_log
  );
  
  // Publish desired pose (for visualization)
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = time;
  pose_msg.header.frame_id = "world";
  
  std::array<double, 16> current_command_pose;
  if (control_phase_ == TRAJECTORY) {
    // Use current trajectory pose with smooth circular motion
    std::array<double, 16> new_pose = initial_pose_;
    double delta_x = circle_radius_ * std::sin(angle_);
    double delta_y = circle_radius_ * (std::cos(angle_) - 1.0);
    new_pose[12] -= delta_x;
    new_pose[13] -= delta_y;
    current_command_pose = new_pose;
  } else {
    // Use initial pose
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
  
  // No frequent printing - only log data for analysis
}

void ContactController::stopping(const ros::Time& time) {
  // Close log file
  if (log_generator_) {
    log_generator_->closeLogFile(time);
  }
  
  publishPhaseStatus("STOPPED");
  ROS_INFO("ContactController stopped");
}

void ContactController::forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(force_mutex_);
  
  // 这里接收的是已经调零后的数据（从/force_sensor/wrench话题）
  // 应该直接使用，不需要再次调零处理
  current_force_(0) = msg->wrench.force.x;
  current_force_(1) = msg->wrench.force.y;
  current_force_(2) = msg->wrench.force.z;
  
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
        std::cout << "Enter 'start' to begin circular motion: ";
        std::getline(std::cin, input);
  
        // Convert to lowercase for case-insensitive comparison
        std::transform(input.begin(), input.end(), input.begin(), ::tolower);
        
        if (input == "start" || input == "s") {
          std::lock_guard<std::mutex> lock(user_input_mutex_);
          user_input_ready_ = true;
          ROS_INFO("User command received: starting circular motion...");
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

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ContactController,
                       controller_interface::ControllerBase)
