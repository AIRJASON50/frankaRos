// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/contact_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <iostream>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka/robot_state.h>
#include <franka_hw/franka_model_interface.h>
#include <Eigen/Dense>

namespace franka_example_controllers {

bool ContactController::init(hardware_interface::RobotHW* robot_hardware,
                            ros::NodeHandle& node_handle) {
  ROS_INFO("正在初始化接触控制器...");

  // 加载配置参数
  if (!loadParameters(node_handle)) {
    ROS_ERROR("加载配置参数失败");
    return false;
  }

  // 获取模型接口
  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR("ContactController: 无法获取模型接口");
    return false;
  }

  // 获取状态接口
  auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("ContactController: 无法获取状态接口");
    return false;
  }

  // 获取关节力矩接口
  auto* effort_joint_interface = robot_hardware->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR("ContactController: 无法获取关节力矩接口");
    return false;
  }

  // 获取机械臂ID
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ContactController: 无法获取arm_id参数");
    return false;
  }

  try {
    // 初始化状态句柄
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));

    // 初始化模型句柄
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_robot"));

    // 初始化关节句柄
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
      ROS_ERROR("ContactController: 无效的关节名称配置");
      return false;
    }

    for (const auto& joint_name : joint_names) {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_name));
    }
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("ContactController: 硬件接口异常: " << ex.what());
    return false;
  }

  // 检查初始位置是否安全
  franka::RobotState robot_state = state_handle_->getRobotState();
  if (!checkSafetyPosition(robot_state.q)) {
    ROS_ERROR("ContactController: 机器人不在安全的起始位置");
    return false;
  }

  // 初始化力传感器
  if (!initializeForceSensor()) {
    ROS_ERROR("ContactController: 力传感器初始化失败");
    return false;
  }

  // 初始化日志系统
  if (!initializeLogging()) {
    ROS_ERROR("ContactController: 日志系统初始化失败");
    return false;
  }

  // 初始化控制状态
  current_phase_ = ControlPhase::CALIBRATION;
  phase_transition_requested_ = false;
  calibration_completed_ = false;
  stable_force_detected_ = false;
  waiting_for_user_input_ = false;
  force_data_valid_ = false;
  
  // 设置日志记录周期
  log_period_ = 1.0 / logging_params_.frequency;  // 2Hz -> 0.5s

  ROS_INFO("接触控制器初始化完成");
  return true;
}

void ContactController::starting(const ros::Time& time) {
  ROS_INFO("启动接触控制器...");
  
  start_time_ = time;
  phase_start_time_ = time;
  last_log_time_ = time;
  
  // 获取初始位姿
  franka::RobotState robot_state = state_handle_->getRobotState();
  initial_pose_ = robot_state.O_T_EE_d;
  
  // 提取初始位置
  initial_position_ << initial_pose_[12], initial_pose_[13], initial_pose_[14];
  current_position_ = initial_position_;
  target_position_ = initial_position_;
  
  ROS_INFO_STREAM("初始位置: [" << initial_position_.transpose() << "]");
  
  // 启动力传感器
  force_sensor_->start();
  
  // 初始化调零状态
  calibration_start_ = std::chrono::steady_clock::now();
  stable_force_start_ = std::chrono::steady_clock::now();
  
  printPhaseInfo("进入调零阶段 - 请确保探头未受外力干扰");
}

void ContactController::update(const ros::Time& time, const ros::Duration& period) {
  // 获取机器人状态
  franka::RobotState robot_state = state_handle_->getRobotState();
  
  // 更新当前位置
  current_position_ << robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14];
  
  // 检查工作空间限制
  if (!checkWorkspaceLimits(current_position_)) {
    ROS_ERROR("机器人超出工作空间限制，紧急停止");
    stopping(time);
    return;
  }
  
  // 获取力传感器数据
  if (force_sensor_->isDataReady()) {
    current_force_ = force_sensor_->getRawForce();
    calibrated_force_ = force_sensor_->getCalibratedForce();
    force_data_valid_ = true;
  }
  
  // 检查紧急停止条件
  if (force_data_valid_ && calibrated_force_.norm() > safety_params_.emergency_stop_force) {
    ROS_ERROR_STREAM("检测到过大外力: " << calibrated_force_.norm() << "N，紧急停止");
    stopping(time);
    return;
  }
  
  // 根据当前阶段执行控制逻辑
  switch (current_phase_) {
    case ControlPhase::CALIBRATION:
      updateCalibrationPhase(time, period);
      break;
    case ControlPhase::APPROACH:
      updateApproachPhase(time, period);
      break;
    case ControlPhase::CONTACT_CONFIRMATION:
      updateContactConfirmationPhase(time, period);
      break;
    case ControlPhase::DEPTH_CONTROL:
      updateDepthControlPhase(time, period);
      break;
    case ControlPhase::TRAJECTORY:
      updateTrajectoryPhase(time, period);
      break;
  }
  
  // 计算关节力矩命令
  std::array<double, 7> tau_d = {{0, 0, 0, 0, 0, 0, 0}};
  positionControl(target_position_, period, tau_d);
  
  // 应用力矩率限制
  tau_d = saturateTorqueRate(tau_d, robot_state.tau_J_d);
  
  // 发送关节力矩命令
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d[i]);
  }
  
  // 记录实验数据
  logExperimentData(time);
}

void ContactController::stopping(const ros::Time& time) {
  ROS_INFO("停止接触控制器...");
  
  // 停止力传感器
  if (force_sensor_) {
    force_sensor_->stop();
  }
  
  // 关闭日志文件
  if (log_generator_ && logging_params_.enable) {
    log_generator_->closeLogFile(time);
  }
  
  ROS_INFO("接触控制器已停止");
}

bool ContactController::loadParameters(ros::NodeHandle& node_handle) {
  ROS_INFO("正在加载配置参数...");
  
  // 从YAML文件加载参数
  std::string config_path = "/home/jason/ws/catkin_ws/src/franka_ros/franka_example_controllers/config/simulation_parameters_config.yaml";
  
  // 力传感器参数
  node_handle.param("force_sensor/serial_port", force_sensor_params_.serial_port, std::string("/dev/ttyUSB0"));
  node_handle.param("force_sensor/baudrate", force_sensor_params_.baudrate, 115200);
  node_handle.param("force_sensor/sampling_frequency", force_sensor_params_.sampling_frequency, 1000);
  node_handle.param("force_sensor/data_timeout", force_sensor_params_.data_timeout, 1000);
  
  // 调零参数
  node_handle.param("calibration/window_size", calibration_params_.window_size, 100);
  node_handle.param("calibration/stable_threshold", calibration_params_.stable_threshold, 0.5);
  node_handle.param("calibration/stable_duration", calibration_params_.stable_duration, 5.0);
  node_handle.param("calibration/max_calibration_time", calibration_params_.max_calibration_time, 60.0);
  node_handle.param("calibration/force_limit", calibration_params_.force_limit, 100.0);
  
  // 接近阶段参数
  node_handle.param("approach/start_delay", approach_params_.start_delay, 1.0);
  node_handle.param("approach/descent_velocity", approach_params_.descent_velocity, 0.005);
  node_handle.param("approach/contact_threshold", approach_params_.contact_threshold, 0.5);
  node_handle.param("approach/safety_force_limit", approach_params_.safety_force_limit, 10.0);
  
  // 接触确认参数
  node_handle.param("contact_confirmation/confirmation_time", contact_params_.confirmation_time, 1.0);
  node_handle.param("contact_confirmation/force_stability_threshold", contact_params_.force_stability_threshold, 0.2);
  
  // 深度控制参数
  node_handle.param("depth_control/target_depth", depth_params_.target_depth, 0.0005);
  node_handle.param("depth_control/real_target_depth", depth_params_.real_target_depth, 0.0005);
  node_handle.param("depth_control/stability_time", depth_params_.stability_time, 3.0);
  node_handle.param("depth_control/depth_tolerance", depth_params_.depth_tolerance, 0.0001);
  node_handle.param("depth_control/control_gains/kp", depth_params_.control_gains.kp, 1000.0);
  node_handle.param("depth_control/control_gains/ki", depth_params_.control_gains.ki, 50.0);
  node_handle.param("depth_control/control_gains/kd", depth_params_.control_gains.kd, 20.0);
  
  // 控制参数
  node_handle.param("controller/control_frequency", control_params_.control_frequency, 1000);
  std::vector<double> position_kp, position_kd;
  node_handle.param("controller/position_gains/kp", position_kp, std::vector<double>{600, 600, 600});
  node_handle.param("controller/position_gains/kd", position_kd, std::vector<double>{50, 50, 50});
  
  if (position_kp.size() == 3 && position_kd.size() == 3) {
    std::copy(position_kp.begin(), position_kp.end(), control_params_.position_kp.begin());
    std::copy(position_kd.begin(), position_kd.end(), control_params_.position_kd.begin());
  }
  
  // 日志参数
  node_handle.param("logging/enable", logging_params_.enable, true);
  node_handle.param("logging/frequency", logging_params_.frequency, 2.0);
  node_handle.param("logging/data_directory", logging_params_.data_directory, 
                   std::string("/home/jason/ws/catkin_ws/src/franka_ros/franka_example_controllers/data"));
  node_handle.param("logging/file_prefix", logging_params_.file_prefix, std::string("contact_experiment"));
  
  // 安全参数
  node_handle.param("safety/emergency_stop_force", safety_params_.emergency_stop_force, 20.0);
  node_handle.param("safety/workspace_limits/x_min", safety_params_.workspace_limits.x_min, 0.2);
  node_handle.param("safety/workspace_limits/x_max", safety_params_.workspace_limits.x_max, 0.8);
  node_handle.param("safety/workspace_limits/y_min", safety_params_.workspace_limits.y_min, -0.4);
  node_handle.param("safety/workspace_limits/y_max", safety_params_.workspace_limits.y_max, 0.4);
  node_handle.param("safety/workspace_limits/z_min", safety_params_.workspace_limits.z_min, 0.1);
  node_handle.param("safety/workspace_limits/z_max", safety_params_.workspace_limits.z_max, 0.8);
  
  // 能量罐参数
  std::vector<double> max_torque, max_velocity, torque_rate;
  node_handle.param("energy_tank/max_joint_torque", max_torque, 
                   std::vector<double>{87, 87, 87, 87, 12, 12, 12});
  node_handle.param("energy_tank/max_joint_velocity", max_velocity,
                   std::vector<double>{2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100});
  node_handle.param("energy_tank/torque_rate_limit", torque_rate,
                   std::vector<double>{1000, 1000, 1000, 1000, 1000, 1000, 1000});
  
  if (max_torque.size() == 7 && max_velocity.size() == 7 && torque_rate.size() == 7) {
    std::copy(max_torque.begin(), max_torque.end(), energy_tank_params_.max_joint_torque.begin());
    std::copy(max_velocity.begin(), max_velocity.end(), energy_tank_params_.max_joint_velocity.begin());
    std::copy(torque_rate.begin(), torque_rate.end(), energy_tank_params_.torque_rate_limit.begin());
  }
  
  ROS_INFO("配置参数加载完成");
  return true;
}

bool ContactController::initializeForceSensor() {
  ROS_INFO("正在初始化力传感器...");
  
  force_sensor_ = std::make_unique<ForceSensorManager>();
  
  // 检测运行模式（仿真或实机）
  ForceSensorManager::Mode mode = ForceSensorManager::REAL_ROBOT;
  
  // 可以通过ROS参数或环境变量来切换模式
  std::string run_mode;
  ros::param::get("~force_sensor_mode", run_mode);
  if (run_mode == "simulation") {
    mode = ForceSensorManager::SIMULATION;
    ROS_INFO("力传感器运行在仿真模式");
  } else {
    ROS_INFO("力传感器运行在实机模式");
  }
  
  if (!force_sensor_->init(mode, force_sensor_params_.serial_port, force_sensor_params_.sampling_frequency)) {
    ROS_ERROR("力传感器初始化失败");
    return false;
  }
  
  ROS_INFO("力传感器初始化完成");
  return true;
}

bool ContactController::initializeLogging() {
  if (!logging_params_.enable) {
    ROS_INFO("日志记录已禁用");
    return true;
  }
  
  ROS_INFO("正在初始化日志系统...");
  
  log_generator_ = std::make_unique<LogGenerator>();
  
  // 这里需要初始化contact参数 - 暂时使用默认值
  ContactParams contact_params;
  contact_params.young_modulus = 100000;  // 默认杨氏模量
  contact_params.poisson_ratio = 0.3;     // 默认泊松比
  contact_params.contact_radius = 0.005;  // 默认接触半径
  contact_params.depth_threshold = 0.001; // 默认深度阈值
  
  double target_force = 1.0; // 默认目标力
  
  log_generator_->initLogFile(contact_params, target_force);
  
  ROS_INFO("日志系统初始化完成");
  return true;
}

bool ContactController::checkSafetyPosition(const std::array<double, 7>& q) {
  // 检查关节位置是否在安全范围内
  // 这里使用Franka的默认安全起始位置
  std::array<double, 7> q_safe{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  
  for (size_t i = 0; i < q.size(); i++) {
    if (std::abs(q[i] - q_safe[i]) > 0.2) {  // 允许0.2弧度的偏差
      ROS_WARN_STREAM("关节 " << i << " 位置偏差过大: " << (q[i] - q_safe[i]));
      return false;
    }
  }
  
  return true;
}

void ContactController::updateCalibrationPhase(const ros::Time& time, const ros::Duration& period) {
  // 计算调零阶段已经进行的时间
  auto current_time = std::chrono::steady_clock::now();
  auto calibration_duration = std::chrono::duration_cast<std::chrono::seconds>(
      current_time - calibration_start_).count();
  
  // 检查是否超过最大调零时间
  if (calibration_duration > calibration_params_.max_calibration_time) {
    ROS_ERROR("调零时间超时，请检查力传感器连接");
    return;
  }
  
  // 在调零阶段保持当前位置不动
  target_position_ = initial_position_;
  
  // 检查力传感器数据是否有效
  if (!force_data_valid_) {
    printPhaseInfo("等待力传感器数据...");
    return;
  }
  
  // 检查调零是否完成
  if (force_sensor_->isZeroingCompleted()) {
    if (!calibration_completed_) {
      printPhaseInfo("力传感器调零完成！");
      calibration_completed_ = true;
      
      // 检查调零后的力是否稳定
      double force_magnitude = calibrated_force_.norm();
      
      if (force_magnitude < calibration_params_.stable_threshold) {
        if (!stable_force_detected_) {
          stable_force_start_ = current_time;
          stable_force_detected_ = true;
          printPhaseInfo("检测到稳定的力信号，开始计时...");
        }
        
        // 检查力是否已经稳定足够长时间
        auto stable_duration = std::chrono::duration_cast<std::chrono::seconds>(
            current_time - stable_force_start_).count();
        
        if (stable_duration >= calibration_params_.stable_duration) {
          if (!waiting_for_user_input_) {
            printPhaseInfo("调零完成！力信号已稳定 " + std::to_string(stable_duration) + " 秒");
            printPhaseInfo("按Enter键继续进入接近阶段...");
            waiting_for_user_input_ = true;
            
            // 在后台线程中等待用户输入
            std::thread input_thread([this]() {
              waitForUserInput();
            });
            input_thread.detach();
          }
        } else {
          // 每2秒打印一次进度
          static auto last_progress_time = current_time;
          if (std::chrono::duration_cast<std::chrono::seconds>(
                current_time - last_progress_time).count() >= 2) {
            printPhaseInfo("力信号稳定中... (" + std::to_string(stable_duration) + "/" + 
                          std::to_string((int)calibration_params_.stable_duration) + "秒)");
            last_progress_time = current_time;
          }
        }
      } else {
        // 力信号不稳定，重置稳定状态
        if (stable_force_detected_) {
          stable_force_detected_ = false;
          printPhaseInfo("力信号不稳定，重新开始稳定性检测...");
        }
      }
    }
  } else {
    // 仍在调零过程中
    static auto last_progress_time = current_time;
    if (std::chrono::duration_cast<std::chrono::seconds>(
          current_time - last_progress_time).count() >= 3) {
      printPhaseInfo("正在进行力传感器调零... (" + std::to_string(calibration_duration) + "秒)");
      last_progress_time = current_time;
    }
  }
  
  // 检查是否请求阶段转换
  if (phase_transition_requested_.load()) {
    transitionToPhase(ControlPhase::APPROACH);
    phase_transition_requested_ = false;
  }
}

void ContactController::updateApproachPhase(const ros::Time& time, const ros::Duration& period) {
  // TODO: 实现接近阶段逻辑
  printPhaseInfo("接近阶段 - 尚未实现");
}

void ContactController::updateContactConfirmationPhase(const ros::Time& time, const ros::Duration& period) {
  // TODO: 实现接触确认阶段逻辑
  printPhaseInfo("接触确认阶段 - 尚未实现");
}

void ContactController::updateDepthControlPhase(const ros::Time& time, const ros::Duration& period) {
  // TODO: 实现深度控制阶段逻辑
  printPhaseInfo("深度控制阶段 - 尚未实现");
}

void ContactController::updateTrajectoryPhase(const ros::Time& time, const ros::Duration& period) {
  // TODO: 实现轨迹运动阶段逻辑
  printPhaseInfo("轨迹运动阶段 - 尚未实现");
}

void ContactController::positionControl(const Eigen::Vector3d& target_pos, 
                                       const ros::Duration& period,
                                       std::array<double, 7>& tau_d) {
  // 获取机器人状态
  franka::RobotState robot_state = state_handle_->getRobotState();
  
  // 获取雅可比矩阵
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  
  // 获取当前位置和姿态
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position = transform.translation();
  Eigen::Quaterniond orientation(transform.linear());
  
  // 计算位置误差
  Eigen::Vector3d position_error = target_pos - position;
  
  // PD控制器
  Eigen::Vector3d velocity = jacobian.topRows(3) * Eigen::Map<const Eigen::Matrix<double, 7, 1>>(robot_state.dq.data());
  
  Eigen::Vector3d force_cmd = Eigen::Vector3d::Zero();
  for (int i = 0; i < 3; ++i) {
    force_cmd(i) = control_params_.position_kp[i] * position_error(i) - control_params_.position_kd[i] * velocity(i);
  }
  
  // 转换为关节力矩命令
  Eigen::VectorXd tau_task = jacobian.topRows(3).transpose() * force_cmd;
  
  // 获取重力补偿
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  
  // 组合总的力矩命令
  Eigen::VectorXd tau_total = tau_task + gravity;
  
  // 复制到输出数组
  for (size_t i = 0; i < 7; ++i) {
    tau_d[i] = tau_total(i);
  }
}

std::array<double, 7> ContactController::saturateTorqueRate(const std::array<double, 7>& tau_d_calculated,
                                                           const std::array<double, 7>& tau_J_d) {
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, energy_tank_params_.torque_rate_limit[i] * 0.001),
                                              -energy_tank_params_.torque_rate_limit[i] * 0.001);
  }
  return tau_d_saturated;
}

bool ContactController::checkWorkspaceLimits(const Eigen::Vector3d& position) {
  return (position.x() >= safety_params_.workspace_limits.x_min &&
          position.x() <= safety_params_.workspace_limits.x_max &&
          position.y() >= safety_params_.workspace_limits.y_min &&
          position.y() <= safety_params_.workspace_limits.y_max &&
          position.z() >= safety_params_.workspace_limits.z_min &&
          position.z() <= safety_params_.workspace_limits.z_max);
}

void ContactController::transitionToPhase(ControlPhase new_phase) {
  ControlPhase old_phase = current_phase_;
  current_phase_ = new_phase;
  phase_start_time_ = ros::Time::now();
  
  std::string phase_names[] = {"调零", "接近", "接触确认", "深度控制", "轨迹运动"};
  ROS_INFO_STREAM("阶段转换: " << phase_names[static_cast<int>(old_phase)] 
                  << " -> " << phase_names[static_cast<int>(new_phase)]);
}

void ContactController::logExperimentData(const ros::Time& time) {
  if (!logging_params_.enable || !log_generator_) {
    return;
  }
  
  // 限制日志记录频率
  double elapsed = (time - last_log_time_).toSec();
  if (elapsed < log_period_) {
    return;
  }
  
  last_log_time_ = time;
  
  // 记录数据
  if (force_data_valid_) {
    double soft_block_z = 0.0;  // 暂时使用默认值
    double theoretical_force = 0.0;  // 暂时使用默认值
    double probe_length = 0.05;  // 默认探头长度
    double contact_reference_z = 0.0;  // 暂时使用默认值
    
    log_generator_->logData(time, current_position_, calibrated_force_, 
                           static_cast<int>(current_phase_), soft_block_z,
                           theoretical_force, probe_length, contact_reference_z,
                           current_force_.z());
  }
}

void ContactController::printPhaseInfo(const std::string& message) {
  static auto last_print_time = std::chrono::steady_clock::now();
  auto current_time = std::chrono::steady_clock::now();
  
  // 限制打印频率到2Hz
  if (std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_print_time).count() >= 500) {
    ROS_INFO_STREAM("[ContactController] " << message);
    last_print_time = current_time;
  }
}

void ContactController::waitForUserInput() {
  std::cout << "\n按 Enter 键继续...";
  std::cin.ignore();
  std::cin.get();
  
  // 设置阶段转换标志
  phase_transition_requested_ = true;
  waiting_for_user_input_ = false;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ContactController,
                       controller_interface::ControllerBase)
