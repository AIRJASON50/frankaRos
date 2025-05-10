// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/circle_controller.h>

#include <cmath>
#include <memory>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <random>

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

// 获取格式化的时间字符串
std::string getFormattedTime() {
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  char buffer[80];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", std::localtime(&time_t_now));
  return std::string(buffer);
}

// 初始化日志文件
void CircleController::initLogFile() {
  if (log_initialized_) {
    return;
  }
  
  try {
    // 记录开始时间
    start_time_ = ros::Time::now();
    
    // 获取日志文件路径，使用格式化的时间作为文件名
    std::string package_path = ros::package::getPath("franka_gazebo");
    std::string time_str = getFormattedTime();
    log_file_path_ = package_path + "/logs/force_data_" + time_str + ".csv";
    
    // 打开日志文件
    log_file_.open(log_file_path_);
    
    if (log_file_.is_open()) {
      // 记录开始时间信息
      log_file_ << "# 实验开始时间: " << std::fixed << std::setprecision(6) 
                << start_time_.toSec() << " 秒" << std::endl;
      log_file_ << "# 本地时间: " << time_str << std::endl;
      log_file_ << "# 控制阶段说明: 0=APPROACH, 1=CONTACT, 2=TRAJECTORY" << std::endl;
      log_file_ << "# 软块物理属性: 杨氏模量=" << contact_params_.young_modulus 
                << "Pa, 泊松比=" << contact_params_.poisson_ratio << std::endl;
      log_file_ << "# 目标力: " << target_force_ << "N" << std::endl;
      log_file_ << "# ---------------------------------------" << std::endl;
      
      // 写入CSV头
      log_file_ << "time,phase,pos_x,pos_y,pos_z,force_x,force_y,force_z,force_magnitude,depth,target_force" << std::endl;
      log_initialized_ = true;
      ROS_INFO_STREAM("已创建日志文件: " << log_file_path_);
    } else {
      ROS_ERROR_STREAM("无法打开日志文件: " << log_file_path_);
    }
  } catch (const std::exception& ex) {
    ROS_ERROR_STREAM("初始化日志文件时出错: " << ex.what());
  }
}

// 关闭日志文件并记录结束时间
void CircleController::closeLogFile(const ros::Time& end_time) {
  if (!log_initialized_ || !log_file_.is_open()) {
    return;
  }
  
  try {
    // 计算实验总时长
    double duration = (end_time - start_time_).toSec();
    
    // 写入结束时间和总时长信息
    log_file_ << "# ---------------------------------------" << std::endl;
    log_file_ << "# 实验结束时间: " << std::fixed << std::setprecision(6) 
              << end_time.toSec() << " 秒" << std::endl;
    log_file_ << "# 实验总时长: " << duration << " 秒" << std::endl;
    log_file_ << "# 记录的数据点数: " << log_counter_ / 10 << std::endl;
    
    // 关闭文件
    log_file_.close();
    ROS_INFO_STREAM("已关闭日志文件: " << log_file_path_);
    ROS_INFO_STREAM("实验总时长: " << duration << " 秒");
  } catch (const std::exception& ex) {
    ROS_ERROR_STREAM("关闭日志文件时出错: " << ex.what());
  }
}

// 记录数据到日志文件
void CircleController::logData(const ros::Time& time, const Eigen::Vector3d& position, 
                              const Eigen::Vector3d& force, int phase) {
  if (!log_initialized_ || !log_file_.is_open()) {
    return;
  }
  
  try {
    // 每10条记录写入一次（降低I/O开销）
    if (log_counter_++ % 10 == 0) {
      // 计算力大小
      double force_magnitude = force.norm();
      
      // 计算深度
      double depth = soft_block_position_(2) - position(2);
      
      // 写入CSV格式数据
      log_file_ << std::fixed << std::setprecision(6)
                << time.toSec() << ","
                << phase << ","
                << position(0) << ","
                << position(1) << ","
                << position(2) << ","
                << force(0) << ","
                << force(1) << ","
                << force(2) << ","
                << force_magnitude << ","
                << depth << ","
                << target_force_
                << std::endl;
      
      // 每100条记录刷新一次文件（确保数据写入磁盘）
      if (log_counter_ % 100 == 0) {
        log_file_.flush();
      }
    }
  } catch (const std::exception& ex) {
    ROS_ERROR_STREAM("记录数据到日志文件时出错: " << ex.what());
  }
}

// 在停止控制器时关闭日志文件
void CircleController::stopping(const ros::Time& time) {
  // 记录结束时间并关闭日志文件
  closeLogFile(time);
}

// 轨迹生成函数 - 根据当前轨迹类型生成目标位置
Eigen::Vector3d CircleController::generateTrajectory(double time) {
  // 应用速度因子来调整轨迹速度（speed_factor_越小，速度越慢）
  double adjusted_time = time * speed_factor_;
  
  // 增加预测时间以减少跟踪延迟 - 预测机械臂的控制延迟
  double prediction_time = 0.05; // 预测50ms的轨迹，提前确定目标位置
  double predicted_time = adjusted_time + prediction_time;
  
  // 根据轨迹类型生成目标位置
  switch (trajectory_type_) {
    case CIRCULAR:
      return generateCircularTrajectory(predicted_time);
    case RECTANGULAR:
      return generateRectangularTrajectory(predicted_time);
    case FIGURE_EIGHT:
      return generateFigureEightTrajectory(predicted_time);
    case LINE:
      return generateLineTrajectory(predicted_time);
    case CUSTOM:
      return generateCustomTrajectory(predicted_time);
    default:
      return generateCircularTrajectory(predicted_time);
  }
}

// 生成圆形轨迹
Eigen::Vector3d CircleController::generateCircularTrajectory(double time) {
  // 计算当前角度
  double angle = 2.0 * M_PI * circle_frequency_ * time;
  
  // 平滑启动：在运动开始的第一秒内使用余弦函数平滑过渡
  double smooth_factor = 1.0;
  if (time < 1.0) {
    smooth_factor = 0.5 * (1.0 - std::cos(M_PI * time));
  }
  double smooth_angle = smooth_factor * angle;  // 应用平滑因子到角度
  
  // 生成圆形轨迹点
  return circle_center_ + 
         circle_radius_ * cos(smooth_angle) * circle_x_axis_ + 
         circle_radius_ * sin(smooth_angle) * circle_y_axis_;
}

// 生成矩形轨迹
Eigen::Vector3d CircleController::generateRectangularTrajectory(double time) {
  // 计算一个完整矩形的周期时间
  double period = 1.0 / circle_frequency_; 
  // 将时间归一化到0-1之间，表示一个周期内的相对位置
  double normalized_time = fmod(time, period) / period;
  
  // 平滑启动：在运动开始的第一秒内使用余弦函数平滑过渡
  double smooth_factor = 1.0;
  if (time < 1.0) {
    smooth_factor = 0.5 * (1.0 - std::cos(M_PI * time));
    // 如果在平滑阶段，限制在第一条边
    normalized_time = normalized_time * 0.25;
  }
  
  // 矩形的四条边
  if (normalized_time < 0.25) {
    // 第一条边：从左下到右下
    double t = normalized_time * 4.0;
    return circle_center_ + 
           smooth_factor * ((t * rect_length_ - rect_length_/2) * circle_x_axis_ - 
           (rect_width_/2) * circle_y_axis_);
  } else if (normalized_time < 0.5) {
    // 第二条边：从右下到右上
    double t = (normalized_time - 0.25) * 4.0;
    return circle_center_ + 
           smooth_factor * ((rect_length_/2) * circle_x_axis_ + 
           (t * rect_width_ - rect_width_/2) * circle_y_axis_);
  } else if (normalized_time < 0.75) {
    // 第三条边：从右上到左上
    double t = (normalized_time - 0.5) * 4.0;
    return circle_center_ + 
           smooth_factor * ((1-t) * rect_length_ - rect_length_/2) * circle_x_axis_ + 
           smooth_factor * (rect_width_/2) * circle_y_axis_;
  } else {
    // 第四条边：从左上到左下
    double t = (normalized_time - 0.75) * 4.0;
    return circle_center_ + 
           smooth_factor * (-rect_length_/2) * circle_x_axis_ + 
           smooth_factor * ((1-t) * rect_width_ - rect_width_/2) * circle_y_axis_;
  }
}

// 生成八字形轨迹
Eigen::Vector3d CircleController::generateFigureEightTrajectory(double time) {
  // 计算当前角度
  double angle = 2.0 * M_PI * circle_frequency_ * time;
  
  // 平滑启动：在运动开始的第一秒内使用余弦函数平滑过渡
  double smooth_factor = 1.0;
  if (time < 1.0) {
    smooth_factor = 0.5 * (1.0 - std::cos(M_PI * time));
  }
  
  // 生成8字形轨迹点 - 使用参数方程
  // x = a * sin(t)
  // y = b * sin(2*t)
  return circle_center_ + 
         smooth_factor * eight_radius_x_ * sin(angle) * circle_x_axis_ + 
         smooth_factor * eight_radius_y_ * sin(2*angle) * circle_y_axis_;
}

// 生成直线往返轨迹
Eigen::Vector3d CircleController::generateLineTrajectory(double time) {
  // 计算当前角度 - 使用正弦波产生往返运动
  double angle = 2.0 * M_PI * circle_frequency_ * time;
  
  // 平滑启动：在运动开始的第一秒内使用余弦函数平滑过渡
  double smooth_factor = 1.0;
  if (time < 1.0) {
    smooth_factor = 0.5 * (1.0 - std::cos(M_PI * time));
  }
  
  // 生成直线往返轨迹点 - 使用正弦函数
  return circle_center_ + 
         smooth_factor * (line_length_ * sin(angle) / 2) * circle_x_axis_;
}

// 生成自定义轨迹
Eigen::Vector3d CircleController::generateCustomTrajectory(double time) {
  // 检查是否有轨迹点
  if (custom_trajectory_points_.empty()) {
    ROS_WARN_ONCE("自定义轨迹为空，返回中心位置");
    return circle_center_;
  }
  
  // 平滑启动：在运动开始的第一秒内使用余弦函数平滑过渡
  double smooth_factor = 1.0;
  if (time < 1.0) {
    smooth_factor = 0.5 * (1.0 - std::cos(M_PI * time));
  }
  
  // 计算周期性轨迹时间
  double trajectory_duration = custom_trajectory_points_.back().time;
  double cyclic_time = fmod(time, trajectory_duration);
  
  // 寻找适当的轨迹点进行插值
  for (size_t i = 0; i < custom_trajectory_points_.size() - 1; i++) {
    if (cyclic_time >= custom_trajectory_points_[i].time &&
        cyclic_time < custom_trajectory_points_[i+1].time) {
      // 线性插值两个点之间的位置
      double alpha = (cyclic_time - custom_trajectory_points_[i].time) / 
                    (custom_trajectory_points_[i+1].time - custom_trajectory_points_[i].time);
      
      Eigen::Vector3d interpolated_position = 
          custom_trajectory_points_[i].position * (1 - alpha) + 
          custom_trajectory_points_[i+1].position * alpha;
      
      // 应用平滑因子并返回结果
      return circle_center_ + smooth_factor * (interpolated_position - circle_center_);
    }
  }
  
  // 默认返回第一个点
  return circle_center_ + smooth_factor * (custom_trajectory_points_[0].position - circle_center_);
}

// 加载自定义轨迹
bool CircleController::loadCustomTrajectory(const std::string& filename) {
  try {
    std::ifstream file(filename);
    if (!file.is_open()) {
      ROS_ERROR_STREAM("无法打开轨迹文件: " << filename);
      return false;
    }
    
    custom_trajectory_points_.clear();
    std::string line;
    
    // 跳过标题行
    std::getline(file, line);
    
    // 读取轨迹点：时间,x,y,z格式
    while (std::getline(file, line)) {
      std::istringstream iss(line);
      std::string token;
      
      // 读取时间
      if (!std::getline(iss, token, ',')) continue;
      double time = std::stod(token);
      
      // 读取x坐标
      if (!std::getline(iss, token, ',')) continue;
      double x = std::stod(token);
      
      // 读取y坐标
      if (!std::getline(iss, token, ',')) continue;
      double y = std::stod(token);
      
      // 读取z坐标
      if (!std::getline(iss, token, ',')) continue;
      double z = std::stod(token);
      
      // 创建轨迹点并添加到列表
      TrajectoryPoint point;
      point.time = time;
      point.position = Eigen::Vector3d(x, y, z);
      custom_trajectory_points_.push_back(point);
    }
    
    if (custom_trajectory_points_.empty()) {
      ROS_ERROR("轨迹文件不包含有效数据点");
      return false;
    }
    
    ROS_INFO_STREAM("成功加载自定义轨迹，共 " << custom_trajectory_points_.size() << " 个点");
    ROS_INFO_STREAM("轨迹持续时间: " << custom_trajectory_points_.back().time << " 秒");
    return true;
  } catch (const std::exception& ex) {
    ROS_ERROR_STREAM("加载自定义轨迹时出错: " << ex.what());
    return false;
  }
}

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

  // 读取轨迹类型参数
  std::string trajectory_type_str;
  if (node_handle.getParam("trajectory_type", trajectory_type_str)) {
    if (trajectory_type_str == "circular") {
      trajectory_type_ = CIRCULAR;
      ROS_INFO_STREAM("CircleController: Using circular trajectory");
    } else if (trajectory_type_str == "rectangular") {
      trajectory_type_ = RECTANGULAR;
      ROS_INFO_STREAM("CircleController: Using rectangular trajectory");
    } else if (trajectory_type_str == "figure_eight") {
      trajectory_type_ = FIGURE_EIGHT;
      ROS_INFO_STREAM("CircleController: Using figure eight trajectory");
    } else if (trajectory_type_str == "line") {
      trajectory_type_ = LINE;
      ROS_INFO_STREAM("CircleController: Using linear trajectory");
    } else if (trajectory_type_str == "custom") {
      trajectory_type_ = CUSTOM;
      ROS_INFO_STREAM("CircleController: Using custom trajectory");
    } else {
      ROS_WARN_STREAM("CircleController: Unknown trajectory type: " << trajectory_type_str 
                     << ". Using default circular trajectory.");
      trajectory_type_ = CIRCULAR;
    }
  } else {
    ROS_INFO_STREAM("CircleController: No trajectory type specified. Using default circular trajectory.");
    trajectory_type_ = CIRCULAR;
  }

  // 从参数服务器读取圆周运动参数
  if (!node_handle.getParam("circle_radius", circle_radius_)) {
    ROS_INFO_STREAM("CircleController: Using default circle radius: " << circle_radius_);
  }
  if (!node_handle.getParam("circle_frequency", circle_frequency_)) {
    ROS_INFO_STREAM("CircleController: Using default circle frequency: " << circle_frequency_);
  }

  // 从参数服务器读取圆周运动平面参数
  if (!node_handle.getParam("circle_plane", circle_plane_)) {
    circle_plane_ = "yz";  // 默认使用YZ平面，垂直于地面
    ROS_INFO_STREAM("CircleController: Using default circle plane: " << circle_plane_);
  }

  // 读取矩形轨迹参数
  node_handle.param<double>("rect_length", rect_length_, 0.2);
  node_handle.param<double>("rect_width", rect_width_, 0.1);
  ROS_INFO_STREAM("CircleController: Rectangle dimensions: " << rect_length_ << "x" << rect_width_ << "m");
  
  // 读取八字形轨迹参数
  node_handle.param<double>("eight_radius_x", eight_radius_x_, 0.1);
  node_handle.param<double>("eight_radius_y", eight_radius_y_, 0.05);
  ROS_INFO_STREAM("CircleController: Figure eight dimensions: " << eight_radius_x_ * 2 << "x" << eight_radius_y_ * 2 << "m");
  
  // 读取直线轨迹参数
  node_handle.param<double>("line_length", line_length_, 0.2);
  ROS_INFO_STREAM("CircleController: Line length: " << line_length_ << "m");
  
  // 读取自定义轨迹文件
  std::string custom_trajectory_file;
  if (node_handle.getParam("custom_trajectory_file", custom_trajectory_file)) {
    custom_trajectory_file_ = custom_trajectory_file;
    ROS_INFO_STREAM("CircleController: Custom trajectory file: " << custom_trajectory_file_);
    if (trajectory_type_ == CUSTOM) {
      if (!loadCustomTrajectory(custom_trajectory_file_)) {
        ROS_WARN("CircleController: Failed to load custom trajectory. Falling back to circular trajectory.");
        trajectory_type_ = CIRCULAR;
      }
    }
  }

  // 从参数服务器读取圆周运动中心点参数
  double center_x = 0.0, center_y = 0.0, center_z = 0.0;
  if (node_handle.getParam("center_x", center_x)) {
    ROS_INFO_STREAM("CircleController: Using specified center_x: " << center_x);
  }
  if (node_handle.getParam("center_y", center_y)) {
    ROS_INFO_STREAM("CircleController: Using specified center_y: " << center_y);
  }
  if (node_handle.getParam("center_z", center_z)) {
    ROS_INFO_STREAM("CircleController: Using specified center_z: " << center_z);
  }

  // 设置圆周运动中心点
  circle_center_ = Eigen::Vector3d(center_x, center_y, center_z);
  ROS_INFO_STREAM("CircleController: Circle center set to: " << circle_center_.transpose());
  
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
  
  // 读取软体块位置
  node_handle.param<double>("soft_block_x", soft_block_x_, 0.4);
  node_handle.param<double>("soft_block_y", soft_block_y_, 0.0);
  node_handle.param<double>("soft_block_z", soft_block_z_, 0.505);
  
  soft_block_position_ = Eigen::Vector3d(soft_block_x_, soft_block_y_, soft_block_z_);
  ROS_INFO_STREAM("CircleController: Soft block position: " << soft_block_position_.transpose());
  
  // 读取力控制参数
  ros::NodeHandle force_nh(node_handle, "force_controller");
  force_nh.getParam("target_force", target_force_);
  base_force_ = target_force_; // 初始化力轨迹基准
  force_nh.getParam("kp", force_p_gain_);
  force_nh.getParam("ki", force_i_gain_);
  force_nh.getParam("kd", force_d_gain_);
  force_nh.getParam("max_force", max_force_);
  
  // 同步力控制参数副本
  force_kp_ = force_p_gain_;
  force_ki_ = force_i_gain_;
  force_kd_ = force_d_gain_;
  
  // 读取相位持续时间
  node_handle.getParam("phase_duration", phase_duration_);
  
  // 读取速度因子参数
  if (!node_handle.getParam("speed_factor", speed_factor_)) {
    ROS_INFO_STREAM("CircleController: Using default speed factor: " << speed_factor_);
  }
  
  // 初始化软接触模型
  contact_model_ = std::make_unique<franka_example_controllers::SoftContactModel>(
      node_handle, contact_params_);
  
  // 初始化发布器
  pose_pub_ = node_handle.advertise<geometry_msgs::PoseStamped>("equilibrium_pose", 10);
  contact_pub_ = node_handle.advertise<geometry_msgs::WrenchStamped>("contact_force", 10);
  phase_pub_ = node_handle.advertise<std_msgs::String>("control_phase", 10);
  path_pub_ = node_handle.advertise<nav_msgs::Path>("trajectory_path", 1);

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
  initLogFile();

  // 读取力轨迹类型参数
  std::string force_profile_type_str;
  if (force_nh.getParam("force_profile_type", force_profile_type_str)) {
    if (force_profile_type_str == "constant") {
      force_profile_type_ = FORCE_CONSTANT;
    } else if (force_profile_type_str == "sine") {
      force_profile_type_ = FORCE_SINE;
    } else if (force_profile_type_str == "step") {
      force_profile_type_ = FORCE_STEP;
    } else {
      ROS_WARN_STREAM("Unknown force_profile_type: " << force_profile_type_str << ", using constant.");
      force_profile_type_ = FORCE_CONSTANT;
    }
  }
  force_nh.param<double>("force_sine_amplitude", force_sine_amplitude_, 0.0);
  force_nh.param<double>("force_sine_frequency", force_sine_frequency_, 1.0);
  force_nh.param<double>("force_step_time", force_step_time_, 5.0);
  force_nh.param<double>("force_step_value", force_step_value_, 0.0);

  // 读取力噪音参数
  force_nh.param("force_noise_enable", force_noise_enable_, false);
  force_nh.param("force_noise_min", force_noise_min_, 1.0);
  force_nh.param("force_noise_max", force_noise_max_, 2.0);

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
    ROS_INFO_STREAM("CircleController: Using initial position as circle center: " 
                    << circle_center_.transpose());
  } else {
    ROS_INFO_STREAM("CircleController: Using specified circle center: " 
                    << circle_center_.transpose());
  }
  
  // 设置初始目标位置和方向
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  
  // 设置零空间平衡配置为初始关节角度
  q_d_nullspace_ = q_initial;
  
  // 重置时间
  elapsed_time_ = 0.0;
  
  // 重置控制阶段
  control_phase_ = APPROACH;
  phase_start_time_ = time;
  
  // 重置力控制变量
  force_error_integral_ = 0.0;
  prev_force_error_ = 0.0;
  last_force_error_ = 0.0;
  
  // 重置接触状态
  contact_detected_ = false;
  contact_position_ = Eigen::Vector3d::Zero();
  
  // 发布初始控制阶段
  std_msgs::String phase_msg;
  phase_msg.data = "APPROACH";
  phase_pub_.publish(phase_msg);
  
  // 输出初始设置信息
  ROS_INFO_STREAM("CircleController: Starting with circle plane: " << circle_plane_ 
                  << ", radius: " << circle_radius_ 
                  << ", frequency: " << circle_frequency_);
  ROS_INFO_STREAM("CircleController: Initial position: " << position_d_.transpose());
  ROS_INFO_STREAM("CircleController: Soft block position: " << soft_block_position_.transpose());
  
  // 重置计数器
  circular_counter_ = 0;
  
  // 发布初始轨迹路径
  publishTrajectoryPath();
}

void CircleController::update(const ros::Time& time,
                             const ros::Duration& period) {
  // 累计经过的时间（用于计算圆周轨迹）
  elapsed_time_ += period.toSec();
  
  // 动态更新目标力
  target_force_ = generateForceProfile(elapsed_time_);
  
  // 更新轨迹路径可视化 - 每10个周期更新一次，平衡性能和可视化效果
  if (circular_counter_ % 10 == 0) {
    publishTrajectoryPath();
  }
  circular_counter_++;
  
  // =====================================================================
  // 1. 获取当前状态：传感器读数
  // =====================================================================
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 49> mass_array = model_handle_->getMass();

  // 将数组转换为Eigen向量/矩阵
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());  // 科里奥利力向量
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());  // 雅可比矩阵：映射关节空间到笛卡尔空间
  Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data());          // 质量矩阵
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());          // 当前关节位置
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());        // 当前关节速度
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(                          // 上一个周期的期望关节力矩
      robot_state.tau_J_d.data());

  // 获取当前末端位姿
  Eigen::Affine3d current_pose(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));  // 当前末端位姿变换矩阵
  Eigen::Vector3d position(current_pose.translation());                           // 当前末端位置
  Eigen::Quaterniond orientation(current_pose.rotation());                        // 当前末端姿态（四元数表示）

  // 计算末端速度
  Eigen::VectorXd v_ee = jacobian * dq;  // 笛卡尔空间速度
  Eigen::Vector3d position_d_dot = Eigen::Vector3d::Zero();  // 期望位置导数（速度）
  Eigen::Vector3d position_d_ddot = Eigen::Vector3d::Zero();  // 期望位置二阶导数（加速度）

  // 从机器人状态获取外部力估计
  Eigen::Map<const Eigen::Matrix<double, 6, 1>> external_wrench(robot_state.O_F_ext_hat_K.data());
  Eigen::Vector3d measured_force = external_wrench.head(3);  // 仅使用力部分
  double force_magnitude = measured_force.norm();

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
  
  // 计算当前位置相对于软体块的深度并强制更新
  double depth = soft_block_position_(2) - position(2);
  bool in_contact = depth > contact_params_.depth_threshold * 0.5;
  
  // 直接使用位置和外部力估计更新接触状态
  ContactState current_contact_state = contact_model_->updateContact(
      position,
      v_ee.head(3),
      orientation,
      soft_block_position_(2)
  );
  
  // 如果在CONTACT或TRAJECTORY阶段，强制设置接触状态
  if (control_phase_ == CONTACT || control_phase_ == TRAJECTORY) {
    // 强制设置接触状态为true
    current_contact_state.setContactState(true);
    
    // 力计算逻辑
    // 1. 如果模型计算的力太小，使用外部力估计
    if (current_contact_state.contact_force < 0.5) {
      // 使用z方向的力（垂直力）
      double z_force = std::abs(measured_force(2));
      
      // 如果外部力也很小，用理论力值
      if (z_force < 0.5) {
        // 根据接触深度计算理论力值
        double theory_force = 0.0;
        if (depth > 0) {
          // 使用赫兹接触理论: F = (4/3) * E_effective * sqrt(R) * depth^(3/2)
          double effective_modulus = contact_params_.young_modulus / (1.0 - std::pow(contact_params_.poisson_ratio, 2));
          theory_force = (4.0 / 3.0) * effective_modulus * 
                        std::sqrt(contact_params_.contact_radius) * 
                        std::pow(depth, 1.5);
        }
        
        // 确保力至少有一个最小值
        double min_force = 1.0; // 最小1N的接触力
        current_contact_state.setForce(std::max(theory_force, min_force));
      } else {
        // 使用测量的外部力
        current_contact_state.setForce(z_force);
      }
      
      // 打印调试信息
      static int debug_count = 0;
      if (debug_count++ % 1000 == 0) {
        ROS_INFO_STREAM("力更新：原始力=" << current_contact_state.contact_force 
                      << "N, 测量力=" << measured_force.norm() 
                      << "N, z方向力=" << measured_force(2) << "N, 深度=" 
                      << depth * 1000 << "mm");
      }
    }
  }
  
  // 发布接触状态
  geometry_msgs::WrenchStamped contact_wrench_msg;
  contact_wrench_msg.header.stamp = time;
  contact_wrench_msg.header.frame_id = "world";
  
  if (current_contact_state.in_contact) {
    // 已检测到接触，调整力
    // 计算力误差
    double force_error = target_force_ - current_contact_state.contact_force;
    
    // 积分项（限制积分饱和）
    force_error_integral_ += force_error * period.toSec();
    force_error_integral_ = std::max(std::min(force_error_integral_, 5.0), -5.0);
    
    // 微分项
    double force_error_derivative = (force_error - prev_force_error_) / period.toSec();
    prev_force_error_ = force_error;
    
    // 力控制PID输出（负号是因为位置增加会减小力）
    double position_adjustment = -(force_p_gain_ * force_error + 
                                 force_i_gain_ * force_error_integral_ + 
                                 force_d_gain_ * force_error_derivative);
    
    // 限制位置调整范围
    position_adjustment = std::max(std::min(position_adjustment, 0.001), -0.001);  // 限制在±1mm
    
    // 仅调整z方向位置（沿法线方向）
    position_d_[2] += position_adjustment;
    
    // 若启用力噪音，给position_d_叠加三维扰动
    if (force_noise_enable_) {
      Eigen::Vector3d noise = generateForceNoise() * 0.001; // 转为最大2mm扰动
      position_d_ += noise;
    }
    
    if (circular_counter_ % 1000 == 0) {
      ROS_INFO_STREAM("力控制: 当前力 " << current_contact_state.contact_force << "N, 目标力 " 
                    << target_force_ << "N, 调整量 " << position_adjustment * 1000 << "mm");
    }
  }

  // 实现赫兹接触理论的力反馈
  // 计算最新的接触力并保存
  Eigen::Vector3d contact_force_vec;
  double actual_contact_force = 0.0;
  
  if (current_contact_state.in_contact) {
    // 获取软接触模型计算的接触力
    actual_contact_force = current_contact_state.contact_force;
    contact_force_vec = current_contact_state.force();
    
    // 计算力误差
    double force_error = target_force_ - actual_contact_force;
    
    // 积分项（限制积分饱和）
    force_error_integral_ += force_error * period.toSec();
    force_error_integral_ = std::max(std::min(force_error_integral_, 5.0), -5.0);
    
    // 微分项
    double force_error_derivative = (force_error - prev_force_error_) / period.toSec();
    prev_force_error_ = force_error;
    
    // 力控制PID输出（负号是因为位置增加会减小力）
    double position_adjustment = -(force_p_gain_ * force_error + 
                                 force_i_gain_ * force_error_integral_ + 
                                 force_d_gain_ * force_error_derivative);
    
    // 限制位置调整范围
    position_adjustment = std::max(std::min(position_adjustment, 0.001), -0.001);  // 限制在±1mm
    
    // 仅调整z方向位置（沿法线方向）
    position_d_[2] += position_adjustment;
    
    // 若启用力噪音，给position_d_叠加三维扰动
    if (force_noise_enable_) {
      Eigen::Vector3d noise = generateForceNoise() * 0.001; // 转为最大2mm扰动
      position_d_ += noise;
    }
    
    // 打印调试信息
    if (circular_counter_ % 1000 == 0) {
      ROS_INFO_STREAM("力控制: 当前力 " << actual_contact_force << "N, 目标力 " 
                    << target_force_ << "N, 调整量 " << position_adjustment * 1000 << "mm");
      ROS_INFO_STREAM("接触深度: " << current_contact_state.depth() * 1000 << "mm, 位置: ["
                    << position.transpose() << "], 目标位置: [" << position_d_.transpose() << "]");
      ROS_INFO_STREAM("接触状态: " << (current_contact_state.in_contact ? "已接触" : "未接触")
                    << ", 力矢量: [" << contact_force_vec.transpose() << "]");
    }
  } else {
    // 未接触时，使用微小力以确保有力反馈
    // 这种方式更符合实际物理系统，力传感器始终会有一些噪声和偏移
    actual_contact_force = 0.05;  // 设置一个最小的力值，仅用于显示
    contact_force_vec = Eigen::Vector3d(0, 0, actual_contact_force);
    
    // 打印调试信息
    if (circular_counter_ % 1000 == 0) {
      ROS_INFO_STREAM("未接触状态，距离软体表面: " 
                    << (position(2) - soft_block_position_(2)) * 1000 << "mm");
    }
  }
  
  // 更新力反馈消息
  contact_wrench_msg.wrench.force.x = contact_force_vec(0);
  contact_wrench_msg.wrench.force.y = contact_force_vec(1);
  contact_wrench_msg.wrench.force.z = contact_force_vec(2);
  
  // 混合实际测量力增强反馈
  if (measured_force.norm() > 0.2) {
    // 如果测量到明显的外部力，与模型力混合
    double alpha = 0.6; // 混合系数：模型力占60%，外部力占40%
    contact_wrench_msg.wrench.force.x = alpha * contact_force_vec(0) + (1-alpha) * measured_force(0);
    contact_wrench_msg.wrench.force.y = alpha * contact_force_vec(1) + (1-alpha) * measured_force(1);
    contact_wrench_msg.wrench.force.z = alpha * contact_force_vec(2) + (1-alpha) * measured_force(2);
    
    // 打印调试信息
    if (circular_counter_ % 1000 == 0) {
      ROS_INFO_STREAM("混合力反馈: 模型力=[" << contact_force_vec.transpose() 
                    << "], 测量力=[" << measured_force.transpose() << "]");
    }
  }
  
  // 设置力矩为零
  contact_wrench_msg.wrench.torque.x = 0.0;
  contact_wrench_msg.wrench.torque.y = 0.0;
  contact_wrench_msg.wrench.torque.z = 0.0;
  
  // 发布接触力信息
  contact_pub_.publish(contact_wrench_msg);
  
  // 检查接触状态切换
  // 在接近阶段，增加接触检测日志
  if (control_phase_ == APPROACH) {
    static int approach_counter = 0;
    if (approach_counter++ % 1000 == 0) {  // 约1s打印一次
      ROS_INFO_STREAM("接近阶段: 当前高度 " << position[2] << "m, 软块表面高度 " << soft_block_position_[2] 
                     << "m, 距离 " << (position[2] - soft_block_position_[2]) * 1000 << "mm");
      ROS_INFO_STREAM("接触状态: " << (current_contact_state.in_contact ? "已接触" : "未接触")
                     << ", 接触力: " << current_contact_state.contact_force << "N");
    }
    
    // 在接近阶段，更新机械臂期望位置（向下移动）
    // 计算每次更新的向下移动增量
    double approach_speed = 0.02; // 每秒下降2cm
    double delta_z = -approach_speed * period.toSec(); // 向下移动的距离增量
    
    // 更新期望位置
    position_d_ = position; // 保持当前XY位置
    position_d_[2] += delta_z; // 向下移动Z位置
    
    // 限制最低高度，避免过度碰撞
    if (position_d_[2] < soft_block_position_[2] - 0.01) { // 限制最大深入深度为1cm
      position_d_[2] = soft_block_position_[2] - 0.01;
    }
    
    // 接触检测 - 使用更宽松的条件
    bool is_contact_detected = false;
    
    // 主要条件: 软接触模型检测到接触
    if (current_contact_state.in_contact) {
      is_contact_detected = true;
      ROS_INFO_STREAM("接触检测: 软接触模型检测到接触!");
    }
    
    // 后备条件1: 位置已经接近或超过软块表面
    if ((soft_block_position_[2] - position[2]) > contact_params_.depth_threshold * 0.1) {
      is_contact_detected = true;
      ROS_INFO_STREAM("接触检测: 位置已接近软块表面! 距离: " << 
                     (soft_block_position_[2] - position[2]) * 1000 << " mm");
    }
    
    // 后备条件2: 检测到任何外部力
    if (measured_force.norm() > 0.5) {
      is_contact_detected = true;
      ROS_INFO_STREAM("接触检测: 检测到外部力! 力大小: " << measured_force.norm() << " N");
    }
    
    // 强制接触条件: 如果机械臂下降到接近软块3mm以内，强制判定为接触
    if ((position[2] - soft_block_position_[2]) < 0.003) {
      is_contact_detected = true;
      ROS_INFO_STREAM("接触检测: 强制判定为接触状态! 当前高度: " << 
                     position[2] << "m, 软块高度: " << soft_block_position_[2] << "m");
    }
    
    // 强制接触计时器: 如果在接近阶段持续太久，强制切换到接触阶段
    static ros::Time approach_start_time = time;
    if ((time - approach_start_time).toSec() > 10.0) { // 10秒后强制切换
      is_contact_detected = true;
      ROS_INFO_STREAM("接触检测: 接近阶段超时，强制进入接触阶段!");
    }
    
    if (is_contact_detected) {
      // 从接近阶段进入接触阶段
      control_phase_ = CONTACT;
      phase_start_time_ = time;
      contact_position_ = position;
      
      // 发布状态变化
      std_msgs::String phase_msg;
      phase_msg.data = "CONTACT";
      phase_pub_.publish(phase_msg);
      
      ROS_INFO_STREAM("===== 进入接触阶段! =====");
      ROS_INFO_STREAM("接触位置: [" << contact_position_.transpose() << "]");
      ROS_INFO_STREAM("接触力大小: " << current_contact_state.contact_force << " N");
      ROS_INFO_STREAM("接触深度: " << current_contact_state.depth() * 1000 << " mm");
      ROS_INFO_STREAM("测量力: [" << measured_force.transpose() << "] N");
    }
  }
  else if (control_phase_ == CONTACT) {
    // 检查是否满足切换条件
    double contact_duration = (time - phase_start_time_).toSec();
    
    // 接触阶段需要保持力控制
    // 设置期望位置，持续稍微向下移动直到达到目标力
    position_d_ = position;
    
    // 力控制 - 逐渐增加下压力直到达到目标力
    if (!current_contact_state.in_contact || current_contact_state.contact_force < target_force_) {
      // 如果未检测到接触或力不足，继续向下移动
      double force_diff = target_force_ - (current_contact_state.in_contact ? 
                                         current_contact_state.contact_force : 0.0);
      // 力差越大，移动越快
      double delta_z = -std::min(force_diff * 0.0001, 0.001); // 限制最大增量为1mm
      position_d_[2] += delta_z;
      
      // 限制最低位置，避免过度碰撞
      if (position_d_[2] < soft_block_position_[2] - 0.02) { // 限制最大深入深度为2cm
        position_d_[2] = soft_block_position_[2] - 0.02;
      }
    }
    
    // 添加更详细的调试信息
    static int debug_counter = 0;
    if (debug_counter++ % 1000 == 0) {  // 约1s打印一次
      ROS_INFO_STREAM("当前处于接触阶段: 已持续 " << contact_duration << " 秒, 需持续 " 
                     << phase_duration_ << " 秒后切换到圆周运动");
      ROS_INFO_STREAM("接触状态: " << (current_contact_state.in_contact ? "已接触" : "未接触")
                     << ", 接触力: " << current_contact_state.contact_force << " N"
                     << ", 目标力: " << target_force_ << " N");
      ROS_INFO_STREAM("当前位置: [" << position.transpose() << "], 期望位置: [" 
                     << position_d_.transpose() << "]");
    }
    
    // 当接触持续时间超过设定阈值时切换到圆周运动
    if (contact_duration > phase_duration_) {
      // 从接触阶段进入圆周运动阶段
      control_phase_ = TRAJECTORY;
      phase_start_time_ = time;
      
      // 设置圆周运动中心为接触点
      circle_center_ = position;
      
      // 重置时间，以便圆周运动从0开始
      elapsed_time_ = 0.0;
      
      // 发布状态变化
      std_msgs::String phase_msg;
      phase_msg.data = "TRAJECTORY";
      phase_pub_.publish(phase_msg);
      
      ROS_INFO_STREAM("===== 进入圆周运动阶段! =====");
      ROS_INFO_STREAM("圆心位置: [" << circle_center_.transpose() << "]");
      ROS_INFO_STREAM("圆周半径: " << circle_radius_ << " m");
      ROS_INFO_STREAM("圆周频率: " << circle_frequency_ << " Hz");
    }
  }
  else if (control_phase_ == TRAJECTORY) {
    // 轨迹运动阶段
    if (circular_counter_ % 1000 == 0) {
      ROS_INFO_STREAM("轨迹运动阶段: 已持续 " << elapsed_time_ << " 秒");
      switch (trajectory_type_) {
        case CIRCULAR:
          ROS_INFO_STREAM("当前轨迹: 圆形, 半径: " << circle_radius_ << "m");
          break;
        case RECTANGULAR:
          ROS_INFO_STREAM("当前轨迹: 矩形, 长宽: " << rect_length_ << "x" << rect_width_ << "m");
          break;
        case FIGURE_EIGHT:
          ROS_INFO_STREAM("当前轨迹: 八字形, 尺寸: " << eight_radius_x_ * 2 << "x" << eight_radius_y_ * 2 << "m");
          break;
        case LINE:
          ROS_INFO_STREAM("当前轨迹: 直线往返, 长度: " << line_length_ << "m");
          break;
        case CUSTOM:
          ROS_INFO_STREAM("当前轨迹: 自定义轨迹, 点数: " << custom_trajectory_points_.size());
          break;
      }
      ROS_INFO_STREAM("机械臂位置: [" << position.transpose() << "]");
      
      // 轨迹路径现在在update函数顶部定期更新，此处不再需要
      // publishTrajectoryPath();
    }
    
    // 生成当前轨迹点 - 添加前馈控制
    // 计算期望位置、速度和加速度
    
    // 当前时间对应的期望位置
    Eigen::Vector3d trajectory_position = generateTrajectory(elapsed_time_);
    
    // 计算轨迹目标速度和加速度 - 使用更高精度的中心差分
    double dt = 0.001; // 1毫秒的时间步长
    Eigen::Vector3d trajectory_position_prev = generateTrajectory(elapsed_time_ - dt);
    Eigen::Vector3d trajectory_position_next = generateTrajectory(elapsed_time_ + dt);
    
    // 更精确的速度计算 - 使用中心差分
    position_d_dot = (trajectory_position_next - trajectory_position_prev) / (2.0 * dt);
    
    // 更精确的加速度计算 - 使用中心差分
    position_d_ddot = (trajectory_position_next - 2.0 * trajectory_position + trajectory_position_prev) / (dt * dt);
    
    // 更新期望位置的XY坐标（Z坐标由力控制决定）
    position_d_[0] = trajectory_position[0];
    position_d_[1] = trajectory_position[1];
    
    // 应用前馈控制补偿物理延迟 - 基于速度和加速度的预测修正
    // 这将使控制器更快地响应轨迹变化
    double feed_forward_time = 0.01; // 10ms的前馈预测
    position_d_[0] += position_d_dot(0) * feed_forward_time + 0.5 * position_d_ddot(0) * feed_forward_time * feed_forward_time;
    position_d_[1] += position_d_dot(1) * feed_forward_time + 0.5 * position_d_ddot(1) * feed_forward_time * feed_forward_time;
    
    // 力控制 - 保持恒定接触力
    // 通过软接触模型计算接触力并调整轨迹
    if (!current_contact_state.in_contact) {
      // 如果失去接触，轻微向下移动以重新建立接触
      position_d_[2] -= 0.0005; // 每次向下移动0.5mm
      
      // 限制最低高度，避免过度碰撞
      if (position_d_[2] < soft_block_position_[2] - 0.02) { // 限制最大深入深度为2cm
        position_d_[2] = soft_block_position_[2] - 0.02;
      }
      
      if (circular_counter_ % 1000 == 0) {
        ROS_WARN_STREAM("轨迹运动中失去接触，尝试向下移动重新建立接触");
      }
    } else {
      // 已检测到接触，调整力
      // 计算力误差
      double force_error = target_force_ - current_contact_state.contact_force;
      
      // 积分项（限制积分饱和）
      force_error_integral_ += force_error * period.toSec();
      force_error_integral_ = std::max(std::min(force_error_integral_, 5.0), -5.0);
      
      // 微分项
      double force_error_derivative = (force_error - prev_force_error_) / period.toSec();
      prev_force_error_ = force_error;
      
      // 力控制PID输出（负号是因为位置增加会减小力）
      double position_adjustment = -(force_p_gain_ * force_error + 
                                   force_i_gain_ * force_error_integral_ + 
                                   force_d_gain_ * force_error_derivative);
      
      // 限制位置调整范围
      position_adjustment = std::max(std::min(position_adjustment, 0.001), -0.001);  // 限制在±1mm
      
      // 仅调整z方向位置（沿法线方向）
      position_d_[2] += position_adjustment;
      
      // 若启用力噪音，给position_d_叠加三维扰动
      if (force_noise_enable_) {
        Eigen::Vector3d noise = generateForceNoise() * 0.001; // 转为最大2mm扰动
        position_d_ += noise;
      }
      
      if (circular_counter_ % 1000 == 0) {
        ROS_INFO_STREAM("力控制: 当前力 " << current_contact_state.contact_force << "N, 目标力 " 
                      << target_force_ << "N, 调整量 " << position_adjustment * 1000 << "mm");
      }
    }
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
                    (nullspace_stiffness_ * (initial_q_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
    
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
      ROS_INFO_STREAM("=== 运动控制状态 ===");
      ROS_INFO_STREAM("控制阶段: " << (control_phase_ == APPROACH ? "接近" : 
                                    (control_phase_ == CONTACT ? "接触" : "轨迹运动")));
      ROS_INFO_STREAM("当前位置: [" << position.transpose() << "]");
      ROS_INFO_STREAM("期望位置: [" << position_d_.transpose() << "]");
      ROS_INFO_STREAM("位置误差: [" << error.head(3).transpose() << "]");
      ROS_INFO_STREAM("位置误差范数: " << error.head(3).norm());
      
      ROS_INFO_STREAM("姿态误差: [" << error.tail(3).transpose() << "]");
      ROS_INFO_STREAM("姿态误差范数: " << error.tail(3).norm());
      
      if (current_contact_state.in_contact) {
        ROS_INFO_STREAM("接触状态: 已接触");
        ROS_INFO_STREAM("接触位置: [" << current_contact_state.position().transpose() << "]");
        Eigen::Vector3d force_vec = current_contact_state.force();
        ROS_INFO_STREAM("接触力: [" << force_vec.transpose() << "]");
        ROS_INFO_STREAM("接触力大小: " << current_contact_state.contact_force << " N");
        ROS_INFO_STREAM("接触深度: " << current_contact_state.depth() * 1000 << " mm");
        ROS_INFO_STREAM("目标力: " << target_force_ << " N");
        ROS_INFO_STREAM("力误差: " << target_force_ - current_contact_state.contact_force << " N");
      } else {
        ROS_INFO_STREAM("接触状态: 未接触");
        ROS_INFO_STREAM("到软体块距离: " << (position(2) - soft_block_position_(2)) << " m");
      }
      
      if (control_phase_ == TRAJECTORY) {
        double angle = 2.0 * M_PI * circle_frequency_ * elapsed_time_;
        ROS_INFO_STREAM("圆周角度: " << fmod(angle, 2.0 * M_PI) << " rad");
      }
      
      ROS_INFO_STREAM("经过时间: " << elapsed_time_ << " s");
      ROS_INFO_STREAM("===============================");
    }
  } catch (const std::exception& ex) {
    // 捕获并记录任何控制过程中的异常
    ROS_ERROR_STREAM("控制过程中发生异常: " << ex.what());
  }

  // 在接触状态或圆周运动阶段记录数据
  // 将此代码放在每个控制周期的最后，确保获取到最新数据
  if (current_contact_state.in_contact) {
    contact_force_vec = current_contact_state.force();
  } else {
    contact_force_vec = Eigen::Vector3d(0, 0, 0.05);
  }
  
  // 记录数据到日志文件
  logData(time, position, contact_force_vec, static_cast<int>(control_phase_));
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

// 实现发布轨迹路径的函数
void CircleController::publishTrajectoryPath() {
  // 创建路径消息
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "world";
  
  // 历史和预测点数量
  int num_history_points = 100;   // 历史轨迹点数量 - 增加到100
  int num_future_points = 200;   // 未来轨迹点数量 - 增加到200
  
  // 时间步长 - 以毫秒为单位
  double dt = 0.02; // 20ms的间隔
  
  // 当前时间
  double current_time = elapsed_time_;
  
  // 将路径置于软体平台表面高度
  double path_z_height = soft_block_position_(2);
  
  // 生成过去的历史轨迹点 (当前时间之前)
  for (int i = num_history_points; i > 0; i--) {
    // 计算过去时间点
    double past_time = current_time - i * dt;
    
    // 确保时间不为负
    if (past_time < 0) {
      continue;
    }
    
    // 根据当前轨迹类型生成历史位置
    Eigen::Vector3d point;
    switch (trajectory_type_) {
      case CIRCULAR:
        point = generateCircularTrajectory(past_time * speed_factor_);
        break;
      case RECTANGULAR:
        point = generateRectangularTrajectory(past_time * speed_factor_);
        break;
      case FIGURE_EIGHT:
        point = generateFigureEightTrajectory(past_time * speed_factor_);
        break;
      case LINE:
        point = generateLineTrajectory(past_time * speed_factor_);
        break;
      case CUSTOM:
        point = generateCustomTrajectory(past_time * speed_factor_);
        break;
      default:
        point = generateCircularTrajectory(past_time * speed_factor_);
        break;
    }
    
    // 设置轨迹Z坐标为软体平台表面高度
    point(2) = path_z_height;
    
    // 创建位姿点
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = point(0);
    pose.pose.position.y = point(1);
    pose.pose.position.z = point(2);
    
    // 使用当前方向
    pose.pose.orientation.x = orientation_d_.x();
    pose.pose.orientation.y = orientation_d_.y();
    pose.pose.orientation.z = orientation_d_.z();
    pose.pose.orientation.w = orientation_d_.w();
    
    path_msg.poses.push_back(pose);
  }
  
  // 添加当前时间点
  {
    Eigen::Vector3d current_point;
    switch (trajectory_type_) {
      case CIRCULAR:
        current_point = generateCircularTrajectory(current_time * speed_factor_);
        break;
      case RECTANGULAR:
        current_point = generateRectangularTrajectory(current_time * speed_factor_);
        break;
      case FIGURE_EIGHT:
        current_point = generateFigureEightTrajectory(current_time * speed_factor_);
        break;
      case LINE:
        current_point = generateLineTrajectory(current_time * speed_factor_);
        break;
      case CUSTOM:
        current_point = generateCustomTrajectory(current_time * speed_factor_);
        break;
      default:
        current_point = generateCircularTrajectory(current_time * speed_factor_);
        break;
    }
    
    current_point(2) = path_z_height;
    
    geometry_msgs::PoseStamped current_pose;
    current_pose.header = path_msg.header;
    current_pose.pose.position.x = current_point(0);
    current_pose.pose.position.y = current_point(1);
    current_pose.pose.position.z = current_point(2);
    current_pose.pose.orientation.x = orientation_d_.x();
    current_pose.pose.orientation.y = orientation_d_.y();
    current_pose.pose.orientation.z = orientation_d_.z();
    current_pose.pose.orientation.w = orientation_d_.w();
    
    path_msg.poses.push_back(current_pose);
  }
  
  // 生成未来轨迹点 (当前时间之后)
  for (int i = 1; i <= num_future_points; i++) {
    // 计算未来时间点
    double future_time = current_time + i * dt;
    
    // 根据当前轨迹类型生成位置
    Eigen::Vector3d point;
    switch (trajectory_type_) {
      case CIRCULAR:
        point = generateCircularTrajectory(future_time * speed_factor_);
        break;
      case RECTANGULAR:
        point = generateRectangularTrajectory(future_time * speed_factor_);
        break;
      case FIGURE_EIGHT:
        point = generateFigureEightTrajectory(future_time * speed_factor_);
        break;
      case LINE:
        point = generateLineTrajectory(future_time * speed_factor_);
        break;
      case CUSTOM:
        point = generateCustomTrajectory(future_time * speed_factor_);
        break;
      default:
        point = generateCircularTrajectory(future_time * speed_factor_);
        break;
    }
    
    // 设置轨迹Z坐标为软体平台表面高度
    point(2) = path_z_height;
    
    // 创建位姿点
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = point(0);
    pose.pose.position.y = point(1);
    pose.pose.position.z = point(2);
    
    // 使用当前方向
    pose.pose.orientation.x = orientation_d_.x();
    pose.pose.orientation.y = orientation_d_.y();
    pose.pose.orientation.z = orientation_d_.z();
    pose.pose.orientation.w = orientation_d_.w();
    
    path_msg.poses.push_back(pose);
  }
  
  // 发布路径消息
  path_pub_.publish(path_msg);
}

// 力轨迹生成函数
// 支持恒定、正弦、阶跃三种类型
// target_force_为恒定力基准
double CircleController::generateForceProfile(double time) {
  double force = 0.0;
  switch (force_profile_type_) {
    case FORCE_CONSTANT:
      force = base_force_;
      break;
    case FORCE_SINE:
      force = base_force_ + force_sine_amplitude_ * std::sin(2 * M_PI * force_sine_frequency_ * time);
      break;
    case FORCE_STEP:
      force = (time < force_step_time_) ? base_force_ : force_step_value_;
      break;
    default:
      force = base_force_;
  }
  if (force_noise_enable_) {
    force += generateForceNoise().norm();
  }
  return force;
}

Eigen::Vector3d CircleController::generateForceNoise() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(force_noise_min_, force_noise_max_);
  return Eigen::Vector3d(dis(gen), dis(gen), dis(gen));
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CircleController,
                       controller_interface::ControllerBase)
