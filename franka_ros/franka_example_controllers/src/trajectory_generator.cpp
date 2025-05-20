/**
 * @file trajectory_generator.cpp
 * @brief 轨迹生成器，用于生成不同类型的运动轨迹
 *
 * @架构：
 * - TrajectoryGenerator类：生成多种轨迹类型，包括圆形、矩形、8字形等
 *
 * @数据流：
 * 输入：轨迹参数（类型、半径、频率等）-> 
 * 处理：基于时间生成轨迹点坐标 -> 
 * 输出：三维轨迹点和可视化路径
 *
 * @概述：
 * 1. 支持多种轨迹类型：圆形、矩形、8字形、直线、自定义轨迹
 * 2. 提供平滑启动功能，避免轨迹起始的突变
 * 3. 支持从CSV文件加载自定义轨迹
 * 4. 发布轨迹路径用于RViz可视化
 * 5. 支持速度因子动态调整轨迹运行速度
 */
// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/trajectory_generator.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace franka_example_controllers {

TrajectoryGenerator::TrajectoryGenerator()
    : trajectory_type_(CIRCULAR),
      trajectory_type_str_("circular"),
      circle_frequency_(0.3),
      circle_radius_(0.1),
      circle_plane_("yz"),
      speed_factor_(1.0),
      circle_center_(Eigen::Vector3d::Zero()),
      circle_x_axis_(Eigen::Vector3d::UnitX()),
      circle_y_axis_(Eigen::Vector3d::UnitY()),
      rect_length_(0.2),
      rect_width_(0.1),
      eight_radius_x_(0.1),
      eight_radius_y_(0.05),
      line_length_(0.2),
      custom_trajectory_file_(""),
      contact_required_(true),             // 新增：默认需要保持接触
      soft_block_surface_z_(0.0),         // 新增：软块表面高度
      target_depth_(0.001),               // 新增：目标接触深度
      preserve_height_(false) {            // 新增：是否保持当前高度
}

bool TrajectoryGenerator::init(ros::NodeHandle& node_handle, ros::Publisher& path_publisher) {
  path_pub_ = path_publisher;

  // 读取轨迹类型参数
  std::string trajectory_type_str;
  if (node_handle.getParam("trajectory_type", trajectory_type_str)) {
    trajectory_type_str_ = trajectory_type_str;
    if (trajectory_type_str == "circular") {
      trajectory_type_ = CIRCULAR;
      ROS_INFO_STREAM("TrajectoryGenerator: Using circular trajectory");
    } else if (trajectory_type_str == "rectangular") {
      trajectory_type_ = RECTANGULAR;
      ROS_INFO_STREAM("TrajectoryGenerator: Using rectangular trajectory");
    } else if (trajectory_type_str == "figure_eight") {
      trajectory_type_ = FIGURE_EIGHT;
      ROS_INFO_STREAM("TrajectoryGenerator: Using figure eight trajectory");
    } else if (trajectory_type_str == "line") {
      trajectory_type_ = LINE;
      ROS_INFO_STREAM("TrajectoryGenerator: Using linear trajectory");
    } else if (trajectory_type_str == "custom") {
      trajectory_type_ = CUSTOM;
      ROS_INFO_STREAM("TrajectoryGenerator: Using custom trajectory");
    } else {
      ROS_WARN_STREAM("TrajectoryGenerator: Unknown trajectory type: " << trajectory_type_str 
                     << ". Using default circular trajectory.");
      trajectory_type_ = CIRCULAR;
      trajectory_type_str_ = "circular";
    }
  } else {
    ROS_INFO_STREAM("TrajectoryGenerator: No trajectory type specified. Using default circular trajectory.");
    trajectory_type_ = CIRCULAR;
    trajectory_type_str_ = "circular";
  }

  // 从参数服务器读取圆周运动参数
  if (!node_handle.getParam("circle_radius", circle_radius_)) {
    ROS_INFO_STREAM("TrajectoryGenerator: Using default circle radius: " << circle_radius_);
  }
  if (!node_handle.getParam("circle_frequency", circle_frequency_)) {
    ROS_INFO_STREAM("TrajectoryGenerator: Using default circle frequency: " << circle_frequency_);
  }

  // 从参数服务器读取圆周运动平面参数
  if (!node_handle.getParam("circle_plane", circle_plane_)) {
    circle_plane_ = "yz";  // 默认使用YZ平面，垂直于地面
    ROS_INFO_STREAM("TrajectoryGenerator: Using default circle plane: " << circle_plane_);
  }

  // 读取矩形轨迹参数
  node_handle.param<double>("rect_length", rect_length_, 0.2);
  node_handle.param<double>("rect_width", rect_width_, 0.1);
  ROS_INFO_STREAM("TrajectoryGenerator: Rectangle dimensions: " << rect_length_ << "x" << rect_width_ << "m");
  
  // 读取八字形轨迹参数
  node_handle.param<double>("eight_radius_x", eight_radius_x_, 0.1);
  node_handle.param<double>("eight_radius_y", eight_radius_y_, 0.05);
  ROS_INFO_STREAM("TrajectoryGenerator: Figure eight dimensions: " << eight_radius_x_ * 2 << "x" << eight_radius_y_ * 2 << "m");
  
  // 读取直线轨迹参数
  node_handle.param<double>("line_length", line_length_, 0.2);
  ROS_INFO_STREAM("TrajectoryGenerator: Line length: " << line_length_ << "m");
  
  // 读取自定义轨迹文件
  std::string custom_trajectory_file;
  if (node_handle.getParam("custom_trajectory_file", custom_trajectory_file)) {
    custom_trajectory_file_ = custom_trajectory_file;
    ROS_INFO_STREAM("TrajectoryGenerator: Custom trajectory file: " << custom_trajectory_file_);
    if (trajectory_type_ == CUSTOM) {
      if (!loadCustomTrajectory(custom_trajectory_file_)) {
        ROS_WARN("TrajectoryGenerator: Failed to load custom trajectory. Falling back to circular trajectory.");
        trajectory_type_ = CIRCULAR;
        trajectory_type_str_ = "circular";
      }
    }
  }

  // 读取速度因子参数
  if (!node_handle.getParam("speed_factor", speed_factor_)) {
    ROS_INFO_STREAM("TrajectoryGenerator: Using default speed factor: " << speed_factor_);
  }

  return true;
}

Eigen::Vector3d TrajectoryGenerator::generateTrajectory(double time, bool in_contact, double current_depth) {
  // 应用速度因子来调整轨迹速度
  double adjusted_time = time * speed_factor_;
  
  // 增加预测时间以减少跟踪延迟
  double prediction_time = 0.05;
  double predicted_time = adjusted_time + prediction_time;
  
  // 根据轨迹类型生成目标位置
  Eigen::Vector3d trajectory_position;
  switch (trajectory_type_) {
    case CIRCULAR:
      trajectory_position = generateCircularTrajectory(predicted_time);
      break;
    case RECTANGULAR:
      trajectory_position = generateRectangularTrajectory(predicted_time);
      break;
    case FIGURE_EIGHT:
      trajectory_position = generateFigureEightTrajectory(predicted_time);
      break;
    case LINE:
      trajectory_position = generateLineTrajectory(predicted_time);
      break;
    case CUSTOM:
      trajectory_position = generateCustomTrajectory(predicted_time);
      break;
    default:
      trajectory_position = generateCircularTrajectory(predicted_time);
  }
  
  // 如果在接触状态且需要保持接触深度，调整z高度
  if (in_contact && contact_required_) {
    // 将z坐标设置为保持目标深度的高度
    trajectory_position(2) = soft_block_surface_z_ - target_depth_;
    
    // 应用高度修正以确保稳定深度
    double height_correction = 0.0;
    
    // 如果当前深度与目标深度有差异，进行小调整
    if (fabs(current_depth - target_depth_) > 0.0002) {  // 0.2mm的差异阈值
      // 减小校正率，使调整更平滑
      height_correction = (current_depth - target_depth_) * 0.3;  // 30%的校正率(原为50%)
      
      // 限制单次高度修正量
      double max_correction = 0.0002;  // 降低最大修正量到0.2mm，使控制更平稳
      height_correction = std::max(std::min(height_correction, max_correction), -max_correction);
    }
    
    // 应用高度修正
    trajectory_position(2) -= height_correction;
    
    // 添加安全检查防止极端情况
    if (trajectory_position(2) < soft_block_surface_z_ - 0.004) {
      // 防止下压过深 - 最大深度4mm(软块厚度5mm)
      trajectory_position(2) = soft_block_surface_z_ - 0.004;
    } else if (trajectory_position(2) > soft_block_surface_z_ + 0.003) {
      // 防止过高 - 最高不超过表面3mm
      trajectory_position(2) = soft_block_surface_z_ + 0.003;
    }
    
    // 记录轨迹生成的高度信息
    static int debug_counter = 0;
    if (debug_counter++ % 200 == 0) { // 每200次迭代输出一次
      ROS_DEBUG_STREAM("轨迹高度: 软块表面=" << soft_block_surface_z_ 
          << ", 当前深度=" << current_depth * 1000.0 << "mm"
          << ", 目标深度=" << target_depth_ * 1000.0 << "mm"
          << ", 轨迹位置z=" << trajectory_position(2)
          << ", 修正量=" << height_correction * 1000.0 << "mm");
    }
  } else if (preserve_height_) {
    // 保持当前高度，不修改z坐标
    trajectory_position(2) = circle_center_(2);
  }
  
  return trajectory_position;
}

Eigen::Vector3d TrajectoryGenerator::generateCircularTrajectory(double time) {
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

Eigen::Vector3d TrajectoryGenerator::generateRectangularTrajectory(double time) {
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

Eigen::Vector3d TrajectoryGenerator::generateFigureEightTrajectory(double time) {
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

Eigen::Vector3d TrajectoryGenerator::generateLineTrajectory(double time) {
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

Eigen::Vector3d TrajectoryGenerator::generateCustomTrajectory(double time) {
  // 检查是否有轨迹点
  if (custom_trajectory_points_.empty()) {
    ROS_WARN_ONCE("Custom trajectory is empty, returning center position");
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

bool TrajectoryGenerator::loadCustomTrajectory(const std::string& filename) {
  try {
    std::ifstream file(filename);
    if (!file.is_open()) {
      ROS_ERROR_STREAM("Cannot open trajectory file: " << filename);
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
      ROS_ERROR("Trajectory file does not contain valid data points");
      return false;
    }
    
    ROS_INFO_STREAM("Successfully loaded custom trajectory with " << custom_trajectory_points_.size() << " points");
    ROS_INFO_STREAM("Trajectory duration: " << custom_trajectory_points_.back().time << " seconds");
    return true;
  } catch (const std::exception& ex) {
    ROS_ERROR_STREAM("Error loading custom trajectory: " << ex.what());
    return false;
  }
}

void TrajectoryGenerator::publishTrajectoryPath(double current_time, double path_z_height) {
  // 创建路径消息
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "world";
  
  // 历史和预测点数量
  int num_history_points = 100;   // 历史轨迹点数量
  int num_future_points = 200;   // 未来轨迹点数量
  
  // 时间步长 - 以毫秒为单位
  double dt = 0.02; // 20ms的间隔
  
  // 将路径置于指定高度
  double path_z = path_z_height;
  
  // 生成过去的历史轨迹点 (当前时间之前)
  for (int i = num_history_points; i > 0; i--) {
    // 计算过去时间点
    double past_time = current_time - i * dt;
    
    // 确保时间不为负
    if (past_time < 0) {
      continue;
    }
    
    // 根据当前轨迹类型生成历史位置
    Eigen::Vector3d point = generateTrajectory(past_time, false, 0.0);
    
    // 设置轨迹Z坐标为指定高度
    point(2) = path_z;
    
    // 创建位姿点
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = point(0);
    pose.pose.position.y = point(1);
    pose.pose.position.z = point(2);
    
    // 使用默认方向
    pose.pose.orientation.w = 1.0;
    
    path_msg.poses.push_back(pose);
  }
  
  // 添加当前时间点
  {
    Eigen::Vector3d current_point = generateTrajectory(current_time, false, 0.0);
    current_point(2) = path_z;
    
    geometry_msgs::PoseStamped current_pose;
    current_pose.header = path_msg.header;
    current_pose.pose.position.x = current_point(0);
    current_pose.pose.position.y = current_point(1);
    current_pose.pose.position.z = current_point(2);
    current_pose.pose.orientation.w = 1.0;
    
    path_msg.poses.push_back(current_pose);
  }
  
  // 生成未来轨迹点 (当前时间之后)
  for (int i = 1; i <= num_future_points; i++) {
    // 计算未来时间点
    double future_time = current_time + i * dt;
    
    // 根据当前轨迹类型生成位置
    Eigen::Vector3d point = generateTrajectory(future_time, false, 0.0);
    
    // 设置轨迹Z坐标为指定高度
    point(2) = path_z;
    
    // 创建位姿点
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = point(0);
    pose.pose.position.y = point(1);
    pose.pose.position.z = point(2);
    
    // 使用默认方向
    pose.pose.orientation.w = 1.0;
    
    path_msg.poses.push_back(pose);
  }
  
  // 发布路径消息
  path_pub_.publish(path_msg);
}

void TrajectoryGenerator::setCenter(const Eigen::Vector3d& center) {
  circle_center_ = center;
  ROS_INFO_STREAM("TrajectoryGenerator: Center set to: [" << circle_center_.transpose() << "]");
}

void TrajectoryGenerator::setAxis(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis) {
  circle_x_axis_ = x_axis;
  circle_y_axis_ = y_axis;
  ROS_INFO_STREAM("TrajectoryGenerator: X-axis set to: [" << circle_x_axis_.transpose() << "]");
  ROS_INFO_STREAM("TrajectoryGenerator: Y-axis set to: [" << circle_y_axis_.transpose() << "]");
}

TrajectoryType TrajectoryGenerator::getTrajectoryType() const {
  return trajectory_type_;
}

std::string TrajectoryGenerator::getTrajectoryTypeStr() const {
  return trajectory_type_str_;
}

void TrajectoryGenerator::setSoftBlockParameters(double surface_z, double target_depth) {
  soft_block_surface_z_ = surface_z;
  target_depth_ = target_depth;
  ROS_INFO("TrajectoryGenerator: Soft block surface set to %.3f m, target depth: %.3f mm",
           soft_block_surface_z_, target_depth_ * 1000.0);
}

void TrajectoryGenerator::setPreserveHeight(bool preserve) {
  preserve_height_ = preserve;
  ROS_INFO("TrajectoryGenerator: %s current height in trajectory", 
           preserve ? "Preserving" : "Not preserving");
}

void TrajectoryGenerator::setContactRequired(bool required) {
  contact_required_ = required;
  ROS_INFO("TrajectoryGenerator: Contact %s for trajectory execution", 
           required ? "required" : "not required");
}

} // namespace franka_example_controllers 