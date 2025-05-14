// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <franka_msgs/FrankaState.h>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <deque>  // 添加队列用于力变化检测

// 导入ADMM软接触模型
#include <franka_example_controllers/soft_contact_model.h>

namespace franka_example_controllers {

/**
 * @brief 软接触力估计器节点，用于对比Gazebo仿真力与理论估计力
 */
class SoftContactForceEstimator {
public:
  /**
   * @brief 构造一个软接触力估计器
   * @param nh ROS节点句柄
   */
  SoftContactForceEstimator(ros::NodeHandle& nh) : nh_(nh) {
    // 从参数服务器获取软接触参数
    ContactParams params;
    nh_.param<double>("contact_model/young_modulus", params.young_modulus, 1000.0);
    nh_.param<double>("contact_model/poisson_ratio", params.poisson_ratio, 0.45);
    nh_.param<double>("contact_model/friction_coef", params.friction_coef, 0.3);
    nh_.param<double>("contact_model/contact_radius", params.contact_radius, 0.01);
    nh_.param<double>("contact_model/path_radius", params.path_radius, 0.1);
    nh_.param<double>("contact_model/damping", params.damping, 50.0);
    
    // 设置接触检测的深度阈值，改为更合理的值
    nh_.param<double>("contact_model/depth_threshold", params.depth_threshold, 0.0005);
    
    // 配置力突变检测参数 - 降低阈值以提高检测灵敏度
    nh_.param<double>("force_change_threshold", force_change_threshold_, 5.0);
    nh_.param<int>("force_window_size", force_window_size_, 5);
    nh_.param<bool>("use_force_change_detection", use_force_change_detection_, true);
    
    // 配置绝对力值检测参数
    nh_.param<double>("absolute_force_threshold", absolute_force_threshold_, 8.0);
    nh_.param<bool>("use_absolute_force_detection", use_absolute_force_detection_, true);
    
    // 配置调试模式
    nh_.param<bool>("debug_mode", debug_mode_, true);
    
    // 设置软体块表面高度
    nh_.param<double>("soft_block_surface_z", soft_block_z_, 0.575);
    
    // 创建软接触模型，修正为传入NodeHandle
    contact_model_ = std::make_unique<SoftContactModel>(nh_, params);
    
    // 初始化发布器
    estimated_force_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/estimated_external_force", 1);
    gazebo_force_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/gazebo_external_force", 1);
    estimated_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/estimated_contact_pose", 1);
    
    // 订阅机械臂状态
    franka_state_sub_ = nh_.subscribe("/franka_state_controller/franka_states", 1, 
                                       &SoftContactForceEstimator::frankaStateCallback, this);
    
    // 订阅关节状态，用于计算雅可比矩阵
    joint_state_sub_ = nh_.subscribe("/joint_states", 1, 
                                     &SoftContactForceEstimator::jointStateCallback, this);
    
    // 创建调试计时器，每秒打印一次状态
    debug_timer_ = nh_.createTimer(ros::Duration(1.0), 
                                   &SoftContactForceEstimator::debugTimerCallback, this);
    
    // 初始化力历史
    force_history_.clear();
    max_force_change_ = 0.0;
    
    // 初始化接触变量
    is_in_contact_ = false;
    is_contact_by_depth_ = false; // 保留变量但不再使用
    is_contact_by_force_ = false;
    is_contact_by_absolute_force_ = false;
    consecutive_contact_frames_ = 0;
    contact_position_initialized_ = false;
    first_contact_z_ = 0.0;
    
    // 初始化日志
    start_time_ = ros::Time::now();
    initLogFile();
    
    ROS_INFO("软接触力估计器启动成功");
    ROS_INFO("软块表面高度: %.4f m", soft_block_z_);
    ROS_INFO("接触检测阈值: %.6f m", params.depth_threshold);
    ROS_INFO("赫兹接触模型参数 - 杨氏模量: %.1f Pa, 半径: %.3f m", 
             params.young_modulus, params.contact_radius);
    ROS_INFO("力变化检测阈值: %.3f N, 窗口大小: %d", force_change_threshold_, force_window_size_);
    ROS_INFO("绝对力检测阈值: %.3f N", absolute_force_threshold_);
    ROS_INFO("注意: 仅使用力检测进行接触判断");
    ROS_INFO("接触检测逻辑: 首次接触需同时满足力变化和绝对力条件，接触后仅通过绝对力判断");
  }
  
  ~SoftContactForceEstimator() {
    closeLogFile();
  }

private:
  ros::NodeHandle& nh_;
  
  // 订阅和发布
  ros::Subscriber franka_state_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher estimated_force_pub_;
  ros::Publisher force_comparison_pub_;
  ros::Publisher gazebo_force_pub_;
  ros::Publisher estimated_pose_pub_;
  ros::Timer debug_timer_;
  
  // 软接触模型
  std::unique_ptr<SoftContactModel> contact_model_;
  
  // 状态记录
  Eigen::Vector3d current_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond current_orientation_ = Eigen::Quaterniond::Identity();
  double gazebo_force_z_ = 0.0;  // 只记录z轴力
  double estimated_force_z_ = 0.0;  // 只记录z轴力
  
  // 力变化检测相关
  std::deque<double> force_history_;  // 力历史队列
  double force_change_threshold_ = 5.0;  // 力变化阈值 (N)
  int force_window_size_ = 5;  // 力历史窗口大小
  bool use_force_change_detection_ = true;  // 是否启用力突变检测
  double max_force_change_ = 0.0;  // 最大力变化量
  
  // 绝对力值检测相关
  double absolute_force_threshold_ = 8.0;  // 绝对力阈值 (N)
  bool use_absolute_force_detection_ = true;  // 是否启用绝对力检测
  bool is_contact_by_absolute_force_ = false;  // 基于绝对力的接触状态
  
  // 软体块属性
  double soft_block_z_;
  double contact_threshold_ = 0.0005; // 接触检测阈值，增大为0.5mm
  bool debug_mode_ = true;
  int contact_detected_count_ = 0;
  int total_updates_ = 0;
  int consecutive_contact_frames_ = 0; // 连续接触帧计数
  
  // 首次接触位置记录
  double first_contact_z_ = 0.0;          // 首次接触时的Z坐标
  bool contact_position_initialized_ = false; // 是否已记录首次接触位置
  
  // 接触状态
  bool is_contact_by_depth_ = false;  // 基于深度的接触状态
  bool is_contact_by_force_ = false;  // 基于力的接触状态
  bool is_in_contact_ = false;        // 最终接触状态
  
  // 日志文件
  std::ofstream log_file_;
  std::string log_file_path_;
  bool log_initialized_ = false;
  ros::Time start_time_;
  int log_counter_ = 0;
  
  /**
   * @brief 初始化日志文件
   */
  void initLogFile() {
    if (log_initialized_) return;
    
    // 创建log文件夹
    std::string package_path = ros::package::getPath("franka_gazebo");
    std::string log_dir = package_path + "/logs";
    std::string cmd = "mkdir -p " + log_dir;
    system(cmd.c_str());
    
    // 获取当前时间作为文件名
    std::time_t t = std::time(nullptr);
    char time_str[20];
    std::strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", std::localtime(&t));
    
    // 创建CSV文件
    std::string filename = log_dir + "/force_estimation_" + std::string(time_str) + ".csv";
    log_file_.open(filename);
    
    if (log_file_.is_open()) {
      log_file_ << "# 实验开始时间: 0 秒" << std::endl;
      log_file_ << "# 本地时间: " << time_str << std::endl;
      log_file_ << "# 软块表面高度: " << std::fixed << std::setprecision(6) << soft_block_z_ << " m" << std::endl;
      log_file_ << "# 接触阈值: " << std::fixed << std::setprecision(6) << contact_threshold_ << " m" << std::endl;
      log_file_ << "# 力变化检测阈值: " << std::fixed << std::setprecision(6) << force_change_threshold_ << " N" << std::endl;
      log_file_ << "# 绝对力检测阈值: " << std::fixed << std::setprecision(6) << absolute_force_threshold_ << " N" << std::endl;
      log_file_ << "# ---------------------------------------" << std::endl;
      log_file_ << "time,position_z,penetration_depth,gazebo_force_z,estimated_force_z,error_z,contact_by_depth,contact_by_force,in_contact,force_change,max_force_change,first_contact_z" << std::endl;
      
      log_initialized_ = true;
      start_time_ = ros::Time::now();
      log_counter_ = 0;
      
      ROS_INFO("日志文件已初始化: %s", filename.c_str());
    } else {
      ROS_ERROR("无法创建日志文件: %s", filename.c_str());
    }
  }
  
  /**
   * @brief 关闭日志文件
   */
  void closeLogFile() {
    if (log_initialized_ && log_file_.is_open()) {
      // 记录实验结束信息
      ros::Time end_time = ros::Time::now();
      double duration = (end_time - start_time_).toSec();
      
      log_file_ << "# ---------------------------------------" << std::endl;
      log_file_ << "# 实验结束时间: " << std::fixed << std::setprecision(6) << duration << " 秒" << std::endl;
      log_file_ << "# 实验总时长: " << std::fixed << std::setprecision(6) << duration << " 秒" << std::endl;
      log_file_ << "# 记录的数据点数: " << log_counter_ << std::endl;
      log_file_ << "# 接触检测率: " << std::fixed << std::setprecision(6) 
                << (total_updates_ > 0 ? (contact_detected_count_ * 100.0 / total_updates_) : 0.0) 
                << "%" << std::endl;
      
      // 关闭文件
      log_file_.close();
      ROS_INFO("软接触力估计器日志已关闭: %s", log_file_path_.c_str());
      ROS_INFO("实验总时长: %.2f 秒", duration);
      ROS_INFO("记录的数据点数: %d", log_counter_);
      ROS_INFO("接触检测率: %.2f%% (%d/%d)", 
               (total_updates_ > 0 ? (contact_detected_count_ * 100.0 / total_updates_) : 0.0),
               contact_detected_count_, total_updates_);
    }
  }

  /**
   * @brief 周期性打印调试信息的回调函数
   */
  void debugTimerCallback(const ros::TimerEvent&) {
    // 输出关键状态信息
    ROS_INFO("===== 接触模型调试信息 =====");
    ROS_INFO("机械臂末端位置: (%.3f, %.3f, %.3f) m", 
             current_position_(0), current_position_(1), current_position_(2));
    
    // 打印接触位置和深度计算信息
    if (contact_position_initialized_) {
      double penetration_depth = std::max(0.0, first_contact_z_ - current_position_(2));
      ROS_INFO("接触点记录 - 首次接触Z位置: %.6f m", first_contact_z_);
      ROS_INFO("压入深度(接触点Z - 当前Z): %.6f m", penetration_depth);
    } else {
      ROS_INFO("尚未检测到接触 - 参考软块表面高度: %.3f m", soft_block_z_);
      ROS_INFO("离接触面距离: %.6f m", current_position_(2) - soft_block_z_);
    }
    
    // 检查各种接触状态
    ROS_INFO("当前力值: %.3f N, 力变化: %.3f N (阈值 %.3f N)", 
             gazebo_force_z_, getCurrentForceChange(), force_change_threshold_);
             
    // 显示当前接触检测模式
    if (contact_position_initialized_) {
      ROS_INFO("接触检测模式: 已建立接触，仅使用绝对力判断 (阈值: %.3f N)", absolute_force_threshold_);
      ROS_INFO("接触状态 - 绝对力(%s) > 阈值: %s", 
             std::to_string(std::abs(gazebo_force_z_)).c_str(),
             is_in_contact_ ? "是" : "否");
    } else {
      ROS_INFO("接触检测模式: 首次接触，需同时满足力变化和绝对力条件");
      ROS_INFO("接触状态 - 基于力变化: %s, 基于绝对力: %s, 综合: %s", 
             is_contact_by_force_ ? "是" : "否", 
             is_contact_by_absolute_force_ ? "是" : "否",
             is_in_contact_ ? "是" : "否");
    }
    
    if (is_in_contact_) {
      ROS_INFO("当前状态: 接触中 (连续接触帧: %d)", consecutive_contact_frames_);
    } else {
      ROS_INFO("当前状态: 未接触 (连续未接触帧: %d)", -consecutive_contact_frames_);
    }
    
    // 计算有效弹性模量和理论力值
    double E_effective = contact_model_->computeEffectiveModulus();
    double theoretical_force = 0.0;
    
    if (is_in_contact_ && contact_position_initialized_) {
      // 计算压入深度和理论力
      double depth = std::max(0.0, first_contact_z_ - current_position_(2));
      double R = contact_model_->getContactRadius();
      theoretical_force = (4.0/3.0) * E_effective * std::sqrt(R) * std::pow(depth, 1.5);
      
      ROS_INFO("Hertz接触力计算: (4/3) * %.1f * sqrt(%.3f) * %.6f^(3/2) = %.3f N", 
               E_effective, R, depth, theoretical_force);
    }
    
    ROS_INFO("有效弹性模量: %.1f Pa", E_effective);
    ROS_INFO("Gazebo力: %.3f N", gazebo_force_z_);
    ROS_INFO("估计力: %.3f N", estimated_force_z_);
    ROS_INFO("接触检测率: %.1f%% (%d/%d)", 
             (contact_detected_count_ * 100.0 / std::max(1, total_updates_)),
             contact_detected_count_, total_updates_);
    ROS_INFO("============================");
  }

  /**
   * @brief 格式化时间字符串
   * @return 格式化的时间字符串
   */
  std::string getFormattedTime() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", std::localtime(&time_t_now));
    return std::string(buffer);
  }
  
  /**
   * @brief 计算赫兹接触力
   * @param depth 接触深度
   * @return 力值
   */
  double computeHertzContactForce(double depth) {
    // 对于极小深度返回最小力值
    if (depth < 0.0001) {
      return 0.1; // 最小接触力0.1N
    }
    
    // 获取接触模型参数
    double E_effective = contact_model_->computeEffectiveModulus();
    double R = contact_model_->getContactRadius();
    
    // 使用Hertz接触模型计算法向力: F = (4/3) * E_effective * sqrt(R) * depth^(3/2)
    double force = (4.0/3.0) * E_effective * std::sqrt(R) * std::pow(depth, 1.5);
    
    // 确保力至少有一个最小值 (稳定性)
    return std::max(force, 0.1); // 最小0.1N
  }
  
  /**
   * @brief 获取当前力的变化量
   * @return 力变化量
   */
  double getCurrentForceChange() {
    if (force_history_.size() < 2) {
      return 0.0;
    }
    
    // 计算相邻时间点的力变化量（最新力值与前一个时间点的差值）
    double latest_force = force_history_.back();
    double previous_force = *(force_history_.rbegin() + 1); // 取倒数第二个元素
    
    return std::fabs(latest_force - previous_force);
  }
  
  /**
   * @brief 更新力历史并检测力突变
   * @param force 当前力值
   * @return 检测到的力变化
   */
  double updateForceHistory(double force) {
    // 添加新的力值到历史队列
    force_history_.push_back(force);
    if (force_history_.size() > force_window_size_) {
      force_history_.pop_front();
    }
    
    // 计算当前力变化
    double force_change = getCurrentForceChange();
    
    // 更新最大力变化
    if (force_change > max_force_change_) {
      max_force_change_ = force_change;
    }
    
    return force_change;
  }
  
  /**
   * @brief 基于力变化检测接触
   * @param force 当前力值
   * @return 是否接触
   */
  bool detectContactByForce(double force) {
    // 更新力历史并获取力变化
    double force_change = updateForceHistory(force);
    
    // 只有队列填满后才考虑力变化检测（防止启动时的误判）
    if (force_history_.size() < force_window_size_) {
      return false;
    }
    
    // 如果力变化超过阈值，判定为接触
    if (force_change > force_change_threshold_) {
      return true;
    }
    
    return false;
  }
  
  /**
   * @brief 基于绝对力值检测接触
   * @param force 当前力值
   * @return 是否接触
   */
  bool detectContactByAbsoluteForce(double force) {
    // 必须接收足够多的力数据才开始检测，以避免启动时的噪声
    if (total_updates_ < 10) {
      return false;
    }
    
    // 如果力值超过阈值，判定为接触
    if (std::fabs(force) > absolute_force_threshold_) {
      return true;
    }
    return false;
  }
  
  /**
   * @brief Franka状态回调函数
   * @param msg Franka状态消息
   */
  void frankaStateCallback(const franka_msgs::FrankaState::ConstPtr& msg) {
    total_updates_++;
    
    // 获取外部力估计的z分量（Gazebo力 - 作为真值）
    gazebo_force_z_ = msg->O_F_ext_hat_K[2];
    
    // 尝试从Franka状态中提取末端位置信息 
    // O_T_EE为4x4变换矩阵，按列优先存储
    if (msg->O_T_EE.size() == 16) { // 确保数据完整
      // 注意: FrankaState中O_T_EE是按列优先顺序排列的
      // 位置在最后一列: [12], [13], [14]
      current_position_(0) = msg->O_T_EE[12];
      current_position_(1) = msg->O_T_EE[13];
      current_position_(2) = msg->O_T_EE[14];
      
      // 更新姿态四元数(可选，如果需要的话)
      Eigen::Matrix3d rot;
      rot << msg->O_T_EE[0], msg->O_T_EE[4], msg->O_T_EE[8],
             msg->O_T_EE[1], msg->O_T_EE[5], msg->O_T_EE[9],
             msg->O_T_EE[2], msg->O_T_EE[6], msg->O_T_EE[10];
      current_orientation_ = Eigen::Quaterniond(rot);
      
      if (debug_mode_ && total_updates_ % 100 == 0) {
        ROS_DEBUG("从FrankaState更新位置: (%.3f, %.3f, %.3f) m", 
                 current_position_(0), current_position_(1), current_position_(2));
      }
    }
    
    // 初始化变量
    double penetration_depth = 0.0;
    is_contact_by_depth_ = false;
    is_contact_by_force_ = false;
    is_contact_by_absolute_force_ = false;
    
    // 启动初期不进行接触检测，等待系统稳定
    if (total_updates_ < 10) {
      // 发布估计力和对比力
      publishForces();
      logForceData(ros::Time::now(), 0.0);
      return;
    }
    
    // 1. 基于力的接触检测
    // 使用力变化检测
    if (use_force_change_detection_) {
      is_contact_by_force_ = detectContactByForce(gazebo_force_z_);
      
      if (is_contact_by_force_ && debug_mode_ && (total_updates_ % 100 == 0)) {
        ROS_DEBUG("基于力变化检测到接触: 力 = %.3f N, 变化 = %.3f N", 
                 gazebo_force_z_, getCurrentForceChange());
      }
    }
    
    // 使用绝对力值检测
    if (use_absolute_force_detection_) {
      is_contact_by_absolute_force_ = detectContactByAbsoluteForce(gazebo_force_z_);
      
      if (is_contact_by_absolute_force_ && debug_mode_ && (total_updates_ % 100 == 0)) {
        ROS_DEBUG("基于绝对力值检测到接触: 力 = %.3f N (阈值 %.3f N)", 
                 gazebo_force_z_, absolute_force_threshold_);
      }
    }
    
    // 2. 综合力检测结果 - 首次接触需同时满足两个条件，已接触后只看绝对力
    bool in_contact = false;
    
    // 如果尚未建立接触，需要同时满足力变化和绝对力条件
    if (!contact_position_initialized_) {
      in_contact = is_contact_by_force_ && is_contact_by_absolute_force_;
      
      if (in_contact && debug_mode_) {
        ROS_INFO("首次接触条件满足: 力变化 = %.3f N, 绝对力 = %.3f N", 
                getCurrentForceChange(), std::abs(gazebo_force_z_));
      }
    } 
    // 已经建立接触后，只要力绝对值还满足要求，就保持接触状态
    else {
      // 只有当力绝对值小于阈值时才判断为不接触
      in_contact = std::abs(gazebo_force_z_) >= absolute_force_threshold_;
      
      if (!in_contact && debug_mode_) {
        ROS_INFO("接触结束: 当前力值 %.3f N 小于阈值 %.3f N", 
                std::abs(gazebo_force_z_), absolute_force_threshold_);
      }
    }
    
    // 3. 记录首次接触位置，并据此计算压入深度
    if (in_contact) {
      // 如果第一次检测到接触，记录当前Z位置作为接触点
      if (!contact_position_initialized_) {
        first_contact_z_ = current_position_(2);
        contact_position_initialized_ = true;
        ROS_INFO("首次检测到接触! 记录接触点Z坐标: %.6f m", first_contact_z_);
      }
      
      // 计算当前压入深度 - 这里使用接触点位置减去当前位置
      penetration_depth = std::max(0.0, first_contact_z_ - current_position_(2));
      
      if (debug_mode_ && (total_updates_ % 100 == 0)) {
        ROS_DEBUG("计算压入深度: %.6f m (当前Z: %.6f m, 接触点Z: %.6f m)", 
                 penetration_depth, current_position_(2), first_contact_z_);
      }
      
      contact_detected_count_++;
      consecutive_contact_frames_++;
    } else {
      // 未接触时，保持接触点不变，但只有当连续多帧未接触时才重置
      if (consecutive_contact_frames_ > 0) {
        consecutive_contact_frames_ = 0;
      } else {
        consecutive_contact_frames_--;
        
        // 如果连续多帧未检测到接触，重置接触点
        if (consecutive_contact_frames_ < -10 && contact_position_initialized_) {
          contact_position_initialized_ = false;
          ROS_INFO("接触结束，重置接触点记录");
        }
      }
    }
    
    // 更新接触状态
    is_in_contact_ = in_contact;
    
    // 使用ADMM软接触模型估计力
    ContactState state;
    
    // 使用接触模型检测接触状态
    if (in_contact && contact_position_initialized_) {
      // 手动更新接触状态
      state.contact_depth = penetration_depth;
      state.in_contact = true;
      state.contact_position = current_position_;
      state.contact_normal = Eigen::Vector3d(0, 0, 1); // 接触法向量向上
      
      // 使用赫兹接触模型计算力
      double contact_force = computeHertzContactForce(penetration_depth);
      state.contact_force = contact_force;
      state.setForce(contact_force);
      
      estimated_force_z_ = contact_force;
      
      if (debug_mode_ && (total_updates_ % 100 == 0 || consecutive_contact_frames_ == 1)) {
        ROS_DEBUG("手动计算接触力: 深度 = %.6f m, Fz = %.3f N", 
                 penetration_depth, estimated_force_z_);
      }
    } else {
      // 未接触状态
      state.in_contact = false;
      state.contact_depth = 0.0;
      state.contact_force = 0.0;
      state.setForce(0.0);
      estimated_force_z_ = 0.0;
    }
    
    // 同时也用软接触模型更新状态（可能会与我们的手动计算不同）
    ContactState model_state = contact_model_->updateContact(
        current_position_,
        current_velocity_,
        current_orientation_,
        soft_block_z_
    );
    
    // 比较两种方法的结果，如果不一致进行记录
    if (model_state.in_contact != state.in_contact && debug_mode_ && (total_updates_ % 50 == 0)) {
      ROS_WARN("接触检测不一致! 手动检测: %s, 模型检测: %s, 深度 = %.6f m", 
               state.in_contact ? "接触" : "未接触",
               model_state.in_contact ? "接触" : "未接触",
               penetration_depth);
    }
    
    // 发布估计力和对比力
    publishForces();
    
    // 记录数据
    logForceData(ros::Time::now(), penetration_depth);
  }
  
  /**
   * @brief 关节状态回调函数
   * @param msg 关节状态消息
   */
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // 简化处理：从关节状态更新速度估计
    // 注：实际应用中应使用雅可比矩阵和关节速度计算末端速度
    static Eigen::Vector3d last_position = current_position_;
    static ros::Time last_time = ros::Time::now();
    
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    
    if (dt > 0.001) { // 避免除零
      // 简单差分计算速度
      current_velocity_ = (current_position_ - last_position) / dt;
      
      // 更新上一次位置和时间
      last_position = current_position_;
      last_time = current_time;
    }
  }
  
  /**
   * @brief 发布力数据
   */
  void publishForces() {
    // 发布估计力
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "panda_link0";
    
    // 只设置z方向的力值
    msg.wrench.force.x = 0.0;
    msg.wrench.force.y = 0.0;
    msg.wrench.force.z = estimated_force_z_;
    msg.wrench.torque.x = 0.0;
    msg.wrench.torque.y = 0.0;
    msg.wrench.torque.z = 0.0;
    
    // 发布估计力
    estimated_force_pub_.publish(msg);
    
    // 发布Gazebo力作为对比
    geometry_msgs::WrenchStamped gazebo_msg;
    gazebo_msg.header = msg.header;
    gazebo_msg.wrench.force.x = 0.0;
    gazebo_msg.wrench.force.y = 0.0;
    gazebo_msg.wrench.force.z = gazebo_force_z_;
    gazebo_msg.wrench.torque.x = 0.0;
    gazebo_msg.wrench.torque.y = 0.0;
    gazebo_msg.wrench.torque.z = 0.0;
    
    // 发布Gazebo力
    gazebo_force_pub_.publish(gazebo_msg);
    
    // 发布力对比信息（用于rqt_plot）
    geometry_msgs::WrenchStamped compare_msg;
    compare_msg.header = msg.header;
    compare_msg.wrench.force.x = 0.0;
    compare_msg.wrench.force.y = 0.0;
    compare_msg.wrench.force.z = gazebo_force_z_ - estimated_force_z_;  // 误差
    compare_msg.wrench.torque.x = 0.0;
    compare_msg.wrench.torque.y = 0.0;
    compare_msg.wrench.torque.z = 0.0;
    
    // 发布力对比
    force_comparison_pub_.publish(compare_msg);
    
    // 如果有接触，发布接触位姿
    if (is_in_contact_ && contact_position_initialized_) {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header = msg.header;
      pose_msg.pose.position.x = current_position_(0);
      pose_msg.pose.position.y = current_position_(1);
      pose_msg.pose.position.z = current_position_(2);
      pose_msg.pose.orientation.w = current_orientation_.w();
      pose_msg.pose.orientation.x = current_orientation_.x();
      pose_msg.pose.orientation.y = current_orientation_.y();
      pose_msg.pose.orientation.z = current_orientation_.z();
      
      // 发布接触位姿
      estimated_pose_pub_.publish(pose_msg);
    }
  }
  
  /**
   * @brief 记录力数据到日志文件
   * @param time 当前时间
   * @param penetration_depth 压入深度
   */
  void logForceData(const ros::Time& time, double penetration_depth) {
    if (!log_initialized_) return;
    
    log_counter_++;
    double elapsed = (time - start_time_).toSec();
    
    // 计算当前的力变化（相邻时间点的差值）
    double force_change = getCurrentForceChange();
    
    // 判断位置数据是否有效，如果无效则记录警告
    bool valid_position = (std::abs(current_position_(0)) > 0.001 || 
                           std::abs(current_position_(1)) > 0.001 || 
                           std::abs(current_position_(2)) > 0.001);
    
    if (!valid_position && debug_mode_ && (log_counter_ % 50 == 0)) {
      ROS_WARN("记录日志时发现位置数据无效: (%.3f, %.3f, %.3f)", 
               current_position_(0), current_position_(1), current_position_(2));
    }
    
    if (log_file_.is_open()) {
      log_file_ << std::fixed << std::setprecision(6)
                << elapsed << ","                                   // 时间
                << current_position_(2) << ","                      // Z位置
                << penetration_depth << ","                         // 压入深度
                << gazebo_force_z_ << ","                           // Gazebo力
                << estimated_force_z_ << ","                        // 估计力
                << gazebo_force_z_ - estimated_force_z_ << ","      // 力误差
                << "0" << ","                                       // 基于深度的接触(已禁用)
                << (is_contact_by_force_ || is_contact_by_absolute_force_ ? 1 : 0) << ","  // 基于力的接触
                << (is_in_contact_ ? 1 : 0) << ","                 // 综合接触状态
                << force_change << ","                             // 力变化量
                << max_force_change_ << ","                        // 最大力变化量
                << (contact_position_initialized_ ? first_contact_z_ : 0.0) // 首次接触Z位置
                << std::endl;
    }
  }
};

} // namespace franka_example_controllers

int main(int argc, char** argv) {
  ros::init(argc, argv, "soft_contact_force_estimator");
  ros::NodeHandle nh("~");
  
  franka_example_controllers::SoftContactForceEstimator force_estimator(nh);
  
  ros::spin();
  
  return 0;
} 