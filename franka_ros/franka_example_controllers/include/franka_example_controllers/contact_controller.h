// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>
#include <mutex>
#include <thread>
#include <vector>
#include <algorithm>
#include <iostream>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <Eigen/Dense>
#include <franka_example_controllers/soft_contact_model.h>

// 前向声明
namespace franka_example_controllers {
  class LogGenerator;
  class EnergyTankMonitor;
}

namespace franka_example_controllers {

/**
 * @brief 接触控制器类，实现圆周运动、力传感器监控、日志记录和能量管理
 */
class ContactController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaPoseCartesianInterface,
                                                franka_hw::FrankaModelInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  /**
   * @brief 控制器初始化
   */
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  
  /**
   * @brief 控制器启动
   */
  void starting(const ros::Time& time) override;
  
  /**
   * @brief 控制器更新循环
   */
  void update(const ros::Time& time, const ros::Duration& period) override;
  
  /**
   * @brief 控制器停止
   */
  void stopping(const ros::Time& time) override;

 private:
  // 控制阶段枚举
  enum ControlPhase {
    CALIBRATION = 0,        // 调零阶段
    APPROACH = 1,           // 接近阶段
    CONTACT = 2,            // 接触阶段  
    DEPTH_CONTROL = 3,      // 下探阶段
    TRAJECTORY = 4,         // 轨迹运动阶段（运动到目标位置）
    PAUSE_AT_TARGET = 5,    // 到达目标后停顿阶段
    PROBE_DESCENT = 6,      // 下探接触阶段
    PROBE_PAUSE = 7,        // 下探后暂停阶段
    CIRCULAR_MOTION = 8     // 圆周运动阶段
  };

  // 力传感器数据采样结构
  struct ForceDataSample {
    ros::Time timestamp;
    Eigen::Vector3d force;
  };

  // ROS接口
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  
  // ROS发布器和订阅器
  ros::Publisher pose_pub_;           // 发布期望姿态
  ros::Publisher phase_pub_;          // 发布控制阶段
  ros::Publisher energy_pub_;         // 发布能量状态
  ros::Subscriber force_sub_;         // 力传感器数据订阅者
  ros::Subscriber raw_force_sub_;     // 原始力传感器数据订阅者（用于日志）
  
  // 控制参数
  ControlPhase control_phase_;
  ros::Time phase_start_time_;
  ros::Duration elapsed_time_;
  
  // 运动参数
  std::array<double, 16> initial_pose_;  // 初始位姿矩阵
  Eigen::Vector3d initial_position_;     // 初始位置
  Eigen::Vector3d target_position_;      // 目标位置
  double circle_radius_;                 // 圆周半径
  double motion_frequency_;              // 运动频率
  double probe_length_;                  // 探头长度
  
  // Trajectory smoothing variables (inspired by joint_impedance_example_controller)
  double vel_current_;        // Current velocity
  double vel_max_;           // Maximum velocity  
  double acceleration_time_; // Time to reach max velocity
  double angle_;             // Current angle in circular motion
  
  // 运动到目标位置相关变量
  bool move_to_target_complete_;         // 运动到目标位置完成标志
  ros::Time move_start_time_;            // 运动开始时间
  ros::Time pause_start_time_;           // 停顿开始时间
  ros::Time circle_motion_start_time_;   // 圆周运动开始时间
  Eigen::Vector3d circle_center_position_;  // 圆周运动中心位置
  Eigen::Vector3d target_reached_position_; // 到达目标位置时的实际位置（圆周上的起始点）
  double move_progress_;                 // 运动进度 (0.0 到 1.0)
  double circle_angle_;                  // 圆周运动当前角度（用于匀速运动）
  bool pause_pose_saved_;                // 标记是否已保存暂停姿态
  std::array<double, 16> circle_base_pose_;  // 圆周运动基准姿态
  
  // ✅ 新增：圆周运动渐进启动参数
  bool circle_motion_started_;           // 圆周运动已启动标志
  double circle_startup_duration_;       // 圆周运动启动渐进时间
  
  // ✅ 新增：分阶段圆周运动控制参数
  bool circle_force_control_enabled_;    // 圆周运动中的力控制使能标志
  double circle_force_control_delay_;    // 力控制启动延迟时间（秒）
  ros::Time circle_force_control_start_time_;  // 力控制启动时间
  
  // 下探阶段相关变量
  ros::Time probe_start_time_;           // 下探开始时间
  Eigen::Vector3d probe_start_position_; // 下探开始位置
  double probe_start_force_z_;           // 下探开始时的z轴力值
  bool probe_contact_detected_;          // 接触检测标志
  ros::Time probe_contact_time_;         // 接触检测时间
  ros::Time probe_pause_start_time_;     // 下探后暂停开始时间
  std::array<double, 16> probe_start_pose_;  // 下探开始姿态
  
  // 力传感器相关
  std::mutex force_mutex_;
  Eigen::Vector3d current_force_;        // 当前力读数
  Eigen::Vector3d raw_force_;            // 原始力读数
  bool force_data_available_;
  
  // 力传感器调零相关
  bool force_zeroing_complete_;          // 调零完成标志
  ros::Time zeroing_start_time_;         // 调零开始时间
  Eigen::Vector3d force_offset_;         // 力传感器偏移量
  std::vector<ForceDataSample> force_window_buffer_;  // 滑动窗口缓冲区
  
  // ==== 优化后的力传感器滤波系统 ====
  // 基本滤波变量
  bool force_filter_initialized_;       // 滤波器初始化标志
  Eigen::Vector3d filtered_force_;      // 滤波后的力值
  double force_ema_alpha_;              // 指数滑动平均滤波系数
  
  // 滑动窗口滤波（简化版）
  int force_window_size_;               // 滑动窗口大小
  std::vector<Eigen::Vector3d> force_history_buffer_;  // 力传感器历史数据缓冲区
  
  // 接触检测专用滤波
  double contact_detection_alpha_;      // 接触检测阶段的滤波系数
  Eigen::Vector3d contact_filtered_force_;  // 接触检测专用滤波值
  bool contact_filter_initialized_;     // 接触检测滤波器初始化标志
  // ===========================================
  
  // ==== 增强的力传感器滤波函数 ====
  Eigen::Vector3d applyForceFiltering(const Eigen::Vector3d& raw_force, bool use_contact_detection_filter = false);
  Eigen::Vector3d applyEMAFilter(const Eigen::Vector3d& raw_force, double alpha, 
                                Eigen::Vector3d& filtered_output, bool& is_initialized);
  // ============================================
  
  // ==== z轴力反馈控制系统 ====
  // 目标力值和控制状态
  double target_force_z_;               // 下压阶段记录的目标z轴力值
  bool force_feedback_enabled_;         // 力反馈控制使能标志
  bool target_force_recorded_;          // 目标力值已记录标志
  
  // PD控制器参数
  double force_kp_;                     // 力控制比例增益
  double force_kd_;                     // 力控制微分增益
  double force_error_prev_;             // 上一次的力误差
  double z_offset_max_;                 // z轴最大调节幅度（米）
  double z_offset_min_;                 // z轴最小调节幅度（米）
  
  // 新增平滑控制参数
  double max_change_per_step_;          // 单次最大变化量（米）
  double force_deadzone_;               // 力误差死区（牛顿）
  double smooth_alpha_;                 // 平滑滤波系数
  
  // 控制器状态变量
  double current_z_offset_;             // 当前z轴偏移量
  
  // ✅ 日志记录频率控制
  ros::Time last_log_time_;             // 上次日志记录时间
  double log_frequency_;                // 日志记录频率 (Hz)
  double log_period_;                   // 日志记录周期 (秒)
  
  // ==== z轴力反馈控制函数 ====
  // ✅ 新增：力控制渐进启动参数
  bool force_control_gradual_startup_;  // 力控制渐进启动标志
  ros::Time force_control_init_time_;   // 力控制初始化时间
  double force_control_startup_duration_;  // 力控制启动持续时间（秒）
  // ===============================
  
  // 用户输入控制
  bool waiting_for_user_command_;        // 等待用户输入标志
  bool user_input_ready_;                // 用户输入就绪标志
  bool user_input_thread_started_;       // 用户输入线程启动标志
  std::mutex user_input_mutex_;          // 用户输入互斥锁
  std::thread user_input_thread_;        // 用户输入监控线程
  
  // 日志和能量监控
  std::unique_ptr<LogGenerator> log_generator_;
  std::unique_ptr<EnergyTankMonitor> energy_monitor_;
  ContactParams contact_params_;
  double target_force_;
  
  /**
   * @brief 力传感器数据回调函数
   */
  void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  
  /**
   * @brief 原始力传感器数据回调函数
   */
  void rawForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  
  /**
   * @brief 发布控制阶段状态
   */
  void publishPhaseStatus(const std::string& phase_name);
  
  /**
   * @brief 处理力传感器调零过程
   * @param time 当前时间
   * @param raw_force 原始力传感器数据
   */
  void processForceZeroing(const ros::Time& time, const Eigen::Vector3d& raw_force);
  
  /**
   * @brief 启动用户输入监控线程
   */
  void startUserInputMonitoring();
  
  /**
   * @brief 检查用户输入是否就绪
   * @return true if user input is ready, false otherwise
   */
  bool checkUserInput();
  
  // ==== z轴力反馈控制函数 ====
  /**
   * @brief 记录接触时的目标力值
   * @param current_force_z 当前z轴力值
   */
  void recordTargetForce(double current_force_z);
  
  /**
   * @brief PD控制器计算z轴位移调节量
   * @param current_force_z 当前z轴力值
   * @param dt 时间间隔
   * @return z轴位移调节量
   */
  double calculateForceControlOutput(double current_force_z, double dt);
  
  /**
   * @brief 应用力反馈控制到z轴位置
   * @param base_z_position 基准z轴位置
   * @param current_force_z 当前z轴力值
   * @param dt 时间间隔
   * @return 调节后的z轴位置
   */
  double applyForceControl(double base_z_position, double current_force_z, double dt);
  // =============================
  
  // 可视化发布函数
  void publishVisualizationPose(const ros::Time& time, const franka::RobotState& robot_state);
};

}  // namespace franka_example_controllers
