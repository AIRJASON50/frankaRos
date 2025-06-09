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
    CALIBRATION = 0,    // 调零阶段
    APPROACH = 1,       // 接近阶段
    CONTACT = 2,        // 接触阶段  
    DEPTH_CONTROL = 3,  // 下探阶段
    TRAJECTORY = 4      // 轨迹运动阶段
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
  double circle_radius_;                 // 圆周半径
  double motion_frequency_;              // 运动频率
  double probe_length_;                  // 探头长度
  
  // Trajectory smoothing variables (inspired by joint_impedance_example_controller)
  double vel_current_;        // Current velocity
  double vel_max_;           // Maximum velocity  
  double acceleration_time_; // Time to reach max velocity
  double angle_;             // Current angle in circular motion
  
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
};

}  // namespace franka_example_controllers
