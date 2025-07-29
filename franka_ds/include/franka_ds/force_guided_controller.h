// Copyright (c) 2024 Franka DS Project
// Use of this source code is governed by the Apache-2.0 license
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_ds {

/**
 * @brief 力引导控制器
 * 
 * 功能特性：
 * - 末端姿态保持竖直（Z轴向下）
 * - 可通过外力引导末端位置移动
 * - 撤销外力后保持当前位姿
 * - 支持动态设置吸引子位置和接触面高度
 */
class ForceGuidedController : public controller_interface::MultiInterfaceController<
                                 franka_hw::FrankaModelInterface,
                                 hardware_interface::EffortJointInterface,
                                 franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time&) override;

 private:
  // 控制参数结构体
  struct ControlParams {
    // 阻抗参数
    double translational_stiffness_;      // 位置刚度 [N/m]
    double rotational_stiffness_;         // 姿态刚度 [Nm/rad]
    double translational_damping_;        // 位置阻尼 [N*s/m]
    double rotational_damping_;           // 姿态阻尼 [Nm*s/rad]
    double nullspace_stiffness_;          // 零空间刚度 [Nm/rad]
    
    // 力引导参数
    double force_threshold_;              // 力引导阈值 [N]
    double force_gain_;                   // 力引导增益 [m/(N*s)]
    double max_force_velocity_;           // 最大力引导速度 [m/s]
    
    // 安全参数
    double max_joint_torque_;             // 最大关节力矩 [Nm]
    double delta_tau_max_;                // 最大力矩变化率 [Nm]
    
    // 滤波参数
    double filter_params_;                // 参数滤波系数
    
    // 吸引子和接触面参数
    Eigen::Vector3d attractor_position_;  // 吸引子位置 [m]
    double contact_surface_height_;       // 接触面高度 [m]
  };

  // 力矩饱和函数
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);

  // 参数加载
  bool loadControlParameters(ros::NodeHandle& nh);
  
  // ROS接口初始化
  void initializeROSInterface(ros::NodeHandle& nh);
  
  // 回调函数
  void forceDataCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  void attractorPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void commandCallback(const std_msgs::String::ConstPtr& msg);
  
  // 控制逻辑
  Eigen::Vector3d computeForceGuidedVelocity(const Eigen::Vector3d& external_force);
  Eigen::Vector3d computeDesiredPosition(const Eigen::Vector3d& current_position, 
                                         const Eigen::Vector3d& force_velocity, 
                                         double dt);

  // 硬件接口
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // 控制参数
  ControlParams params_;
  
  // 控制状态
  Eigen::Vector3d position_d_;              // 期望位置（被动模式下跟随当前位置）
  Eigen::Vector3d position_d_target_;       // 目标位置（滤波用）
  
  // 外力感知
  Eigen::Vector3d external_force_;         // 外部力
  bool force_data_received_;               // 力数据接收标志
  
  // 力引导状态
  bool force_guided_mode_;                 // 力引导模式标志
  Eigen::Vector3d last_position_;          // 上一次位置
  
  // 线程安全
  std::mutex position_target_mutex_;
  std::mutex force_data_mutex_;
  
  // ROS接口
  ros::Subscriber sub_force_data_;
  ros::Subscriber sub_attractor_position_;
  ros::Subscriber sub_command_;
  ros::Publisher pub_current_pose_;
  ros::Publisher pub_control_status_;
  
  // 常量定义
  static constexpr double kMaxVelocity = 0.2;   // 最大移动速度 [m/s]
  static constexpr double kForceDeadzone = 1.0; // 力死区 [N]
};

}  // namespace franka_ds 