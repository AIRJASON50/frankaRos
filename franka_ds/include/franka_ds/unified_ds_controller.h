// Copyright (c) 2024 Franka DS Project
// Use of this source code is governed by the Apache-2.0 license

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

// Franka interfaces
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

// ROS messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

// DS components
#include <franka_ds/energy_tank_manager.h>

namespace franka_ds {

// 简化的控制状态：只有校准和运动两个状态
enum class SimpleControlState {
  CALIBRATING,  // 正在校准力传感器
  READY,        // 校准完成，等待启动
  MOTION        // 运行中
};

using Vector7d = Eigen::Matrix<double, 7, 1>;

class UnifiedDSController : public controller_interface::MultiInterfaceController<
                                hardware_interface::EffortJointInterface,
                                franka_hw::FrankaStateInterface,
                                franka_hw::FrankaModelInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  // === 参数结构体 ===
  struct DSImpedanceParams {
    // 目标位置和接触面
    Eigen::Vector3d target_position;
    double contact_surface_height_;
    
    // DS核心参数
    double ds_impedance_gain_;
    double ds_damping_gain_;
    double approach_gain_;
    double radial_gain_;
    
    // 运动参数
    double linear_lambda_;
    double linear_max_velocity_;
    double circular_radius_;
    double constant_tangential_speed_;
    double exploration_speed_;
    double blend_distance_;
    double min_velocity_;
    
    // 力控制参数
    double desired_normal_force_;
    double contact_force_threshold_;
    double max_contact_force_;
    double filtered_force_gain_;
    double force_modulation_gain_;
    
    // 阻抗控制参数
    Eigen::Matrix3d cartesian_stiffness_pos_;
    Eigen::Matrix3d cartesian_stiffness_ori_;
    Eigen::Matrix3d cartesian_damping_pos_;
    Eigen::Matrix3d cartesian_damping_ori_;
    
    // 姿态控制参数
    double orientation_stiffness_;
    double orientation_damping_;
    double max_orientation_torque_;
    
    // 零空间参数
    double nullspace_stiffness_;
    
    // 安全参数
    double max_velocity_;
    double max_acceleration_;
    double max_joint_torque_;
    double velocity_safety_factor_;
    
    // 能量罐参数
    double energy_tank_max_;
  };

  // === 核心计算函数 ===
  
  // 统一速度场计算（新的核心函数）
  Eigen::Vector3d computeUnifiedDS(const Eigen::Vector3d& current_position);
  
  // 基础速度场：接近场与圆周场的自动混合
  Eigen::Vector3d computeBaseVelocityField(const Eigen::Vector3d& current_position);
  
  // 接近速度场
  Eigen::Vector3d computeApproachVelocity(const Eigen::Vector3d& relative_position);
  
  // 圆周速度场
  Eigen::Vector3d computeCircularVelocity(const Eigen::Vector3d& relative_position, 
                                          const Eigen::Vector3d& attractor);
  
  // 力调制
  Eigen::Vector3d computeForceModulation(const Eigen::Vector3d& current_position);
  double computeDesiredContactForce(const Eigen::Vector3d& current_position);
  
  // 探测DS（保留但简化）
  Eigen::Vector3d computeProbeDS(const Eigen::Vector3d& current_position);
  
  // 阻抗控制
  Vector7d computeImpedanceControl(const Eigen::Vector3d& desired_velocity,
                                  const Eigen::Affine3d& transform,
                                  const Eigen::Matrix<double, 6, 1>& current_velocity,
                                  const Eigen::Matrix<double, 6, 7>& jacobian);

  // === 辅助函数 ===
  bool detectContact(const Eigen::Vector3d& external_force);
  Vector7d saturateTorqueRate(const Vector7d& tau_d_calculated, const Vector7d& tau_J_d);
  void updateSimpleControlState(double current_time, 
                               const Eigen::Vector3d& current_position, 
                               const Eigen::Vector3d& external_force);

  // === 参数和ROS接口 ===
  bool loadDSImpedanceParameters(ros::NodeHandle& nh);
  void initializeROSInterface(ros::NodeHandle& nh);
  
  // ROS回调函数
  void userCommandCallback(const std_msgs::String::ConstPtr& msg);
  void forceDataCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  
  // 状态发布
  void publishControlStatus();
  void publishEnergyStatus();

  // === 硬件接口 ===
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

  // === 控制状态 ===
  bool is_calibrated_;              // 是否已校准
  bool motion_started_;             // 是否已开始运动
  
  // === 控制参数 ===
  DSImpedanceParams ds_impedance_params_;
  
  // === 状态变量 ===
  Eigen::Vector3d initial_position_;
  Eigen::Quaterniond initial_orientation_;
  Eigen::Vector3d target_position_;
  Eigen::Quaterniond target_orientation_;
  std::array<double, 7> initial_joint_positions_;
  
  // === 传感器数据 ===
  Eigen::Vector3d external_force_;
  Eigen::Vector3d contact_force_;
  double baseline_force_z_;
  
  // === 能量罐管理 ===
  std::unique_ptr<EnergyTankManager> energy_tank_manager_;
  
  // === 性能优化缓存 ===
  mutable Eigen::Affine3d transform_cache_;
  mutable Eigen::Vector3d position_cache_;
  mutable Eigen::Quaterniond orientation_cache_;
  mutable Eigen::Matrix<double, 6, 1> cartesian_velocity_cache_;
  mutable Eigen::Vector3d nominal_ds_velocity_cache_;
  mutable Eigen::Vector3d constrained_velocity_cache_;
  mutable Vector7d tau_d_cache_;
  
  // === ROS接口 ===
  ros::Subscriber sub_user_command_;
  ros::Subscriber sub_force_data_;
  ros::Publisher pub_control_status_;
  ros::Publisher pub_control_phase_;
  ros::Publisher pub_energy_status_;
  ros::Publisher pub_velocity_debug_;
  
  // === 时间和调试 ===
  ros::Time controller_start_time_;
  size_t debug_print_counter_;
  
  // === 常量 ===
  static constexpr double kDeltaTauMax = 1.0;
  static constexpr size_t kDebugPrintRate = 500; // 每500次循环打印一次调试信息（约2Hz在1000Hz控制频率下）
};

}  // namespace franka_ds 