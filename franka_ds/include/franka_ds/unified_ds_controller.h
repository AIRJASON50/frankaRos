// Copyright (c) 2024 Franka DS Project
// Use of this source code is governed by the Apache-2.0 license
#pragma once

#include <array>
#include <memory>
#include <string>
#include <mutex>
#include <iomanip>
#include <sstream>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>

#include <Eigen/Dense>
#include <franka_ds/ds_robot_state.h>
#include <franka_ds/energy_tank_manager.h>
#include <franka_ds/ds_primitives.h>
#include <franka_ds/ds_utils.h>

namespace franka_ds {

// 类型定义
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

/**
 * @brief 统一DS-阻抗控制器类
 * 
 * 该控制器整合了动力学系统(DS)理论和阻抗控制，实现基于力矩控制的柔顺机械臂控制。
 * 
 * 核心特性：
 * - DS引导的笛卡尔速度规划
 * - 阻抗控制律：tau = J^T * (K*(x_d-x) + D*(dx_ds-dx))
 * - 能量罐保证的无源性控制
 * - 接触力检测和响应
 * - 多阶段状态机管理
 * 
 * 控制架构：
 * DS调制器 -> 期望笛卡尔速度 -> 阻抗控制律 -> 关节力矩命令
 */
class UnifiedDSController : public controller_interface::MultiInterfaceController<
                                       hardware_interface::EffortJointInterface,
                                       franka_hw::FrankaStateInterface,
                                       franka_hw::FrankaModelInterface> {
 public:
  /**
   * @brief Controller initialization
   */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  
  /**
   * @brief Controller startup
   */
  void starting(const ros::Time& time) override;
  
  /**
   * @brief Controller update loop (main control logic)
   */
  void update(const ros::Time& time, const ros::Duration& period) override;
  
  /**
   * @brief Controller shutdown
   */
  void stopping(const ros::Time& time) override;

 private:
  // ========== Control Phase Definition ==========
  enum ControlPhase {
    CALIBRATION,       // 校准阶段（零速度，力传感器校准）
    INITIALIZATION,    // 初始化阶段（零速度，等待start命令）
    ACCELERATION,      // 加速阶段（从零开始逐渐增加速度）
    LINEAR_APPROACH,   // 线性接近阶段（恒定速度接近目标）
    PROBE_DESCENT,     // 探测下降阶段（接触检测）
    CIRCULAR_MOTION    // 圆形运动阶段（接触后的表面运动）
  };

  // ========== DS-Impedance Control Parameters ==========
  struct DSImpedanceParams {
    // 目标位置参数
    Eigen::Vector3d target_position;      // 目标位置 [m]
    
    // DS参数
    double linear_lambda_;                // 线性DS收敛速率 [1/s]
    double linear_max_velocity_;          // 线性DS最大速度 [m/s]
    double circular_radius_;              // 圆形DS半径 [m]
    // circular_omega_ 已删除 - 代码中实际使用 constant_tangential_speed_
    double exploration_speed_;            // 探测下降速度 [m/s]
    
    // DS控制核心参数
    double ds_impedance_gain_;            // DS阻抗控制增益 d1 [N*s/m]
    double ds_damping_gain_;              // DS阻尼增益 [N*s/m]
    double approach_gain_;                // 接近运动增益
    double radial_gain_;                  // 径向收缩增益
    double constant_tangential_speed_;    // 固定切向速度 [m/s]
    double blend_distance_;               // 运动模式混合距离阈值 [m]
    double min_velocity_;                 // 最小速度阈值 [m/s]
    
    // 姿态控制参数
    double orientation_stiffness_;        // 姿态控制刚度 [Nm/rad]
    double orientation_damping_;          // 姿态控制阻尼 [Nm*s/rad]
    double max_orientation_torque_;       // 最大姿态控制力矩 [Nm]
    
    // 接触力控制参数
    double desired_normal_force_;         // 期望法向接触力 [N]
    double contact_surface_height_;       // 接触面高度 [m] - 用于接触检测和力控制
    double filtered_force_gain_;          // 力传感器滤波增益 [0-1] - 修复：添加可配置滤波参数
    double force_modulation_gain_;        // 力调制增益 γ_p - 修复：添加gamma_p参数，用于力调制计算
    
    // 安全力矩限制
    double max_joint_torque_;             // 每个关节最大安全力矩 [Nm]
    
    // 阻抗参数
    Eigen::Matrix3d cartesian_stiffness_pos_;  // 笛卡尔位置刚度 [N/m]
    Eigen::Matrix3d cartesian_stiffness_ori_;  // 笛卡尔方向刚度 [Nm/rad]
    Eigen::Matrix3d cartesian_damping_pos_;    // 笛卡尔位置阻尼 [Ns/m]
    Eigen::Matrix3d cartesian_damping_ori_;    // 笛卡尔方向阻尼 [Nms/rad]
    double nullspace_stiffness_;               // 零空间刚度 [Nm/rad]
    
    // 接触检测参数
    double contact_force_threshold_;      // 接触力阈值 [N]
    double max_contact_force_;            // 最大接触力 [N]
    
    // 能量罐参数
    double energy_tank_max_;              // 能量罐最大容量 [J]
    
    // 安全参数
    double max_velocity_;                 // 最大速度限制 [m/s]
    double max_acceleration_;             // 最大加速度限制 [m/s²]
    double velocity_safety_factor_;       // 速度安全系数
    
    // 加速阶段参数
    double acceleration_duration_;        // 加速阶段持续时间 [s]

    double gamma_;           // 标量调节函数 γ
    double gamma_p_;         // 力调制增益 γ_p
    double d1_;              // DS-阻抗增益 d1

    bool is_enabled_;        // 能量罐是否启用
    bool is_initialized_;    // 能量罐是否已初始化

    // *** 添加：常量定义，使其更适应Frankas和DS理论 ***
  };

  // ========== Core Control Functions ==========
  
  /**
   * @brief 计算DS调制的期望速度
   * @param current_position 当前末端执行器位置
   * @return DS调制后的期望速度 [m/s]
   */
  Eigen::Vector3d computeDSModulatedVelocity(const Eigen::Vector3d& current_position);
  
  /**
   * @brief 计算阻抗控制力矩
   * @param desired_velocity DS调制的期望速度
   * @param current_pose 当前末端执行器位姿
   * @param current_velocity 当前末端执行器速度
   * @param jacobian 当前雅可比矩阵
   * @return 关节力矩命令
   */
  Vector7d computeImpedanceControl(const Eigen::Vector3d& desired_velocity,
                                   const Eigen::Affine3d& transform,
                                   const Eigen::Matrix<double, 6, 1>& current_velocity,
                                   const Eigen::Matrix<double, 6, 7>& jacobian);
  
  /**
   * @brief 计算线性DS速度场
   */
  Eigen::Vector3d computeLinearDS(const Eigen::Vector3d& current_position);
  
  /**
   * @brief 计算圆形DS速度场
   */
  Eigen::Vector3d computeCircularDS(const Eigen::Vector3d& current_position);
  
  /**
   * @brief 计算力调制速度场项 - DS统一力运动生成的核心
   * 按照论文公式：fn(x) = Fd(x)/d1 * n(x)
   * @param current_position 当前末端执行器位置
   * @return 力调制速度场项 [m/s]
   */
  Eigen::Vector3d computeForceModulation(const Eigen::Vector3d& current_position);
  
  /**
   * @brief 计算期望接触力（动态调整）
   * 参考原版DS动态力生成逻辑
   * @param current_position 当前末端执行器位置  
   * @return 期望接触力 [N]
   */
  double computeDesiredContactForce(const Eigen::Vector3d& current_position);
  
  /**
   * @brief 计算探测DS速度场
   */
  Eigen::Vector3d computeProbeDS(const Eigen::Vector3d& current_position);
  
  /**
   * @brief 检测接触状态
   */
  bool detectContact(const Eigen::Vector3d& external_force);
  
  /**
   * @brief 限制力矩变化率（安全保护）
   */
  Vector7d saturateTorqueRate(const Vector7d& tau_d_calculated, const Vector7d& tau_J_d);
  
  /**
   * @brief 更新控制阶段状态机
   * @param current_position 当前末端执行器位置
   */
  void updateControlPhase(double current_time, 
                         const Eigen::Vector3d& current_position, 
                         const Eigen::Vector3d& external_force);

  // ========== Parameter Loading ==========
  bool loadDSImpedanceParameters(ros::NodeHandle& nh);
  void initializeROSInterface(ros::NodeHandle& nh);

  // ========== ROS Interface ==========
  void userCommandCallback(const std_msgs::String::ConstPtr& msg);
  void forceDataCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  void publishControlStatus();
  void publishEnergyStatus();

  // ========== Member Variables ==========
  
  // Hardware interfaces
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  
  // DS-Impedance parameters
  DSImpedanceParams ds_impedance_params_;
  
  // Control state
  ControlPhase current_phase_;
  ros::Time phase_start_time_;
  ros::Time controller_start_time_;
  bool user_start_command_received_;
  
  // Control targets
  Eigen::Vector3d initial_position_;
  Eigen::Vector3d target_position_;
  Eigen::Quaterniond initial_orientation_;
  Eigen::Quaterniond target_orientation_;
  std::array<double, 7> initial_joint_positions_;  // 初始关节位置
  
  // Force sensing - 直接使用外部力传感器数据
  Eigen::Vector3d external_force_;
  Eigen::Vector3d contact_force_;
  double baseline_force_z_;
  bool force_sensor_calibrated_;
  
  // *** 修复：性能优化 - 预分配临时变量避免频繁内存分配 ***
  // 预分配的Eigen对象，在update函数中重复使用
  mutable Eigen::Affine3d transform_cache_;
  mutable Eigen::Vector3d position_cache_;
  mutable Eigen::Quaterniond orientation_cache_;
  mutable Vector6d cartesian_velocity_cache_;
  mutable Eigen::Vector3d nominal_ds_velocity_cache_;
  mutable Eigen::Vector3d constrained_velocity_cache_;
  mutable Vector7d tau_d_cache_;
  
  // Energy tank management
  std::unique_ptr<EnergyTankManager> energy_tank_manager_;
  
  // Safety limits
  static constexpr double kDeltaTauMax = 1.0;  // 最大力矩变化率 [Nm/ms]
  static constexpr double kMaxVelocity = 0.5;  // 最大速度 [m/s]
  
  // ROS interface
  ros::Subscriber sub_user_command_;
  ros::Subscriber sub_force_data_;
  ros::Publisher pub_control_status_;
  ros::Publisher pub_control_phase_;      // 添加控制阶段发布器
  ros::Publisher pub_energy_status_;
  ros::Publisher pub_velocity_debug_;
  
  // Debug and logging
  std::mutex status_mutex_;
  int debug_print_counter_;
  static constexpr int kDebugPrintRate = 200;  // 每200个周期打印一次调试信息
};

}  // namespace franka_ds 