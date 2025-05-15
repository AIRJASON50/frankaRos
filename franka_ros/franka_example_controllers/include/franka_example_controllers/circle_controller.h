// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_example_controllers/soft_contact_model.h>
#include <franka_example_controllers/log_generator.h>
#include <franka_example_controllers/trajectory_generator.h>
#include <franka_example_controllers/force_generator.h>

namespace franka_example_controllers {

class CircleController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // 轨迹生成器
  std::unique_ptr<TrajectoryGenerator> trajectory_generator_;
  
  // 力轨迹生成器
  std::unique_ptr<ForceGenerator> force_generator_;
  
  // 控制参数
  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  double elapsed_time_{0.0};  // 已经过的时间（秒）
  
  // 目标位置和方向
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  
  // 初始位置（圆周中心）
  Eigen::Vector3d circle_center_;

  // 期望姿态相关
  bool desired_pose_initialized_ = false;
  Eigen::Affine3d desired_pose_ = Eigen::Affine3d::Identity();
  
  // 圆周运动平面相关
  Eigen::Vector3d circle_normal_ = Eigen::Vector3d::UnitZ(); // probe坐标系z轴（法线）
  Eigen::Vector3d circle_x_axis_ = Eigen::Vector3d::UnitX(); // 圆周平面x轴
  Eigen::Vector3d circle_y_axis_ = Eigen::Vector3d::UnitY(); // 圆周平面y轴
  
  // 开环控制相关
  Eigen::Matrix<double, 7, 1> initial_q_ = Eigen::Matrix<double, 7, 1>::Zero(); // 初始关节角度
  
  // 过渡控制相关
  Eigen::Matrix<double, 7, 1> tau_d_last_openloop_ = Eigen::Matrix<double, 7, 1>::Zero(); // 上一个开环控制周期的力矩
  
  // 停止相关
  bool stop_message_printed_{false}; // 跟踪是否已打印停止消息
  
  // 接触检测相关
  bool contact_detected_{false}; // 是否已经检测到接触
  Eigen::Vector3d contact_position_; // 接触点位置
  double force_threshold_{5.0}; // 接触力阈值(N)
  
  // 软接触模型和相关状态
  std::unique_ptr<franka_example_controllers::SoftContactModel> contact_model_;
  franka_example_controllers::ContactParams contact_params_;
  franka_example_controllers::ContactState contact_state_;
  
  // 接触点位置（软体块表面）
  Eigen::Vector3d soft_block_position_{0.4, 0.0, 0.505}; // 软体块位置
  double soft_block_x_{0.4};
  double soft_block_y_{0.0};
  double soft_block_z_{0.505};
  
  // 圆周轨迹参数 - 用于向后兼容，实际值由轨迹生成器管理
  double circle_frequency_{0.3}; // 圆周运动频率
  double circle_radius_{0.1};    // 圆周运动半径
  std::string circle_plane_{"xy"}; // 圆周运动平面
  
  // 接触控制相关
  enum ControlPhase {
    APPROACH = 0,   // 接近阶段
    CONTACT = 1,    // 接触阶段
    TRAJECTORY = 2    // 轨迹运动阶段（原CIRCULAR）
  };
  
  ControlPhase control_phase_{APPROACH};
  ros::Time phase_start_time_;
  double phase_duration_{2.0};  // 每个阶段默认持续时间
  
  // 恒力控制参数
  double target_force_{10.0};
  double base_force_{10.0}; // 力轨迹基准
  double force_error_integral_{0.0};
  double last_force_error_{0.0};
  double prev_force_error_{0.0}; // 之前的力误差，用于计算微分项
  double force_kp_{0.1};
  double force_ki_{0.01};
  double force_kd_{0.001};
  double force_p_gain_{0.1}; // 与force_kp_相同
  double force_i_gain_{0.01}; // 与force_ki_相同
  double force_d_gain_{0.001}; // 与force_kd_相同
  double max_force_{20.0};
  
  // 力噪声相关参数
  bool force_noise_enable_{false}; // 是否启用力噪声
  
  // 发布接触状态
  ros::Publisher phase_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher contact_pub_; // 发布接触力信息
  ros::Publisher path_pub_;    // 发布轨迹路径
  
  // 添加计数器变量，用于控制调试信息输出频率
  int circular_counter_{0};
  
  // 日志生成器
  LogGenerator log_generator_;
};

}  // namespace franka_example_controllers 