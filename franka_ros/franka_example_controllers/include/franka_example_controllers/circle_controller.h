// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <deque>

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

  // 公共接口，用于获取接触状态和深度
  bool isContactDetected() const { return contact_detected_; }
  double getContactDepth() const { return contact_depth_; }
  Eigen::Vector3d getContactPosition() const { return contact_position_; }
  Eigen::Vector3d getExternalForce() const { return external_force_; }

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
  bool contact_detected_{false};
  Eigen::Vector3d contact_position_{Eigen::Vector3d::Zero()};
  double contact_depth_{0.0};  // 接触深度，从接触点开始计算
  double contact_reference_z_{0.0}; // 接触参考Z坐标，首次接触时探头末端Z坐标
  Eigen::Vector3d external_force_{Eigen::Vector3d::Zero()}; // 外部力
  double force_threshold_{5.0}; // 接触力阈值(N)
  
  // 软接触模型和相关状态
  std::unique_ptr<franka_example_controllers::SoftContactModel> contact_model_;
  franka_example_controllers::ContactParams contact_params_;
  franka_example_controllers::ContactState contact_state_;
  
  // 接触点位置（软体块表面）
  Eigen::Vector3d soft_block_position_; // 软体块位置
  double soft_block_x_;
  double soft_block_y_;
  double soft_block_z_;
  double probe_length_; // 探头长度
  
  // 圆周轨迹参数 - 用于向后兼容，实际值由轨迹生成器管理
  double circle_frequency_{0.3}; // 圆周运动频率
  double circle_radius_{0.1};    // 圆周运动半径
  std::string circle_plane_{"xy"}; // 圆周运动平面
  
  // 接触控制相关
  /**
   * @brief 控制阶段枚举
   */
  enum ControlPhase {
    CALIBRATION,         // 力传感器调零阶段
    APPROACH,            // 接近阶段
    CONTACT_CONFIRMATION,// 接触确认和零点设置阶段
    DEPTH_CONTROL,       // 深度控制（下探）阶段
    TRAJECTORY           // 轨迹执行阶段
  };
  
  ControlPhase control_phase_{CALIBRATION}; // 默认从校准阶段开始
  ros::Time phase_start_time_;
  double phase_duration_{2.0};  // 每个阶段默认持续时间
  
  // 控制模式
  enum ControlMode {
    DEPTH_CONTROL_MODE = 0,    // 深度控制模式
    FORCE_CONTROL_MODE = 1     // 力控制模式
  };
  
  ControlMode control_mode_{DEPTH_CONTROL_MODE};  // 默认使用深度控制模式
  
  // 深度控制参数
  double target_depth_{0.001};  // 目标深度1mm
  double depth_p_gain_{1.5};    // 深度控制比例增益
  double depth_i_gain_{0.1};    // 深度控制积分增益
  double depth_d_gain_{0.2};    // 深度控制微分增益
  double depth_max_adjustment_{0.0015};  // 深度控制最大调整量
  double depth_error_integral_{0.0};  // 深度误差积分项
  double prev_depth_error_{0.0};      // 上一次深度误差，用于D项计算
  double trajectory_reference_depth_{0.0};  // 轨迹参考深度，保存稳定后的实际深度
  
  // 力控制参数
  double target_force_{10.0};   // 目标力
  double force_p_gain_{0.1};    // 力控制比例增益
  double force_i_gain_{0.01};   // 力控制积分增益
  double force_d_gain_{0.001};  // 力控制微分增益
  double force_error_integral_{0.0};  // 力误差积分项
  double prev_force_error_{0.0};      // 上一次力误差
  double last_force_error_{0.0};      // 上上次力误差
  double max_force_{20.0};      // 最大力限制
  
  // 力控制增益副本（用于兼容性）
  double force_kp_{0.1};        // 力控制比例增益副本
  double force_ki_{0.01};       // 力控制积分增益副本
  double force_kd_{0.001};      // 力控制微分增益副本
  
  // 力噪声相关参数
  bool force_noise_enable_{false}; // 是否启用力噪声
  
  // 发布接触状态
  ros::Publisher phase_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher contact_pub_; // 发布接触力信息
  ros::Publisher path_pub_;    // 发布轨迹路径
  // 力相关发布器
  ros::Publisher external_force_pub_; // 发布估计力信息
  ros::Publisher measured_force_pub_; // 发布测量力信息
  ros::Publisher contact_pose_pub_;   // 发布接触位姿
  
  // 添加计数器变量，用于控制调试信息输出频率
  int circular_counter_{0};
  
  // 日志生成器
  LogGenerator log_generator_;

  // 用于ROS话题发布的节点句柄
  ros::NodeHandle node_handle_;

  // 力传感器和接触检测相关变量
  std::deque<double> force_history_z_;        // Z方向力的历史值
  Eigen::Vector3d force_offset_{Eigen::Vector3d::Zero()};   // 力传感器零偏
  bool calibration_completed_{false};                      // 调零完成标志
  int calibration_sample_count_{0};                        // 调零采样计数
  static constexpr int calibration_required_samples_{500}; // 调零所需采样数，减少一些以便更快收敛初步测试
  double force_variance_threshold_{0.1};                   // 力方差阈值(N^2)，更严格
  double calibration_mean_force_threshold_{0.5};           // 平均力范数阈值(N)，按算法要求设置为0.5N
  std::deque<Eigen::Vector3d> force_samples_;              // 力采样队列，用于计算方差
  int calibration_attempts_{0};                            // 当前校准尝试次数
  static constexpr int max_calibration_attempts_{5};       // 最大校准尝试次数
  ros::Time calibration_stable_since_;                     // 校准进入稳定状态的起始时间
  bool calibration_criteria_met_{false};                   // 校准条件是否已满足（等待3秒稳定）
  double calibration_stability_duration_{1.0}; // 调零稳定时间要求（秒）
  
  // 调零和接近阶段相关变量
  Eigen::Vector3d initial_position_;         // 调零阶段初始法兰位置
  double approach_speed_{0.01};      // 默认接近速度: 1cm/s
  double approach_distance_{0.1};    // 默认接近距离: 10cm
  Eigen::Vector3d initial_approach_position_; // 接近阶段初始位置
  ros::Time approach_start_movement_time_{0}; // 记录接近阶段开始移动的时间
  double approach_start_time_ = 0.0;         // 接近阶段开始时间
  
  // 接触确认阶段
  ros::Time contact_confirmation_start_time_;
  double contact_confirmation_duration_{0.5}; // 接触确认持续时间（秒）

  // 深度控制阶段
  ros::Time depth_stable_since_;                         // 深度进入稳定状态的起始时间
  bool depth_criteria_met_{false};                       // 深度目标和稳定条件是否已满足
  double depth_stability_threshold_{0.0005}; // 深度稳定阈值 0.5mm - 从配置文件读取
  double depth_stability_duration_{2.0}; // 深度稳定持续时间要求（秒） - 从配置文件读取
  
  // 通用
  Eigen::Vector3d current_flange_position_; // 当前法兰位置
  Eigen::Vector3d current_probe_tip_position_; // 当前探头末端位置
  double current_calibrated_force_z_;      // 当前校准后的Z轴力
};

}  // namespace franka_example_controllers 