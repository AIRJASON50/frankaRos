#pragma once

#include <memory>
#include <string>
#include <array>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <realtime_tools/realtime_publisher.h>

#include <Eigen/Dense>
#include <franka/robot_state.h>

#include <franka_example_controllers/force_sensor_manager.h>
#include <franka_example_controllers/log_generator.h>

namespace franka_example_controllers {

/**
 * @brief 五阶段接触控制器
 * 
 * 实现五个控制阶段：
 * 0. CALIBRATION - 调零阶段
 * 1. APPROACH - 接近阶段  
 * 2. CONTACT_CONFIRMATION - 接触确认阶段
 * 3. DEPTH_CONTROL - 深度控制阶段
 * 4. TRAJECTORY - 轨迹运动阶段
 */
class ContactController : public controller_interface::MultiInterfaceController<
                            franka_hw::FrankaModelInterface,
                            hardware_interface::EffortJointInterface,
                            franka_hw::FrankaStateInterface> {
public:
  ContactController() = default;
  ~ContactController() override = default;

  /**
   * @brief 控制器初始化
   */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;

  /**
   * @brief 控制器启动
   */
  void starting(const ros::Time&) override;

  /**
   * @brief 主控制循环 (1kHz)
   */
  void update(const ros::Time&, const ros::Duration& period) override;

  /**
   * @brief 控制器停止
   */
  void stopping(const ros::Time&) override;

private:
  // 控制阶段枚举
  enum class ControlPhase {
    CALIBRATION = 0,          // 调零阶段
    APPROACH = 1,             // 接近阶段
    CONTACT_CONFIRMATION = 2, // 接触确认阶段
    DEPTH_CONTROL = 3,        // 深度控制阶段
    TRAJECTORY = 4            // 轨迹运动阶段
  };

  // 硬件接口
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // 力传感器管理器
  std::unique_ptr<ForceSensorManager> force_sensor_;
  
  // 日志记录器
  std::unique_ptr<LogGenerator> log_generator_;

  // 控制状态
  ControlPhase current_phase_;
  std::atomic<bool> phase_transition_requested_;
  
  // 时间相关
  ros::Time start_time_;
  ros::Time phase_start_time_;
  ros::Time last_log_time_;
  double log_period_;

  // 机器人状态
  std::array<double, 16> initial_pose_;
  Eigen::Vector3d initial_position_;
  Eigen::Vector3d current_position_;
  Eigen::Vector3d target_position_;
  
  // 力数据
  Eigen::Vector3d current_force_;
  Eigen::Vector3d calibrated_force_;
  bool force_data_valid_;

  // 调零相关参数
  struct CalibrationParams {
    int window_size;
    double stable_threshold;
    double stable_duration;
    double max_calibration_time;
    double force_limit;
  } calibration_params_;

  // 调零状态
  bool calibration_completed_;
  std::chrono::steady_clock::time_point calibration_start_;
  std::chrono::steady_clock::time_point stable_force_start_;
  bool stable_force_detected_;
  bool waiting_for_user_input_;

  // 接近阶段参数
  struct ApproachParams {
    double start_delay;
    double descent_velocity;
    double contact_threshold;
    double safety_force_limit;
  } approach_params_;

  // 接触确认参数
  struct ContactParams {
    double confirmation_time;
    double force_stability_threshold;
  } contact_params_;

  // 深度控制参数
  struct DepthParams {
    double target_depth;
    double real_target_depth;
    double stability_time;
    double depth_tolerance;
    struct {
      double kp, ki, kd;
    } control_gains;
  } depth_params_;

  // 控制参数
  struct ControlParams {
    int control_frequency;
    std::array<double, 3> position_kp;
    std::array<double, 3> position_kd;
  } control_params_;

  // 安全参数
  struct SafetyParams {
    double emergency_stop_force;
    struct {
      double x_min, x_max;
      double y_min, y_max;
      double z_min, z_max;
    } workspace_limits;
  } safety_params_;

  // 能量罐参数
  struct EnergyTankParams {
    std::array<double, 7> max_joint_torque;
    std::array<double, 7> max_joint_velocity;
    std::array<double, 7> torque_rate_limit;
  } energy_tank_params_;

  // 日志参数
  struct LoggingParams {
    bool enable;
    double frequency;
    std::string data_directory;
    std::string file_prefix;
  } logging_params_;

  // 力传感器参数
  struct ForceSensorParams {
    std::string serial_port;
    int baudrate;
    int sampling_frequency;
    int data_timeout;
  } force_sensor_params_;

  // 私有方法
  /**
   * @brief 加载配置参数
   */
  bool loadParameters(ros::NodeHandle& node_handle);

  /**
   * @brief 初始化力传感器
   */
  bool initializeForceSensor();

  /**
   * @brief 初始化日志系统
   */
  bool initializeLogging();

  /**
   * @brief 检查机器人是否在安全位置
   */
  bool checkSafetyPosition(const std::array<double, 7>& q);

  /**
   * @brief 调零阶段控制逻辑
   */
  void updateCalibrationPhase(const ros::Time& time, const ros::Duration& period);

  /**
   * @brief 接近阶段控制逻辑
   */
  void updateApproachPhase(const ros::Time& time, const ros::Duration& period);

  /**
   * @brief 接触确认阶段控制逻辑
   */
  void updateContactConfirmationPhase(const ros::Time& time, const ros::Duration& period);

  /**
   * @brief 深度控制阶段控制逻辑
   */
  void updateDepthControlPhase(const ros::Time& time, const ros::Duration& period);

  /**
   * @brief 轨迹运动阶段控制逻辑
   */
  void updateTrajectoryPhase(const ros::Time& time, const ros::Duration& period);

  /**
   * @brief 位置控制（参考cartesian_pose_example_controller）
   */
  void positionControl(const Eigen::Vector3d& target_pos, 
                      const ros::Duration& period,
                      std::array<double, 7>& tau_d);

  /**
   * @brief 应用关节力矩限制
   */
  std::array<double, 7> saturateTorqueRate(const std::array<double, 7>& tau_d_calculated,
                                          const std::array<double, 7>& tau_J_d);

  /**
   * @brief 检查工作空间限制
   */
  bool checkWorkspaceLimits(const Eigen::Vector3d& position);

  /**
   * @brief 阶段转换
   */
  void transitionToPhase(ControlPhase new_phase);

  /**
   * @brief 记录实验数据
   */
  void logExperimentData(const ros::Time& time);

  /**
   * @brief 打印阶段信息（限制频率到2Hz）
   */
  void printPhaseInfo(const std::string& message);

  /**
   * @brief 等待用户输入继续
   */
  void waitForUserInput();
}; 