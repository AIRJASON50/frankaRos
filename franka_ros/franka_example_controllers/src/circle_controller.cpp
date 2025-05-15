#include <franka_example_controllers/circle_controller.h>

#include <cmath>
#include <memory>
#include <chrono>
#include <ctime>
#include <random>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

// 在停止控制器时关闭日志文件
void CircleController::stopping(const ros::Time& time) {
  // 记录结束时间并关闭日志文件
  log_generator_.closeLogFile(time);
}

bool CircleController::init(hardware_interface::RobotHW* robot_hw,
                           ros::NodeHandle& node_handle) {
  // 获取机器人ID参数
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CircleController: Could not read parameter arm_id");
    return false;
  }
  
  // 获取关节名称参数
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CircleController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  // 初始化轨迹生成器
  trajectory_generator_ = std::make_unique<TrajectoryGenerator>();
  
  // 读取轨迹参数
  node_handle.param<double>("circle_frequency", circle_frequency_, 0.3);
  node_handle.param<double>("circle_radius", circle_radius_, 0.1);
  node_handle.param<std::string>("circle_plane", circle_plane_, "xy");
  ROS_INFO_STREAM("CircleController: Circle parameters: frequency=" << circle_frequency_ 
                 << "Hz, radius=" << circle_radius_ << "m, plane=" << circle_plane_);
  
  // 初始化路径发布器
  path_pub_ = node_handle.advertise<nav_msgs::Path>("trajectory_path", 1);
  
  // 初始化轨迹生成器
  if (!trajectory_generator_->init(node_handle, path_pub_)) {
    ROS_ERROR_STREAM("CircleController: Failed to initialize trajectory generator");
    return false;
  }
  
  // 读取软接触模型参数
  ros::NodeHandle soft_contact_nh(node_handle, "contact_model");
  contact_params_.young_modulus = 1000.0;  // 默认值：1kPa
  contact_params_.poisson_ratio = 0.45;    // 默认值：近似不可压缩
  contact_params_.friction_coef = 0.3;     // 默认值：中等摩擦系数
  contact_params_.contact_radius = 0.01;   // 默认值：1cm半径的探头
  contact_params_.path_radius = 0.1;       // 默认值：10cm圆周路径半径
  contact_params_.damping = 50.0;          // 默认值：中等阻尼
  contact_params_.force_threshold = 3.0;   // 默认值：3N接触力阈值
  contact_params_.depth_threshold = 0.001; // 默认值：1mm接触深度阈值
  
  soft_contact_nh.getParam("young_modulus", contact_params_.young_modulus);
  soft_contact_nh.getParam("poisson_ratio", contact_params_.poisson_ratio);
  soft_contact_nh.getParam("friction_coef", contact_params_.friction_coef);
  soft_contact_nh.getParam("contact_radius", contact_params_.contact_radius);
  soft_contact_nh.getParam("path_radius", contact_params_.path_radius);
  soft_contact_nh.getParam("damping", contact_params_.damping);
  soft_contact_nh.getParam("force_threshold", contact_params_.force_threshold);
  soft_contact_nh.getParam("depth_threshold", contact_params_.depth_threshold);
  
  // 读取软体块位置
  node_handle.param<double>("soft_block_x", soft_block_x_, 0.4);
  node_handle.param<double>("soft_block_y", soft_block_y_, 0.0);
  node_handle.param<double>("soft_block_z", soft_block_z_, 0.505);
  
  soft_block_position_ = Eigen::Vector3d(soft_block_x_, soft_block_y_, soft_block_z_);
  ROS_INFO_STREAM("CircleController: Soft block position: " << soft_block_position_.transpose());
  
  // 读取力控制参数
  ros::NodeHandle force_nh(node_handle, "force_controller");
  force_nh.getParam("target_force", target_force_);
  base_force_ = target_force_; // 初始化力轨迹基准
  force_nh.getParam("kp", force_p_gain_);
  force_nh.getParam("ki", force_i_gain_);
  force_nh.getParam("kd", force_d_gain_);
  force_nh.getParam("max_force", max_force_);
  
  // 同步力控制参数副本
  force_kp_ = force_p_gain_;
  force_ki_ = force_i_gain_;
  force_kd_ = force_d_gain_;
  
  // 读取相位持续时间
  node_handle.getParam("phase_duration", phase_duration_);
  
  // 初始化软接触模型
  contact_model_ = std::make_unique<franka_example_controllers::SoftContactModel>(
      node_handle, contact_params_);
  
  // 初始化发布器
  pose_pub_ = node_handle.advertise<geometry_msgs::PoseStamped>("equilibrium_pose", 10);
  contact_pub_ = node_handle.advertise<geometry_msgs::WrenchStamped>("contact_force", 10);
  phase_pub_ = node_handle.advertise<std_msgs::String>("control_phase", 10);

  // 获取机器人模型接口
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CircleController: Error getting model interface from hardware");
    return false;
  }
  
  // 尝试获取模型句柄
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CircleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // 获取机器人状态接口
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CircleController: Error getting state interface from hardware");
    return false;
  }
  
  // 尝试获取状态句柄
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CircleController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // 获取关节力矩控制接口
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CircleController: Error getting effort joint interface from hardware");
    return false;
  }
  
  // 获取每个关节的控制句柄
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CircleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // 设置笛卡尔阻抗控制参数（在开环控制中仅用于计算虚拟力矩）
  double translational_stiffness = 600.0;  // 位置刚度提高到600.0 N/m (原100.0)
  double rotational_stiffness = 50.0;      // 姿态刚度提高到50.0 Nm/rad (原20.0)
  
  cartesian_stiffness_.setZero();
  cartesian_stiffness_.topLeftCorner(3, 3)
      << translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_.bottomRightCorner(3, 3)
      << rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_.setZero();
  cartesian_damping_.topLeftCorner(3, 3)
      << 2.0 * sqrt(translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_ = 0.5;  // 零空间刚度：0.5

  // 初始化日志文件
  log_generator_.initLogFile(contact_params_, target_force_);

  // 初始化力轨迹生成器
  force_generator_ = std::make_unique<ForceGenerator>();
  if (!force_generator_->init(node_handle)) {
    ROS_ERROR_STREAM("CircleController: Failed to initialize force generator");
    return false;
  }
  
  // 保存噪声参数以供其他地方使用
  force_nh.getParam("force_noise_enable", force_noise_enable_);

  return true;
}

void CircleController::starting(const ros::Time& time) {
  // 获取初始状态
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // 如果中心点未指定，则使用初始位置作为中心点
  if (circle_center_.norm() < 1e-6) {
  circle_center_ = initial_transform.translation();
    ROS_INFO_STREAM("CircleController: Using initial position as circle center: " 
                    << circle_center_.transpose());
  } else {
    ROS_INFO_STREAM("CircleController: Using specified circle center: " 
                    << circle_center_.transpose());
  }
  
  // 设置初始目标位置和方向
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  
  // 设置零空间平衡配置为初始关节角度
  q_d_nullspace_ = q_initial;
  
  // 重置时间
  elapsed_time_ = 0.0;
  
  // 重置控制阶段
  control_phase_ = APPROACH;
  phase_start_time_ = time;
  
  // 重置力控制变量
  force_error_integral_ = 0.0;
  prev_force_error_ = 0.0;
  last_force_error_ = 0.0;
  
  // 重置接触状态
  contact_detected_ = false;
  contact_position_ = Eigen::Vector3d::Zero();
  
  // 发布初始控制阶段
  std_msgs::String phase_msg;
  phase_msg.data = "APPROACH";
  phase_pub_.publish(phase_msg);
  
  // 输出初始设置信息
  ROS_INFO_STREAM("CircleController: Starting with circle plane: " << circle_plane_ 
                  << ", radius: " << circle_radius_ 
                  << ", frequency: " << circle_frequency_);
  ROS_INFO_STREAM("CircleController: Initial position: " << position_d_.transpose());
  ROS_INFO_STREAM("CircleController: Soft block position: " << soft_block_position_.transpose());
  
  // 重置计数器
  circular_counter_ = 0;
  
  // 重置姿态初始化标志
  desired_pose_initialized_ = false;
}

void CircleController::update(const ros::Time& time,
                             const ros::Duration& period) {
  // 累计经过的时间（用于计算圆周轨迹）
  elapsed_time_ += period.toSec();
  
  // 动态更新目标力
  target_force_ = force_generator_->generateForceProfile(elapsed_time_);
  
  // 更新轨迹路径可视化 - 每10个周期更新一次，平衡性能和可视化效果
  if (circular_counter_ % 10 == 0) {
    trajectory_generator_->publishTrajectoryPath(elapsed_time_, soft_block_position_(2));
  }
  circular_counter_++;
  
  // =====================================================================
  // 1. 获取当前状态：传感器读数
  // =====================================================================
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 49> mass_array = model_handle_->getMass();

  // 将数组转换为Eigen向量/矩阵
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());  // 科里奥利力向量
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());  // 雅可比矩阵：映射关节空间到笛卡尔空间
  Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data());          // 质量矩阵
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());          // 当前关节位置
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());        // 当前关节速度
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(                          // 上一个周期的期望关节力矩
      robot_state.tau_J_d.data());

  // 获取当前末端位姿
  Eigen::Affine3d current_pose(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));  // 当前末端位姿变换矩阵
  Eigen::Vector3d position(current_pose.translation());                           // 当前末端位置
  Eigen::Quaterniond orientation(current_pose.rotation());                        // 当前末端姿态（四元数表示）

  // 计算末端速度
  Eigen::VectorXd v_ee = jacobian * dq;  // 笛卡尔空间速度
  Eigen::Vector3d position_d_dot = Eigen::Vector3d::Zero();  // 期望位置导数（速度）
  Eigen::Vector3d position_d_ddot = Eigen::Vector3d::Zero();  // 期望位置二阶导数（加速度）

  // 从机器人状态获取外部力估计
  Eigen::Map<const Eigen::Matrix<double, 6, 1>> external_wrench(robot_state.O_F_ext_hat_K.data());
  Eigen::Vector3d measured_force = external_wrench.head(3);  // 仅使用力部分
  double force_magnitude = measured_force.norm();

  // =====================================================================
  // 2. 第一个周期：初始化期望姿态（只执行一次）
  // =====================================================================
  if (!desired_pose_initialized_) {
    // 记录当前姿态作为期望姿态（仅需记录一次）
    desired_pose_ = current_pose;
    
    // 确保z轴指向地面（探头朝下）
    Eigen::Vector3d z_axis = desired_pose_.rotation().col(2);  // 提取旋转矩阵的第3列，即z轴方向
    if (z_axis(2) > 0) {
      // 如果z轴向上（z分量为正），旋转180度使其向下
      Eigen::Matrix3d rotation_matrix = desired_pose_.rotation();
      // 创建180度绕x轴的旋转（使z轴翻转）
      Eigen::Matrix3d flip_rotation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
      // 应用旋转
      rotation_matrix = flip_rotation * rotation_matrix;
      // 更新期望姿态的旋转部分
      desired_pose_.linear() = rotation_matrix;
    }
    
    // 保存probe坐标系z轴作为圆周运动的法线（垂直于圆周平面）
    circle_normal_ = desired_pose_.rotation().col(2).normalized();
    
    // 计算圆周运动平面的基底向量（与法线垂直的两个单位向量）
    // 步骤1: 找到与z轴不平行的参考向量
    Eigen::Vector3d reference = (std::abs(circle_normal_(0)) < 0.9) ? 
        Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    // 步骤2: x平面向量 = 法线与参考向量的叉积（垂直于两者）
    circle_x_axis_ = circle_normal_.cross(reference).normalized();
    // 步骤3: y平面向量 = 法线与x平面向量的叉积（垂直于两者，形成右手系）
    circle_y_axis_ = circle_normal_.cross(circle_x_axis_).normalized();
    
    // 设置轨迹生成器的中心点和轴向
    trajectory_generator_->setCenter(circle_center_);
    trajectory_generator_->setAxis(circle_x_axis_, circle_y_axis_);
    
    // 记录初始关节角度（用于零空间控制）
    initial_q_ = q;
    
    // 初始位置作为起始点
    position_d_ = position;
    orientation_d_ = Eigen::Quaterniond(desired_pose_.rotation());
    
    // 初始化完成标记
    desired_pose_initialized_ = true;
    
    // 输出初始化信息
    ROS_INFO_STREAM("Desired pose initialized: \n" << desired_pose_.matrix());
    ROS_INFO_STREAM("Circle normal (z-axis): [" << circle_normal_.transpose() << "]");
    ROS_INFO_STREAM("Circle x-axis: [" << circle_x_axis_.transpose() << "]");
    ROS_INFO_STREAM("Circle y-axis: [" << circle_y_axis_.transpose() << "]");
    
    // 重置时间计数，确保从0开始
    elapsed_time_ = 0.0;
    return;
  }

  // =====================================================================
  // 3. 运动阶段控制
  // =====================================================================
  
  // 转换笛卡尔空间质量矩阵
  Eigen::Matrix3d mass_matrix_cart;
  mass_matrix_cart = (jacobian.topLeftCorner(3, 7) * mass.inverse() * jacobian.topLeftCorner(3, 7).transpose()).inverse();
  
  // 计算当前位置相对于软体块的深度并强制更新
  double depth = soft_block_position_(2) - position(2);
  bool in_contact = depth > contact_params_.depth_threshold * 0.5;
  
  // 直接使用位置和外部力估计更新接触状态
  ContactState current_contact_state = contact_model_->updateContact(
      position,
      v_ee.head(3),
      orientation,
      soft_block_position_(2)
  );
  
  // 如果在CONTACT或TRAJECTORY阶段，强制设置接触状态
  if (control_phase_ == CONTACT || control_phase_ == TRAJECTORY) {
    // 强制设置接触状态为true
    current_contact_state.setContactState(true);
    
    // 直接使用测量的力
    if (measured_force.norm() > 0.1) {
      // 使用z方向的力（垂直力）
      double z_force = std::abs(measured_force(2));
        current_contact_state.setForce(z_force);
      
      // 打印调试信息
      static int debug_count = 0;
      if (debug_count++ % 1000 == 0) {
        ROS_INFO_STREAM("Force update: measured force=" << measured_force.norm() 
                      << "N, z-direction force=" << measured_force(2) << "N, depth=" 
                      << depth * 1000 << "mm");
      }
    } else {
      // 如果测量力太小，使用最小阈值
      current_contact_state.setForce(0.5);
    }
  }
  
  // 发布接触状态
  geometry_msgs::WrenchStamped contact_wrench_msg;
  contact_wrench_msg.header.stamp = time;
  contact_wrench_msg.header.frame_id = "world";
  
  if (current_contact_state.in_contact) {
    // 已检测到接触，调整力
    // 计算力误差
    double force_error = target_force_ - current_contact_state.contact_force;
    
    // 积分项（限制积分饱和）
    force_error_integral_ += force_error * period.toSec();
    force_error_integral_ = std::max(std::min(force_error_integral_, 5.0), -5.0);
    
    // 微分项
    double force_error_derivative = (force_error - prev_force_error_) / period.toSec();
    prev_force_error_ = force_error;
    
    // 力控制PID输出（负号是因为位置增加会减小力）
    double position_adjustment = -(force_p_gain_ * force_error + 
                                 force_i_gain_ * force_error_integral_ + 
                                 force_d_gain_ * force_error_derivative);
    
    // 限制位置调整范围
    position_adjustment = std::max(std::min(position_adjustment, 0.001), -0.001);  // 限制在±1mm
    
    // 仅调整z方向位置（沿法线方向）
    position_d_[2] += position_adjustment;
    
    // 若启用力噪音，给position_d_叠加三维扰动
    if (force_noise_enable_) {
      Eigen::Vector3d noise = force_generator_->generateForceNoise() * 0.001; // 转为最大2mm扰动
      position_d_ += noise;
    }
    
    if (circular_counter_ % 1000 == 0) {
      ROS_INFO_STREAM("Force control: current force " << current_contact_state.contact_force << "N, target force " 
                    << target_force_ << "N, adjustment " << position_adjustment * 1000 << "mm");
      ROS_INFO_STREAM("Contact depth: " << current_contact_state.depth() * 1000 << "mm, position: ["
                    << position.transpose() << "], target position: [" << position_d_.transpose() << "]");
      ROS_INFO_STREAM("Contact state: " << (current_contact_state.in_contact ? "in contact" : "not in contact")
                    << ", force vector: [" << current_contact_state.force().transpose() << "]");
    }
  } else {
    // 未接触时，使用微小力以确保有力反馈
    // 这种方式更符合实际物理系统，力传感器始终会有一些噪声和偏移
    if (circular_counter_ % 1000 == 0) {
      ROS_INFO_STREAM("Not in contact, distance to soft surface: " 
                    << (position(2) - soft_block_position_(2)) * 1000 << "mm");
    }
  }
  
  // 使用测量的力填充力反馈消息
  contact_wrench_msg.wrench.force.x = measured_force(0);
  contact_wrench_msg.wrench.force.y = measured_force(1);
  contact_wrench_msg.wrench.force.z = measured_force(2);
  
  // 设置力矩为零
  contact_wrench_msg.wrench.torque.x = 0.0;
  contact_wrench_msg.wrench.torque.y = 0.0;
  contact_wrench_msg.wrench.torque.z = 0.0;
  
  // 发布接触力信息
  contact_pub_.publish(contact_wrench_msg);
  
  // 检查接触状态切换
  // 在接近阶段，增加接触检测日志
  if (control_phase_ == APPROACH) {
    static int approach_counter = 0;
    if (approach_counter++ % 1000 == 0) {  // 约1s打印一次
      ROS_INFO_STREAM("Approach phase: current height " << position[2] << "m, soft block surface height " << soft_block_position_[2] 
                     << "m, distance " << (position[2] - soft_block_position_[2]) * 1000 << "mm");
      ROS_INFO_STREAM("Contact state: " << (current_contact_state.in_contact ? "in contact" : "not in contact")
                    << ", contact force: " << current_contact_state.contact_force << "N");
    }
    
    // 在接近阶段，更新机械臂期望位置（向下移动）
    // 计算每次更新的向下移动增量
    double approach_speed = 0.02; // 每秒下降2cm
    double delta_z = -approach_speed * period.toSec(); // 向下移动的距离增量
    
    // 更新期望位置
    position_d_ = position; // 保持当前XY位置
    position_d_[2] += delta_z; // 向下移动Z位置
    
    // 限制最低高度，避免过度碰撞
    if (position_d_[2] < soft_block_position_[2] - 0.01) { // 限制最大深入深度为1cm
      position_d_[2] = soft_block_position_[2] - 0.01;
    }
    
    // 接触检测 - 使用更宽松的条件
    bool is_contact_detected = false;
    
    // 主要条件: 软接触模型检测到接触
    if (current_contact_state.in_contact) {
        is_contact_detected = true;
      ROS_INFO_STREAM("Contact detection: soft contact model detected contact!");
    }
    
    // 后备条件1: 位置已经接近或超过软块表面
    if ((soft_block_position_[2] - position[2]) > contact_params_.depth_threshold * 0.1) {
      is_contact_detected = true;
      ROS_INFO_STREAM("Contact detection: position is close to soft block surface! Distance: " << 
                     (soft_block_position_[2] - position[2]) * 1000 << " mm");
    }
    
    // 后备条件2: 检测到任何外部力
    if (measured_force.norm() > 0.5) {
      is_contact_detected = true;
      ROS_INFO_STREAM("Contact detection: external force detected! Force magnitude: " << measured_force.norm() << " N");
    }
    
    // 强制接触条件: 如果机械臂下降到接近软块3mm以内，强制判定为接触
    if ((position[2] - soft_block_position_[2]) < 0.003) {
      is_contact_detected = true;
      ROS_INFO_STREAM("Contact detection: forced contact state! Current height: " << 
                     position[2] << "m, soft block height: " << soft_block_position_[2] << "m");
    }
    
    // 强制接触计时器: 如果在接近阶段持续太久，强制切换到接触阶段
    static ros::Time approach_start_time = time;
    if ((time - approach_start_time).toSec() > 10.0) { // 10秒后强制切换
      is_contact_detected = true;
      ROS_INFO_STREAM("Contact detection: approach phase timeout, forced to enter contact phase!");
    }
    
    if (is_contact_detected) {
      // 从接近阶段进入接触阶段
      control_phase_ = CONTACT;
      phase_start_time_ = time;
      contact_position_ = position;
      
      // 发布状态变化
      std_msgs::String phase_msg;
      phase_msg.data = "CONTACT";
      phase_pub_.publish(phase_msg);
      
      ROS_INFO_STREAM("===== Entering contact phase! =====");
      ROS_INFO_STREAM("Contact position: [" << contact_position_.transpose() << "]");
      ROS_INFO_STREAM("Contact force magnitude: " << current_contact_state.contact_force << " N");
      ROS_INFO_STREAM("Contact depth: " << current_contact_state.depth() * 1000 << " mm");
      ROS_INFO_STREAM("Measured force: [" << measured_force.transpose() << "] N");
    }
  }
  else if (control_phase_ == CONTACT) {
    // 检查是否满足切换条件
    double contact_duration = (time - phase_start_time_).toSec();
    
    // 接触阶段需要保持力控制
    // 设置期望位置，持续稍微向下移动直到达到目标力
    position_d_ = position;
    
    // 力控制 - 逐渐增加下压力直到达到目标力
    if (!current_contact_state.in_contact || current_contact_state.contact_force < target_force_) {
      // 如果未检测到接触或力不足，继续向下移动
      double force_diff = target_force_ - (current_contact_state.in_contact ? 
                                         current_contact_state.contact_force : 0.0);
      // 力差越大，移动越快
      double delta_z = -std::min(force_diff * 0.0001, 0.001); // 限制最大增量为1mm
      position_d_[2] += delta_z;
      
      // 限制最低位置，避免过度碰撞
      if (position_d_[2] < soft_block_position_[2] - 0.02) { // 限制最大深入深度为2cm
        position_d_[2] = soft_block_position_[2] - 0.02;
      }
    }
    
    // 添加更详细的调试信息
    static int debug_counter = 0;
    if (debug_counter++ % 1000 == 0) {  // 约1s打印一次
      ROS_INFO_STREAM("Currently in contact phase: duration " << contact_duration << " seconds, need to continue " 
                     << phase_duration_ << " seconds before switching to circular motion");
      ROS_INFO_STREAM("Contact state: " << (current_contact_state.in_contact ? "in contact" : "not in contact")
                     << ", contact force: " << current_contact_state.contact_force << " N"
                     << ", target force: " << target_force_ << " N");
      ROS_INFO_STREAM("Current position: [" << position.transpose() << "], desired position: [" 
                     << position_d_.transpose() << "]");
    }
    
    // 当接触持续时间超过设定阈值时切换到圆周运动
    if (contact_duration > phase_duration_) {
      // 从接触阶段进入圆周运动阶段
      control_phase_ = TRAJECTORY;
      phase_start_time_ = time;
      
      // 设置圆周运动中心为接触点
      circle_center_ = position;
      
      // 重置时间，以便圆周运动从0开始
      elapsed_time_ = 0.0;
      
      // 发布状态变化
      std_msgs::String phase_msg;
      phase_msg.data = "TRAJECTORY";
      phase_pub_.publish(phase_msg);
      
      ROS_INFO_STREAM("===== Entering trajectory phase! =====");
      ROS_INFO_STREAM("Circle center position: [" << circle_center_.transpose() << "]");
      ROS_INFO_STREAM("Circle radius: " << circle_radius_ << " m");
      ROS_INFO_STREAM("Circle frequency: " << circle_frequency_ << " Hz");
    }
  }
  else if (control_phase_ == TRAJECTORY) {
    // 轨迹运动阶段
    if (circular_counter_ % 1000 == 0) {
      ROS_INFO_STREAM("Trajectory motion phase: duration " << elapsed_time_ << " seconds");
      ROS_INFO_STREAM("Current trajectory: " << trajectory_generator_->getTrajectoryTypeStr());
      ROS_INFO_STREAM("Robot position: [" << position.transpose() << "]");
    }
    
    // 生成当前轨迹点 - 添加前馈控制
    // 计算期望位置、速度和加速度
    
    // 使用轨迹生成器获取当前时间对应的期望位置
    Eigen::Vector3d trajectory_position = trajectory_generator_->generateTrajectory(elapsed_time_);
    
    // 计算轨迹目标速度和加速度 - 使用更高精度的中心差分
    double dt = 0.001; // 1毫秒的时间步长
    Eigen::Vector3d trajectory_position_prev = trajectory_generator_->generateTrajectory(elapsed_time_ - dt);
    Eigen::Vector3d trajectory_position_next = trajectory_generator_->generateTrajectory(elapsed_time_ + dt);
    
    // 更精确的速度计算 - 使用中心差分
    position_d_dot = (trajectory_position_next - trajectory_position_prev) / (2.0 * dt);
    
    // 更精确的加速度计算 - 使用中心差分
    position_d_ddot = (trajectory_position_next - 2.0 * trajectory_position + trajectory_position_prev) / (dt * dt);
    
    // 更新期望位置的XY坐标（Z坐标由力控制决定）
    position_d_[0] = trajectory_position[0];
    position_d_[1] = trajectory_position[1];
    
    // 应用前馈控制补偿物理延迟 - 基于速度和加速度的预测修正
    // 这将使控制器更快地响应轨迹变化
    double feed_forward_time = 0.01; // 10ms的前馈预测
    position_d_[0] += position_d_dot(0) * feed_forward_time + 0.5 * position_d_ddot(0) * feed_forward_time * feed_forward_time;
    position_d_[1] += position_d_dot(1) * feed_forward_time + 0.5 * position_d_ddot(1) * feed_forward_time * feed_forward_time;
    
    // 力控制 - 保持恒定接触力
    // 通过软接触模型计算接触力并调整轨迹
    if (!current_contact_state.in_contact) {
      // 如果失去接触，轻微向下移动以重新建立接触
      position_d_[2] -= 0.0005; // 每次向下移动0.5mm
      
      // 限制最低高度，避免过度碰撞
      if (position_d_[2] < soft_block_position_[2] - 0.02) { // 限制最大深入深度为2cm
        position_d_[2] = soft_block_position_[2] - 0.02;
      }
      
      if (circular_counter_ % 1000 == 0) {
        ROS_WARN_STREAM("Lost contact during trajectory motion, attempting to move down to re-establish contact");
      }
    } else {
      // 已检测到接触，调整力
      // 计算力误差
      double force_error = target_force_ - current_contact_state.contact_force;
      
      // 积分项（限制积分饱和）
      force_error_integral_ += force_error * period.toSec();
      force_error_integral_ = std::max(std::min(force_error_integral_, 5.0), -5.0);
      
      // 微分项
      double force_error_derivative = (force_error - prev_force_error_) / period.toSec();
      prev_force_error_ = force_error;
      
      // 力控制PID输出（负号是因为位置增加会减小力）
      double position_adjustment = -(force_p_gain_ * force_error + 
                                   force_i_gain_ * force_error_integral_ + 
                                   force_d_gain_ * force_error_derivative);
      
      // 限制位置调整范围
      position_adjustment = std::max(std::min(position_adjustment, 0.001), -0.001);  // 限制在±1mm
      
      // 仅调整z方向位置（沿法线方向）
      position_d_[2] += position_adjustment;
      
      // 若启用力噪音，给position_d_叠加三维扰动
      if (force_noise_enable_) {
        Eigen::Vector3d noise = force_generator_->generateForceNoise() * 0.001; // 转为最大2mm扰动
        position_d_ += noise;
      }
      
      if (circular_counter_ % 1000 == 0) {
        ROS_INFO_STREAM("Force control: current force " << current_contact_state.contact_force << "N, target force " 
                      << target_force_ << "N, adjustment " << position_adjustment * 1000 << "mm");
      }
    }
  }
  
  // 发布期望位姿
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = time;
  pose_msg.header.frame_id = "world";
  pose_msg.pose.position.x = position_d_(0);
  pose_msg.pose.position.y = position_d_(1);
  pose_msg.pose.position.z = position_d_(2);
  pose_msg.pose.orientation.x = orientation_d_.x();
  pose_msg.pose.orientation.y = orientation_d_.y();
  pose_msg.pose.orientation.z = orientation_d_.z();
  pose_msg.pose.orientation.w = orientation_d_.w();
  pose_pub_.publish(pose_msg);

  // =====================================================================
  // 5. 闭环控制计算
  // =====================================================================
  try {
    // 计算位置误差向量
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d_;  // 位置误差
    
    // 计算姿态误差
    // 确保四元数方向一致（避免绕远路旋转）
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    
    // 计算姿态误差四元数（表示从当前姿态到目标姿态的旋转）
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    
    // 提取误差四元数的虚部（表示旋转轴*sin(角度/2)）
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    
    // 将姿态误差转换到基坐标系
    error.tail(3) << -current_pose.rotation() * error.tail(3);
    
    // 限制误差幅度以避免控制不稳定
    for (int i = 0; i < 6; i++) {
      error(i) = std::max(std::min(error(i), 0.2), -0.2);
    }
    
    // 计算速度误差
    Eigen::Matrix<double, 6, 1> vel_error;
    vel_error.head(3) << v_ee.head(3) - position_d_dot;  // 线速度误差
    vel_error.tail(3) << v_ee.tail(3);  // 角速度误差（目标角速度为0）
    
    // 笛卡尔空间PD控制 (P: 刚度, D: 阻尼)
    Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

    // 计算任务空间力矩：-K*x - D*dx (K: 刚度, D: 阻尼, x: 位置误差, dx: 速度)
    tau_task << jacobian.transpose() *
                (-cartesian_stiffness_ * error - cartesian_damping_ * vel_error);
    
    // 零空间控制：保持初始的关节配置
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    // 零空间控制：使用阻尼比为1的PD控制保持初始关节角度
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                      (nullspace_stiffness_ * (initial_q_ - q) -
                          (2.0 * sqrt(nullspace_stiffness_)) * dq);
    
    // 最终控制输入：任务空间力 + 零空间力 + 重力补偿
    tau_d << tau_task + tau_nullspace + coriolis;
    
    // 限制力矩变化率，避免不连续性
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    
    // 应用控制输入
    for (size_t i = 0; i < 7; ++i) {
      joint_handles_[i].setCommand(tau_d(i));
    }
    
    // 监控输出（仅用于调试）
    static int motion_counter = 0;
    if (motion_counter++ % 1000 == 0) {  // 约1s打印一次
      ROS_INFO_STREAM("=== Motion Control Status ===");
      ROS_INFO_STREAM("Control phase: " << (control_phase_ == APPROACH ? "Approach" : 
                                    (control_phase_ == CONTACT ? "Contact" : "Trajectory")));
      ROS_INFO_STREAM("Current position: [" << position.transpose() << "]");
      ROS_INFO_STREAM("Desired position: [" << position_d_.transpose() << "]");
      ROS_INFO_STREAM("Position error: [" << error.head(3).transpose() << "]");
      ROS_INFO_STREAM("Position error norm: " << error.head(3).norm());
      
      ROS_INFO_STREAM("Orientation error: [" << error.tail(3).transpose() << "]");
      ROS_INFO_STREAM("Orientation error norm: " << error.tail(3).norm());
      
      if (current_contact_state.in_contact) {
        ROS_INFO_STREAM("Contact state: In contact");
        ROS_INFO_STREAM("Contact position: [" << current_contact_state.position().transpose() << "]");
        ROS_INFO_STREAM("Measured force: [" << measured_force.transpose() << "]");
        ROS_INFO_STREAM("Measured force magnitude: " << measured_force.norm() << " N");
        ROS_INFO_STREAM("Contact depth: " << current_contact_state.depth() * 1000 << " mm");
        ROS_INFO_STREAM("Target force: " << target_force_ << " N");
        ROS_INFO_STREAM("Force error: " << target_force_ - (measured_force.norm() > 0.1 ? measured_force.norm() : 0.0) << " N");
      } else {
        ROS_INFO_STREAM("Contact state: Not in contact");
        ROS_INFO_STREAM("Distance to soft block: " << (position(2) - soft_block_position_(2)) << " m");
      }
      
      if (control_phase_ == TRAJECTORY) {
        double angle = 2.0 * M_PI * circle_frequency_ * elapsed_time_;
        ROS_INFO_STREAM("Circle angle: " << fmod(angle, 2.0 * M_PI) << " rad");
      }
      
      ROS_INFO_STREAM("Elapsed time: " << elapsed_time_ << " s");
      ROS_INFO_STREAM("===============================");
    }
  } catch (const std::exception& ex) {
    // 捕获并记录任何控制过程中的异常
    ROS_ERROR_STREAM("Exception during control process: " << ex.what());
  }

  // 在接触状态或圆周运动阶段记录数据
  // 将此代码放在每个控制周期的最后，确保获取到最新数据
  if (measured_force.norm() > 0.1) {
    // 使用测量的力进行记录
    log_generator_.logData(time, position, measured_force, static_cast<int>(control_phase_),
                          soft_block_position_(2), target_force_);
  } else {
    // 如果测量力太小，提供一个默认的小值进行记录
    Eigen::Vector3d default_force(0, 0, 0.05);
    log_generator_.logData(time, position, default_force, static_cast<int>(control_phase_),
                          soft_block_position_(2), target_force_);
  }
}

/**
 * @brief 限制力矩变化率，避免不连续性
 * 
 * @param tau_d_calculated 计算得到的期望力矩
 * @param tau_J_d 上一周期的期望力矩
 * @return Eigen::Matrix<double, 7, 1> 限制变化率后的力矩
 */
Eigen::Matrix<double, 7, 1> CircleController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    // 计算力矩变化量
    double difference = tau_d_calculated[i] - tau_J_d[i];
    // 限制变化量在 ±delta_tau_max_ 范围内
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CircleController,
                       controller_interface::ControllerBase)
