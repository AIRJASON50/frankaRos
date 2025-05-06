#include <franka_example_controllers/soft_contact_model.h>

#include <cmath>
#include <Eigen/Geometry>
#include <std_msgs/ColorRGBA.h>

namespace franka_example_controllers {

SoftContactModel::SoftContactModel(ros::NodeHandle& node_handle, const ContactParams& params)
    : node_handle_(node_handle), params_(params), last_state_() {
  
  // 初始化ROS发布者
  marker_pub_ = node_handle_.advertise<visualization_msgs::MarkerArray>(
      "contact_markers", 1);
  wrench_pub_ = node_handle_.advertise<geometry_msgs::WrenchStamped>(
      "contact_force", 1);
      
  // 初始化消息
  wrench_msg_.header.frame_id = "world";
  
  // 初始化可视化标记
  markers_.markers.resize(3);
  
  // 接触点标记
  auto& contact_marker = markers_.markers[0];
  contact_marker.header.frame_id = "world";
  contact_marker.ns = "contact";
  contact_marker.id = 0;
  contact_marker.type = visualization_msgs::Marker::SPHERE;
  contact_marker.action = visualization_msgs::Marker::ADD;
  contact_marker.scale.x = 0.02;
  contact_marker.scale.y = 0.02;
  contact_marker.scale.z = 0.02;
  contact_marker.color.r = 1.0;
  contact_marker.color.g = 0.0;
  contact_marker.color.b = 0.0;
  contact_marker.color.a = 1.0;
  
  // 接触力标记
  auto& force_marker = markers_.markers[1];
  force_marker.header.frame_id = "world";
  force_marker.ns = "force";
  force_marker.id = 1;
  force_marker.type = visualization_msgs::Marker::ARROW;
  force_marker.action = visualization_msgs::Marker::ADD;
  force_marker.scale.x = 0.01;  // 箭头柄直径
  force_marker.scale.y = 0.02;  // 箭头头部直径
  force_marker.color.r = 0.0;
  force_marker.color.g = 1.0;
  force_marker.color.b = 0.0;
  force_marker.color.a = 1.0;
  
  // 接触深度文本标记
  auto& text_marker = markers_.markers[2];
  text_marker.header.frame_id = "world";
  text_marker.ns = "depth_text";
  text_marker.id = 2;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.scale.z = 0.05;  // 文本高度
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;
  
  // 设置标记的生存时间为1秒
  for (auto& marker : markers_.markers) {
    marker.lifetime = ros::Duration(1.0);
  }
  
  ROS_INFO("SoftContactModel initialized with parameters:");
  ROS_INFO("  Young's modulus: %.2f Pa", params_.young_modulus);
  ROS_INFO("  Poisson ratio: %.2f", params_.poisson_ratio);
  ROS_INFO("  Friction coefficient: %.2f", params_.friction_coef);
  ROS_INFO("  Contact radius: %.3f m", params_.contact_radius);
  ROS_INFO("  Path radius: %.3f m", params_.path_radius);
  ROS_INFO("  Damping: %.2f Ns/m", params_.damping);
  ROS_INFO("  Force threshold: %.2f N", params_.force_threshold);
  ROS_INFO("  Depth threshold: %.3f m", params_.depth_threshold);
}

ContactState SoftContactModel::updateContact(
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity,
    const Eigen::Quaterniond& orientation,
    double soft_surface_height) {
  
  ContactState state;
  
  // 软体表面位置
  Eigen::Vector3d surface_point(position[0], position[1], soft_surface_height);
  
  // 计算接触深度（z方向）- 正值表示接触，负值表示无接触
  double depth = soft_surface_height - position[2];
  
  // 记录状态更新信息
  static int debug_counter = 0;
  
  // 判断是否接触（使用更宽松的条件以提高灵敏度）
  if (depth > params_.depth_threshold * 0.5) {
    // 确认接触状态
    state.in_contact = true;
    state.contact_depth = depth;
    state.contact_position = position;
    
    // 获取接触法向量 - 根据接触表面法线方向（默认垂直向上）
    state.contact_normal = Eigen::Vector3d(0, 0, 1);
    
    // 根据赫兹接触理论计算接触力
    Eigen::Vector3d normal_force = computeContactForce(depth, state.contact_normal);
    double normal_force_mag = normal_force.norm();
    state.contact_force = normal_force_mag;
    
    // 计算接触刚度（赫兹接触的非线性刚度）
    state.effective_stiffness = computeContactStiffness(depth);
    
    // 保证最小接触力 - 提高接触稳定性
    if (normal_force_mag < 0.1) {
      // 设置一个最小的接触力，确保接触力不为零
      double min_force = 0.1; 
      state.contact_force = min_force;
      normal_force = min_force * state.contact_normal;
    }
    
    // 计算切向速度 - 用于计算摩擦力
    Eigen::Vector3d tangential_velocity = velocity;
    tangential_velocity -= velocity.dot(state.contact_normal) * state.contact_normal;
    
    // 计算摩擦力 - 使用Coulomb模型
    state.friction_force = computeFrictionForce(normal_force_mag, tangential_velocity);
    
    // 每100次更新输出一次调试信息
    if (debug_counter++ % 100 == 0) {
      ROS_DEBUG_STREAM("接触深度: " << depth * 1000.0 << " mm, 接触力: " << normal_force_mag 
          << " N, 切向速度: " << tangential_velocity.norm() << " m/s");
    }
    
    // 发布接触信息 - 用于可视化
    publishContactMarkers(state);
    publishContactForce(normal_force + state.friction_force, state.contact_position);
  } else {
    // 未接触状态
    state.in_contact = false;
    state.contact_depth = 0.0;
    state.contact_force = 0.0;
    state.effective_stiffness = 0.0;
    state.contact_normal = Eigen::Vector3d(0, 0, 1);
    state.contact_position = position;
    state.friction_force = Eigen::Vector3d::Zero();
    
    // 每100次更新输出一次调试信息
    if (debug_counter++ % 100 == 0) {
      ROS_DEBUG_STREAM("未接触，距离表面: " << -depth * 1000.0 << " mm");
    }
  }
  
  // 保存状态，用于下次更新参考
  last_state_ = state;
  
  return state;
}

Eigen::Vector3d SoftContactModel::computeContactForce(double depth, const Eigen::Vector3d& normal) {
  if (depth <= 0) {
    return Eigen::Vector3d::Zero();
  }
  
  // 根据论文公式：F = (4/3) * E_effective * sqrt(R) * depth^(3/2)
  // E_effective = E / (1-v^2)，其中E是杨氏模量，v是泊松比
  double effective_modulus = params_.young_modulus / (1 - std::pow(params_.poisson_ratio, 2));
  
  // 计算接触面积半径
  // a = sqrt(R*d)，其中R是接触半径，d是接触深度
  double contact_radius = std::sqrt(params_.contact_radius * depth);
  
  // 计算弹性力
  double elastic_force = (4.0 / 3.0) * effective_modulus * 
                        std::sqrt(params_.contact_radius) * 
                        std::pow(depth, 1.5);
  
  // 添加阻尼项：Hunt-Crossley模型 (基于论文中的粘弹性模型)
  // F_damping = k_d * depth^n * velocity
  static Eigen::Vector3d last_normal = normal;
  static double last_depth = 0.0;
  static ros::Time last_time = ros::Time::now();
  
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  
  if (dt > 0) {
    // 估计接触点速度
    double depth_rate = (depth - last_depth) / dt;
    
    // 阻尼力 - 仅在挤压时应用(深度增加)
    double damping_force = 0;
    if (depth_rate > 0) {
      // 按照Hunt-Crossley模型，阻尼与深度的1.5次方成比例
      damping_force = params_.damping * std::pow(depth, 1.5) * depth_rate;
    }
    
    // 总力 = 弹性力 + 阻尼力
    double force_magnitude = elastic_force + damping_force;
    
    // 保存状态以计算下一次的速度
    last_depth = depth;
    last_normal = normal;
    last_time = current_time;
    
    // 力不应为负值（仅在挤压方向存在力）
    force_magnitude = std::max(0.0, force_magnitude);
    
    // 返回沿法线方向的力
    return force_magnitude * normal;
  } else {
    // 如果时间差为0，则只返回弹性力
    return elastic_force * normal;
  }
}

Eigen::Vector3d SoftContactModel::computeFrictionForce(
    double normal_force, 
    const Eigen::Vector3d& tangential_velocity) {
  
  // 切向速度的大小
  double vel_magnitude = tangential_velocity.norm();
  
  if (vel_magnitude < 1e-6) {
    // 切向速度太小，返回零摩擦力
    return Eigen::Vector3d::Zero();
  }
  
  // 计算库仑摩擦力
  double friction_magnitude = params_.friction_coef * normal_force;
  
  // 摩擦力方向与速度相反
  Eigen::Vector3d friction_direction = -tangential_velocity / vel_magnitude;
  
  return friction_magnitude * friction_direction;
}

void SoftContactModel::publishContactMarkers(const ContactState& state) {
  ros::Time now = ros::Time::now();
  
  // 更新所有标记的时间戳
  for (auto& marker : markers_.markers) {
    marker.header.stamp = now;
  }
  
  if (state.in_contact) {
    // 接触点标记
    auto& contact_marker = markers_.markers[0];
    contact_marker.pose.position.x = state.contact_position.x();
    contact_marker.pose.position.y = state.contact_position.y();
    contact_marker.pose.position.z = state.contact_position.z();
    
    // 接触力标记（箭头）
    auto& force_marker = markers_.markers[1];
    force_marker.points.resize(2);
    
    // 箭头起点（接触点）
    force_marker.points[0].x = state.contact_position.x();
    force_marker.points[0].y = state.contact_position.y();
    force_marker.points[0].z = state.contact_position.z();
    
    // 计算箭头终点（与接触力成比例）
    double arrow_scale = 0.02;  // 缩放因子，使箭头长度适合可视化
    Eigen::Vector3d force_end = state.contact_position + 
                               arrow_scale * state.contact_force * state.contact_normal;
    force_marker.points[1].x = force_end.x();
    force_marker.points[1].y = force_end.y();
    force_marker.points[1].z = force_end.z();
    
    // 接触深度文本标记
    auto& text_marker = markers_.markers[2];
    text_marker.pose.position.x = state.contact_position.x() + 0.1;
    text_marker.pose.position.y = state.contact_position.y();
    text_marker.pose.position.z = state.contact_position.z() + 0.1;
    
    char buffer[50];
    std::snprintf(buffer, sizeof(buffer), "Depth: %.3f mm\nForce: %.2f N", 
                 state.contact_depth * 1000.0, state.contact_force);
    text_marker.text = buffer;
    
    // 设置标记可见
    for (auto& marker : markers_.markers) {
      marker.action = visualization_msgs::Marker::ADD;
    }
  } else {
    // 未接触时隐藏标记
    for (auto& marker : markers_.markers) {
      marker.action = visualization_msgs::Marker::DELETE;
    }
  }
  
  // 发布标记
  marker_pub_.publish(markers_);
}

void SoftContactModel::publishContactForce(
    const Eigen::Vector3d& force, 
    const Eigen::Vector3d& contact_point) {
  
  wrench_msg_.header.stamp = ros::Time::now();
  
  // 填充力数据
  wrench_msg_.wrench.force.x = force.x();
  wrench_msg_.wrench.force.y = force.y();
  wrench_msg_.wrench.force.z = force.z();
  
  // 力矩设为零（简化模型）
  wrench_msg_.wrench.torque.x = 0.0;
  wrench_msg_.wrench.torque.y = 0.0;
  wrench_msg_.wrench.torque.z = 0.0;
  
  // 发布力消息
  wrench_pub_.publish(wrench_msg_);
}

double SoftContactModel::computeEffectiveModulus() const {
  // 有效弹性模量计算
  // E* = E / (1 - ν²)
  return params_.young_modulus / (1.0 - params_.poisson_ratio * params_.poisson_ratio);
}

double SoftContactModel::computeContactStiffness(double depth) const {
  if (depth <= 0) {
    return 0.0;
  }
  
  // 接触刚度是接触力对深度的导数
  // K = dF/dδ = 2 * E* * sqrt(R) * sqrt(δ)
  double effective_modulus = computeEffectiveModulus();
  return 2.0 * effective_modulus * std::sqrt(params_.contact_radius) * std::sqrt(depth);
}

// 实现ContactVisualizer类的方法
ContactVisualizer::ContactVisualizer(ros::NodeHandle& nh, const std::string& frame_id)
    : frame_id_(frame_id) {
  marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("contact_markers", 1);
}

void ContactVisualizer::publishContactMarker(const Eigen::Vector3d& position, 
                                           const Eigen::Vector3d& force, 
                                           double depth) {
  // 创建标记数组
  visualization_msgs::MarkerArray markers;
  markers.markers.resize(3);
  
  // 接触点标记
  auto& contact_marker = markers.markers[0];
  contact_marker.header.frame_id = frame_id_;
  contact_marker.header.stamp = ros::Time::now();
  contact_marker.ns = "contact";
  contact_marker.id = 0;
  contact_marker.type = visualization_msgs::Marker::SPHERE;
  contact_marker.action = visualization_msgs::Marker::ADD;
  contact_marker.pose.position.x = position.x();
  contact_marker.pose.position.y = position.y();
  contact_marker.pose.position.z = position.z();
  contact_marker.scale.x = 0.02;
  contact_marker.scale.y = 0.02;
  contact_marker.scale.z = 0.02;
  contact_marker.color.r = 1.0;
  contact_marker.color.g = 0.0;
  contact_marker.color.b = 0.0;
  contact_marker.color.a = 1.0;
  
  // 接触力标记（箭头）
  auto& force_marker = markers.markers[1];
  force_marker.header.frame_id = frame_id_;
  force_marker.header.stamp = ros::Time::now();
  force_marker.ns = "force";
  force_marker.id = 1;
  force_marker.type = visualization_msgs::Marker::ARROW;
  force_marker.action = visualization_msgs::Marker::ADD;
  force_marker.points.resize(2);
  
  // 箭头起点
  force_marker.points[0].x = position.x();
  force_marker.points[0].y = position.y();
  force_marker.points[0].z = position.z();
  
  // 箭头终点
  double arrow_scale = 0.02;  // 缩放因子
  Eigen::Vector3d force_end = position + arrow_scale * force;
  force_marker.points[1].x = force_end.x();
  force_marker.points[1].y = force_end.y();
  force_marker.points[1].z = force_end.z();
  
  force_marker.scale.x = 0.01;  // 箭头柄直径
  force_marker.scale.y = 0.02;  // 箭头头部直径
  force_marker.color.r = 0.0;
  force_marker.color.g = 1.0;
  force_marker.color.b = 0.0;
  force_marker.color.a = 1.0;
  
  // 接触深度文本标记
  auto& text_marker = markers.markers[2];
  text_marker.header.frame_id = frame_id_;
  text_marker.header.stamp = ros::Time::now();
  text_marker.ns = "depth_text";
  text_marker.id = 2;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.pose.position.x = position.x() + 0.1;
  text_marker.pose.position.y = position.y();
  text_marker.pose.position.z = position.z() + 0.1;
  text_marker.scale.z = 0.05;  // 文本高度
  
  char buffer[50];
  std::snprintf(buffer, sizeof(buffer), "Depth: %.3f mm\nForce: %.2f N", 
               depth * 1000.0, force.norm());
  text_marker.text = buffer;
  
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;
  
  // 发布标记
  marker_pub_.publish(markers);
}

}  // namespace franka_example_controllers