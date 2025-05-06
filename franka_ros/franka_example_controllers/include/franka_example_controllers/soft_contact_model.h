#pragma once

#include <array>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>

namespace franka_example_controllers {

/**
 * @brief 接触参数结构体，定义软接触模型的物理特性
 */
struct ContactParams {
  double young_modulus;     // 材料杨氏模量 (Pa)
  double poisson_ratio;     // 泊松比 (无量纲)
  double friction_coef;     // 摩擦系数 (无量纲)
  double contact_radius;    // 接触探头半径 (m)
  double path_radius;       // 路径圆半径 (m)
  double damping;           // 阻尼系数 (Ns/m)
  double force_threshold;   // 接触力阈值 (N)
  double depth_threshold;   // 接触深度阈值 (m)
  
  // 兼容旧代码的别名
  double& E = young_modulus;
  double& mu = friction_coef;
  double& nu = poisson_ratio;
  double& R = contact_radius;
  double& R_path = path_radius;
  double& Kd = damping;
};

/**
 * @brief 接触状态结构体，保存当前接触信息
 */
struct ContactState {
  bool in_contact;                   // 是否处于接触状态
  double contact_force;              // 接触力大小 (N)
  double contact_depth;              // 接触深度 (m)
  double effective_stiffness;        // 有效刚度 (N/m)
  Eigen::Vector3d contact_normal;    // 接触法向量
  Eigen::Vector3d contact_position;  // 接触点位置
  Eigen::Vector3d friction_force;    // 摩擦力向量
  
  // 默认构造函数
  ContactState() : in_contact(false), contact_force(0.0), contact_depth(0.0), 
                  effective_stiffness(0.0) {
    contact_normal = Eigen::Vector3d(0, 0, 1);
    contact_position = Eigen::Vector3d::Zero();
    friction_force = Eigen::Vector3d::Zero();
  }
  
  // 兼容旧代码的方法（使用getter而不是引用）
  double depth() const { return contact_depth; }
  const Eigen::Vector3d& position() const { return contact_position; }
  const Eigen::Vector3d& normal() const { return contact_normal; }
  
  // 向后兼容的方法，获取力向量
  Eigen::Vector3d force() const {
    return contact_normal * contact_force;
  }
  
  // 兼容的索引运算符，返回力向量的分量
  double force(int idx) const {
    Eigen::Vector3d f = force();
    return f(idx);
  }
  
  // 添加可以设置力的方法
  void setForce(double force) {
    contact_force = force;
  }
  
  // 设置接触状态
  void setContactState(bool contact) {
    in_contact = contact;
  }
};

/**
 * @brief 软接触模型类，实现Hertz接触理论
 * 
 * 该类提供了基于Hertz接触理论的软接触力学模型，支持计算接触力、摩擦力、
 * 接触状态检测以及可视化接触信息。
 */
class SoftContactModel {
public:
  /**
   * @brief 构造一个软接触模型
   * 
   * @param node_handle ROS节点句柄，用于发布接触信息
   * @param params 接触参数结构体
   */
  SoftContactModel(ros::NodeHandle& node_handle, const ContactParams& params);
  
  /**
   * @brief 更新接触状态
   * 
   * @param position 末端执行器位置
   * @param velocity 末端执行器速度
   * @param orientation 末端执行器姿态四元数
   * @param soft_surface_height 软体表面高度
   * @return 更新后的接触状态
   */
  ContactState updateContact(
      const Eigen::Vector3d& position,
      const Eigen::Vector3d& velocity,
      const Eigen::Quaterniond& orientation,
      double soft_surface_height);
      
  /**
   * @brief 计算接触力
   * 
   * @param depth 接触深度
   * @param normal 接触法向量
   * @return 接触力向量
   */
  Eigen::Vector3d computeContactForce(double depth, const Eigen::Vector3d& normal);
  
  /**
   * @brief 计算摩擦力
   * 
   * @param normal_force 法向力大小
   * @param tangential_velocity 切向速度向量
   * @return 摩擦力向量
   */
  Eigen::Vector3d computeFrictionForce(
      double normal_force, 
      const Eigen::Vector3d& tangential_velocity);
  
  /**
   * @brief 发布接触状态可视化标记
   * 
   * @param state 当前接触状态
   */
  void publishContactMarkers(const ContactState& state);
  
  /**
   * @brief 发布接触力消息
   * 
   * @param force 接触力向量
   * @param contact_point 接触点位置
   */
  void publishContactForce(const Eigen::Vector3d& force, const Eigen::Vector3d& contact_point);
  
  /**
   * @brief 计算有效弹性模量
   * 
   * @return 有效弹性模量值
   */
  double computeEffectiveModulus() const;
  
  /**
   * @brief 计算接触刚度
   * 
   * @param depth 接触深度
   * @return 接触刚度值
   */
  double computeContactStiffness(double depth) const;
  
  /**
   * @brief 向后兼容方法 - 检查是否接触
   * 
   * @param position 末端执行器位置
   * @param surface_height 软体表面高度
   * @param depth 如果接触，输出接触深度
   * @return 是否处于接触状态
   */
  bool checkContact(const Eigen::Vector3d& position, 
                    double surface_height,
                    double& depth) {
    depth = surface_height - position(2);
    return (depth > params_.depth_threshold);
  }
  
  /**
   * @brief 向后兼容方法 - 更新接触状态
   * 
   * @param position 末端执行器位置
   * @param orientation 末端执行器姿态
   * @param velocity 末端执行器线速度
   * @param angular_velocity 末端执行器角速度
   * @param surface_height 接触表面高度
   * @return 更新后的接触状态
   */
  ContactState updateContactState(
      const Eigen::Vector3d& position,
      const Eigen::Quaterniond& orientation,
      const Eigen::Vector3d& velocity,
      const Eigen::Vector3d& angular_velocity,
      double surface_height) {
    return updateContact(position, velocity, orientation, surface_height);
  }
  
private:
  // 接触参数
  ContactParams params_;
  
  // ROS通信
  ros::NodeHandle& node_handle_;
  ros::Publisher marker_pub_;
  ros::Publisher wrench_pub_;
  
  // 软体块表面位置
  Eigen::Vector3d soft_block_position_;
  
  // 可视化标记
  visualization_msgs::MarkerArray markers_;
  
  // 接触状态消息
  geometry_msgs::WrenchStamped wrench_msg_;
  
  // 上一帧接触状态
  ContactState last_state_;
};

/**
 * @brief 接触状态可视化类，负责在RViz中显示接触信息
 */
class ContactVisualizer {
public:
  /**
   * @brief 构造函数
   * @param nh ROS节点句柄
   * @param frame_id 参考坐标系
   */
  ContactVisualizer(ros::NodeHandle& nh, const std::string& frame_id = "world");
  
  /**
   * @brief 发布接触标记
   * @param position 接触点位置
   * @param force 接触力
   * @param depth 接触深度
   */
  void publishContactMarker(const Eigen::Vector3d& position, 
                           const Eigen::Vector3d& force, 
                           double depth);
private:
  ros::Publisher marker_pub_;
  std::string frame_id_;
};

}  // namespace franka_example_controllers 