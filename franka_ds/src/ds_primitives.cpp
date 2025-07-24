// Copyright (c) 2024 Franka DS Project
// Use of this source code is governed by the Apache-2.0 license

#include <franka_ds/ds_primitives.h>
#include <franka_ds/ds_utils.h>
#include <ros/ros.h>
#include <cmath>

namespace franka_ds {

// ========== DSPrimitive基类实现 ==========

Eigen::Vector3d DSPrimitive::limitVelocity(const Eigen::Vector3d& velocity) const {
  return DSUtils::clampVelocity(velocity, max_velocity_);
}

// ========== LinearDS实现 ==========

LinearDS::LinearDS(const Eigen::Matrix3d& lambda, double max_vel) 
  : lambda_(lambda) {
  max_velocity_ = max_vel;
}

LinearDS::LinearDS(double lambda_scalar, double max_vel) 
  : lambda_(Eigen::Matrix3d::Identity() * lambda_scalar) {
  max_velocity_ = max_vel;
}

Eigen::Vector3d LinearDS::computeVelocity(const Eigen::Vector3d& current_position, 
                                     const Eigen::Vector3d& attractor_position) {
  // 计算当前位置到吸引子的向量
  Eigen::Vector3d error = attractor_position - current_position;
  
  // 如果已经非常接近吸引子，返回零速度
  if (error.norm() < 1e-6) {
    return Eigen::Vector3d::Zero();
  }
  
  // 计算速度向量 v = -lambda * (x - x_target)
  Eigen::Vector3d velocity = -lambda_ * error;
  
  // 限制速度大小
  double vel_norm = velocity.norm();
  if (vel_norm > max_velocity_) {
    velocity = velocity * (max_velocity_ / vel_norm);
  }
  
  // 确保速度不会太小
  const double min_velocity = 0.005; // 最小速度阈值，0.5cm/s
  if (vel_norm < min_velocity && vel_norm > 1e-6) {
    velocity = velocity.normalized() * min_velocity;
  }
  
  return velocity;
}

bool LinearDS::isConverged(const Eigen::Vector3d& current_position,
                          const Eigen::Vector3d& target_position,
                          double tolerance) const {
  const double distance = (target_position - current_position).norm();
  return distance < tolerance;
}

// ========== CircularDS实现 ==========

CircularDS::CircularDS(double radius, double omega, double max_vel)
  : radius_(radius), angular_velocity_(omega) {
  max_velocity_ = max_vel;
}

Eigen::Vector3d CircularDS::computeVelocity(const Eigen::Vector3d& current_position,
                                           const Eigen::Vector3d& center_position) {
  // 参考SurfacePolishing.cpp:605-620行的getCircularMotionVelocity实现
  
  const Eigen::Vector3d position_rel = current_position - center_position;
  
  // 计算当前半径和角度
  const double R = sqrt(position_rel.x() * position_rel.x() + position_rel.y() * position_rel.y());
  const double theta = atan2(position_rel.y(), position_rel.x());
  
  // 计算圆周运动速度
  Eigen::Vector3d velocity;
  
  // X方向速度：径向收敛 + 切向运动
  velocity.x() = -(R - radius_) * cos(theta) - R * angular_velocity_ * sin(theta);
  
  // Y方向速度：径向收敛 + 切向运动
  velocity.y() = -(R - radius_) * sin(theta) + R * angular_velocity_ * cos(theta);
  
  // Z方向速度：回到圆周平面
  velocity.z() = -position_rel.z();
  
  // 应用速度限制
  return limitVelocity(velocity);
}

bool CircularDS::isConverged(const Eigen::Vector3d& current_position,
                            const Eigen::Vector3d& center_position,
                            double tolerance) const {
  const Eigen::Vector3d pos_rel = current_position - center_position;
  const double current_radius = sqrt(pos_rel.x() * pos_rel.x() + pos_rel.y() * pos_rel.y());
  
  // 检查是否在目标圆周上且在平面内
  const bool radius_converged = std::abs(current_radius - radius_) < tolerance;
  const bool height_converged = std::abs(pos_rel.z()) < tolerance;
  
  return radius_converged && height_converged;
}

void CircularDS::setCircularParams(double radius, double omega) {
  radius_ = radius;
  angular_velocity_ = omega;
}

// ========== OrientationDS实现 ==========

OrientationDS::OrientationDS(double stiffness) : stiffness_(stiffness) {
}

Eigen::Vector3d OrientationDS::computeAngularVelocity(const Eigen::Vector4d& current_quat,
                                                     const Eigen::Vector4d& desired_quat) {
  // 基于四元数误差的姿态DS控制
  
  // 计算四元数误差
  const Eigen::Vector4d quat_inv = DSUtils::quaternionInverse(current_quat);
  const Eigen::Vector4d quat_error = DSUtils::quaternionProduct(quat_inv, desired_quat);
  
  // 转换为轴角表示
  Eigen::Vector3d axis;
  double angle;
  DSUtils::quaternionToAxisAngle(quat_error, axis, angle);
  
  // DS控制律：ω = -k * θ * n
  Eigen::Vector3d angular_velocity = -stiffness_ * angle * axis;
  
  // 应用角速度限制
  return DSUtils::clampVelocity(angular_velocity, max_angular_velocity_);
}

bool OrientationDS::isOrientationConverged(const Eigen::Vector4d& current_quat,
                                          const Eigen::Vector4d& desired_quat,
                                          double tolerance) const {
  // 计算四元数之间的角度差
  const Eigen::Vector4d quat_inv = DSUtils::quaternionInverse(current_quat);
  const Eigen::Vector4d quat_error = DSUtils::quaternionProduct(quat_inv, desired_quat);
  
  // 角度误差
  const double angle_error = 2.0 * std::abs(acos(std::min(1.0, std::abs(quat_error(0)))));
  
  return angle_error < tolerance;
}

// ========== CompositeDS实现 ==========

void CompositeDS::addComponent(std::shared_ptr<DSPrimitive> ds, double weight) {
  if (ds != nullptr && weight > 0.0) {
    components_.push_back({ds, weight});
  }
}

void CompositeDS::setComponentWeight(size_t index, double weight) {
  if (index < components_.size() && weight >= 0.0) {
    components_[index].weight = weight;
  }
}

Eigen::Vector3d CompositeDS::computeVelocity(const Eigen::Vector3d& current_position,
                                           const Eigen::Vector3d& target_position) {
  if (components_.empty()) {
    return Eigen::Vector3d::Zero();
  }
  
  Eigen::Vector3d weighted_velocity = Eigen::Vector3d::Zero();
  double total_weight = 0.0;
  
  // 计算加权平均
  for (const auto& component : components_) {
    if (component.weight > 0.0) {
      const Eigen::Vector3d vel = component.ds->computeVelocity(current_position, target_position);
      weighted_velocity += component.weight * vel;
      total_weight += component.weight;
    }
  }
  
  // 归一化权重
  if (total_weight > 0.0) {
    weighted_velocity /= total_weight;
  }
  
  // 应用速度限制
  return limitVelocity(weighted_velocity);
}

bool CompositeDS::isConverged(const Eigen::Vector3d& current_position,
                             const Eigen::Vector3d& target_position,
                             double tolerance) const {
  if (components_.empty()) {
    return true;
  }
  
  // 所有组件都必须收敛
  for (const auto& component : components_) {
    if (component.weight > 0.0) {
      if (!component.ds->isConverged(current_position, target_position, tolerance)) {
        return false;
      }
    }
  }
  
  return true;
}

void CompositeDS::clearComponents() {
  components_.clear();
}

}  // namespace franka_ds 