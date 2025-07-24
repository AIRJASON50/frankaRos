// Copyright (c) 2024 Franka DS Project
// Use of this source code is governed by the Apache-2.0 license

#include <franka_ds/ds_utils.h>
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace franka_ds {

// ========== 四元数操作工具 ==========

Eigen::Matrix3d DSUtils::quaternionToRotationMatrix(const Eigen::Vector4d& quat) {
  // 四元数 [w, x, y, z]
  const double w = quat(0);
  const double x = quat(1);
  const double y = quat(2);
  const double z = quat(3);
  
  // 归一化
  const double norm = quat.norm();
  if (norm < EPSILON) {
    return Eigen::Matrix3d::Identity();
  }
  
  const double w_n = w / norm;
  const double x_n = x / norm;
  const double y_n = y / norm;
  const double z_n = z / norm;
  
  // 构建旋转矩阵
  Eigen::Matrix3d R;
  R(0, 0) = 1 - 2 * (y_n * y_n + z_n * z_n);
  R(0, 1) = 2 * (x_n * y_n - w_n * z_n);
  R(0, 2) = 2 * (x_n * z_n + w_n * y_n);
  
  R(1, 0) = 2 * (x_n * y_n + w_n * z_n);
  R(1, 1) = 1 - 2 * (x_n * x_n + z_n * z_n);
  R(1, 2) = 2 * (y_n * z_n - w_n * x_n);
  
  R(2, 0) = 2 * (x_n * z_n - w_n * y_n);
  R(2, 1) = 2 * (y_n * z_n + w_n * x_n);
  R(2, 2) = 1 - 2 * (x_n * x_n + y_n * y_n);
  
  return R;
}

Eigen::Vector4d DSUtils::rotationMatrixToQuaternion(const Eigen::Matrix3d& R) {
  Eigen::Vector4d q;
  
  const double trace = R.trace();
  
  if (trace > 0) {
    const double s = sqrt(trace + 1.0) * 2; // s = 4 * qw
    q(0) = 0.25 * s;
    q(1) = (R(2, 1) - R(1, 2)) / s;
    q(2) = (R(0, 2) - R(2, 0)) / s;
    q(3) = (R(1, 0) - R(0, 1)) / s;
  } else if ((R(0, 0) > R(1, 1)) && (R(0, 0) > R(2, 2))) {
    const double s = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2; // s = 4 * qx
    q(0) = (R(2, 1) - R(1, 2)) / s;
    q(1) = 0.25 * s;
    q(2) = (R(0, 1) + R(1, 0)) / s;
    q(3) = (R(0, 2) + R(2, 0)) / s;
  } else if (R(1, 1) > R(2, 2)) {
    const double s = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2; // s = 4 * qy
    q(0) = (R(0, 2) - R(2, 0)) / s;
    q(1) = (R(0, 1) + R(1, 0)) / s;
    q(2) = 0.25 * s;
    q(3) = (R(1, 2) + R(2, 1)) / s;
  } else {
    const double s = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2; // s = 4 * qz
    q(0) = (R(1, 0) - R(0, 1)) / s;
    q(1) = (R(0, 2) + R(2, 0)) / s;
    q(2) = (R(1, 2) + R(2, 1)) / s;
    q(3) = 0.25 * s;
  }
  
  return q.normalized();
}

Eigen::Vector4d DSUtils::quaternionProduct(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2) {
  Eigen::Vector4d result;
  
  result(0) = q1(0) * q2(0) - q1(1) * q2(1) - q1(2) * q2(2) - q1(3) * q2(3);
  result(1) = q1(0) * q2(1) + q1(1) * q2(0) + q1(2) * q2(3) - q1(3) * q2(2);
  result(2) = q1(0) * q2(2) - q1(1) * q2(3) + q1(2) * q2(0) + q1(3) * q2(1);
  result(3) = q1(0) * q2(3) + q1(1) * q2(2) - q1(2) * q2(1) + q1(3) * q2(0);
  
  return result;
}

Eigen::Vector4d DSUtils::quaternionInverse(const Eigen::Vector4d& quat) {
  Eigen::Vector4d q_inv;
  const double norm_squared = quat.squaredNorm();
  
  if (norm_squared < EPSILON) {
    return Eigen::Vector4d(1, 0, 0, 0); // 单位四元数
  }
  
  q_inv(0) = quat(0) / norm_squared;
  q_inv(1) = -quat(1) / norm_squared;
  q_inv(2) = -quat(2) / norm_squared;
  q_inv(3) = -quat(3) / norm_squared;
  
  return q_inv;
}

void DSUtils::quaternionToAxisAngle(const Eigen::Vector4d& quat, Eigen::Vector3d& axis, double& angle) {
  const Eigen::Vector4d q_norm = quat.normalized();
  
  angle = 2.0 * acos(std::abs(q_norm(0)));
  
  const double sin_half_angle = sqrt(1.0 - q_norm(0) * q_norm(0));
  
  if (sin_half_angle < EPSILON) {
    // 接近零旋转
    axis = Eigen::Vector3d(1, 0, 0);
    angle = 0.0;
  } else {
    axis(0) = q_norm(1) / sin_half_angle;
    axis(1) = q_norm(2) / sin_half_angle;
    axis(2) = q_norm(3) / sin_half_angle;
    axis.normalize();
  }
}

// ========== 平滑函数 ==========

double DSUtils::smoothFall(double x, double a, double b) {
  if (x < a) return 1.0;
  if (x > b) return 0.0;
  return 0.5 * (1.0 + cos(M_PI * (x - a) / (b - a)));
}

double DSUtils::smoothRise(double x, double a, double b) {
  return 1.0 - smoothFall(x, a, b);
}

double DSUtils::sigmoid(double x, double center, double slope) {
  return 1.0 / (1.0 + exp(-slope * (x - center)));
}

// ========== 向量操作 ==========

Eigen::Vector3d DSUtils::clampVelocity(const Eigen::Vector3d& vector, double max_norm) {
  const double norm = vector.norm();
  if (norm > max_norm && norm > EPSILON) {
    return vector * (max_norm / norm);
  }
  return vector;
}

Eigen::Vector3d DSUtils::safeNormalize(const Eigen::Vector3d& vector, double min_norm) {
  const double norm = vector.norm();
  if (norm > min_norm) {
    return vector / norm;
  }
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d DSUtils::projectVector(const Eigen::Vector3d& vector, const Eigen::Vector3d& direction) {
  const Eigen::Vector3d dir_norm = safeNormalize(direction);
  return vector.dot(dir_norm) * dir_norm;
}

Eigen::Vector3d DSUtils::rejectVector(const Eigen::Vector3d& vector, const Eigen::Vector3d& direction) {
  return vector - projectVector(vector, direction);
}

// ========== 类型转换 ==========

std::array<double, 6> DSUtils::eigenToArray6D(const Eigen::Vector3d& linear_vel,
                                              const Eigen::Vector3d& angular_vel) {
  return {linear_vel(0), linear_vel(1), linear_vel(2),
          angular_vel(0), angular_vel(1), angular_vel(2)};
}

std::array<double, 3> DSUtils::eigenToArray3D(const Eigen::Vector3d& position) {
  return {position(0), position(1), position(2)};
}

Eigen::Vector3d DSUtils::arrayToEigen3D(const std::array<double, 3>& array) {
  return Eigen::Vector3d(array[0], array[1], array[2]);
}

// ========== 数学函数 ==========

double DSUtils::safeDivision(double numerator, double denominator, double default_value) {
  if (std::abs(denominator) < EPSILON) {
    return default_value;
  }
  return numerator / denominator;
}

double DSUtils::clamp(double value, double min_val, double max_val) {
  return std::max(min_val, std::min(max_val, value));
}

Eigen::Vector3d DSUtils::clampVector(const Eigen::Vector3d& vector, double min_val, double max_val) {
  return vector.cwiseMax(min_val).cwiseMin(max_val);
}

// ========== 几何计算 ==========

double DSUtils::pointToLineDistance(const Eigen::Vector3d& point,
                                   const Eigen::Vector3d& line_start,
                                   const Eigen::Vector3d& line_end) {
  const Eigen::Vector3d line_vec = line_end - line_start;
  const Eigen::Vector3d point_vec = point - line_start;
  
  const double line_length_sq = line_vec.squaredNorm();
  if (line_length_sq < EPSILON) {
    return (point - line_start).norm();
  }
  
  const double t = clamp(point_vec.dot(line_vec) / line_length_sq, 0.0, 1.0);
  const Eigen::Vector3d projection = line_start + t * line_vec;
  
  return (point - projection).norm();
}

double DSUtils::pointToPlaneDistance(const Eigen::Vector3d& point,
                                    const Eigen::Vector3d& plane_point,
                                    const Eigen::Vector3d& plane_normal) {
  const Eigen::Vector3d normal_unit = safeNormalize(plane_normal);
  return (point - plane_point).dot(normal_unit);
}

// ========== 调试工具 ==========

void DSUtils::printVector(const Eigen::Vector3d& vector, const std::string& label) {
  ROS_INFO("%s: [%.6f, %.6f, %.6f]", label.c_str(), vector(0), vector(1), vector(2));
}

void DSUtils::printQuaternion(const Eigen::Vector4d& quat, const std::string& label) {
  ROS_INFO("%s: [w=%.6f, x=%.6f, y=%.6f, z=%.6f]", 
           label.c_str(), quat(0), quat(1), quat(2), quat(3));
}

bool DSUtils::isVectorValid(const Eigen::Vector3d& vector) {
  return vector.allFinite();
}

bool DSUtils::isQuaternionValid(const Eigen::Vector4d& quat) {
  return quat.allFinite() && (quat.norm() > EPSILON);
}

}  // namespace franka_ds 