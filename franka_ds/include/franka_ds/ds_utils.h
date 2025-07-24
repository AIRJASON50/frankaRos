// Copyright (c) 2024 Franka DS Project
// Use of this source code is governed by the Apache-2.0 license
#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>

namespace franka_ds {

/**
 * @brief DS工具类，提供通用数学函数和转换工具
 * 
 * 参考DS开源代码的Utils.h模式，整合常用功能：
 * - 四元数操作
 * - 向量运算
 * - 平滑函数
 * - 类型转换
 */
class DSUtils {
 public:
  // ========== 四元数操作工具 ==========
  
  /**
   * @brief 四元数转旋转矩阵
   * @param quat 四元数 [w,x,y,z]
   * @return 3x3旋转矩阵
   */
  static Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Vector4d& quat);
  
  /**
   * @brief 旋转矩阵转四元数
   * @param rotation_matrix 3x3旋转矩阵
   * @return 四元数 [w,x,y,z]
   */
  static Eigen::Vector4d rotationMatrixToQuaternion(const Eigen::Matrix3d& rotation_matrix);
  
  /**
   * @brief 四元数球面线性插值 (SLERP)
   * @param q1 起始四元数
   * @param q2 结束四元数
   * @param t 插值参数 [0,1]
   * @return 插值后的四元数
   */
  static Eigen::Vector4d slerpQuaternion(const Eigen::Vector4d& q1, 
                                        const Eigen::Vector4d& q2, 
                                        double t);
  
  /**
   * @brief 四元数乘法
   * @param q1 四元数1
   * @param q2 四元数2
   * @return 乘积四元数
   */
  static Eigen::Vector4d quaternionProduct(const Eigen::Vector4d& q1, 
                                          const Eigen::Vector4d& q2);
  
  /**
   * @brief 四元数共轭（逆）
   * @param quat 四元数
   * @return 共轭四元数
   */
  static Eigen::Vector4d quaternionInverse(const Eigen::Vector4d& quat);
  
  /**
   * @brief 四元数转轴角表示
   * @param quat 四元数
   * @param axis 输出轴向量
   * @param angle 输出角度
   */
  static void quaternionToAxisAngle(const Eigen::Vector4d& quat, 
                                   Eigen::Vector3d& axis, 
                                   double& angle);
  
  // ========== 平滑函数 ==========
  
  /**
   * @brief 平滑衰减函数（参考Utils::smoothFall）
   * @param x 输入值
   * @param a 起始点
   * @param b 结束点
   * @return 平滑衰减值 [0,1]
   */
  static double smoothFall(double x, double a, double b);
  
  /**
   * @brief 平滑上升函数
   * @param x 输入值
   * @param a 起始点
   * @param b 结束点
   * @return 平滑上升值 [0,1]
   */
  static double smoothRise(double x, double a, double b);
  
  /**
   * @brief Sigmoid函数
   * @param x 输入值
   * @param center 中心点
   * @param slope 斜率
   * @return Sigmoid值 [0,1]
   */
  static double sigmoid(double x, double center = 0.0, double slope = 1.0);
  
  // ========== 向量操作 ==========
  
  /**
   * @brief 向量长度限制
   * @param vector 输入向量
   * @param max_norm 最大长度
   * @return 限制后的向量
   */
  static Eigen::Vector3d clampVelocity(const Eigen::Vector3d& vector, double max_norm);
  
  /**
   * @brief 向量归一化（安全版本）
   * @param vector 输入向量
   * @param min_norm 最小长度阈值
   * @return 归一化向量，如果长度过小则返回零向量
   */
  static Eigen::Vector3d safeNormalize(const Eigen::Vector3d& vector, double min_norm = 1e-6);
  
  /**
   * @brief 向量投影
   * @param vector 被投影向量
   * @param direction 投影方向
   * @return 投影向量
   */
  static Eigen::Vector3d projectVector(const Eigen::Vector3d& vector, 
                                      const Eigen::Vector3d& direction);
  
  /**
   * @brief 向量拒绝（垂直分量）
   * @param vector 输入向量
   * @param direction 拒绝方向
   * @return 垂直分量
   */
  static Eigen::Vector3d rejectVector(const Eigen::Vector3d& vector, 
                                     const Eigen::Vector3d& direction);
  
  // ========== 类型转换 ==========
  
  /**
   * @brief Eigen向量转std::array（Franka命令格式）
   * @param eigen_vector Eigen向量
   * @return std::array<double, 6>格式
   */
  static std::array<double, 6> eigenToArray6D(const Eigen::Vector3d& linear_vel,
                                             const Eigen::Vector3d& angular_vel);
  
  /**
   * @brief Eigen位置向量转std::array
   * @param position Eigen位置向量
   * @return std::array<double, 3>格式
   */
  static std::array<double, 3> eigenToArray3D(const Eigen::Vector3d& position);
  
  /**
   * @brief std::array转Eigen向量
   * @param array std::array输入
   * @return Eigen向量
   */
  static Eigen::Vector3d arrayToEigen3D(const std::array<double, 3>& array);
  
  // ========== 数学常量和函数 ==========
  
  /**
   * @brief 角度转弧度
   */
  static double deg2rad(double degrees) { return degrees * M_PI / 180.0; }
  
  /**
   * @brief 弧度转角度
   */
  static double rad2deg(double radians) { return radians * 180.0 / M_PI; }
  
  /**
   * @brief 安全除法
   * @param numerator 分子
   * @param denominator 分母
   * @param default_value 分母为零时的默认值
   * @return 除法结果
   */
  static double safeDivision(double numerator, double denominator, double default_value = 0.0);
  
  /**
   * @brief 值约束到区间
   * @param value 输入值
   * @param min_val 最小值
   * @param max_val 最大值
   * @return 约束后的值
   */
  static double clamp(double value, double min_val, double max_val);
  
  /**
   * @brief 向量约束到区间
   * @param vector 输入向量
   * @param min_val 最小值
   * @param max_val 最大值
   * @return 约束后的向量
   */
  static Eigen::Vector3d clampVector(const Eigen::Vector3d& vector, 
                                    double min_val, double max_val);
  
  // ========== 几何计算 ==========
  
  /**
   * @brief 计算点到直线的距离
   * @param point 点
   * @param line_start 直线起点
   * @param line_end 直线终点
   * @return 距离
   */
  static double pointToLineDistance(const Eigen::Vector3d& point,
                                   const Eigen::Vector3d& line_start,
                                   const Eigen::Vector3d& line_end);
  
  /**
   * @brief 计算点到平面的距离
   * @param point 点
   * @param plane_point 平面上一点
   * @param plane_normal 平面法向量
   * @return 距离（有符号）
   */
  static double pointToPlaneDistance(const Eigen::Vector3d& point,
                                    const Eigen::Vector3d& plane_point,
                                    const Eigen::Vector3d& plane_normal);
  
  // ========== 调试工具 ==========
  
  /**
   * @brief 打印Eigen向量（调试用）
   * @param vector 向量
   * @param label 标签
   */
  static void printVector(const Eigen::Vector3d& vector, const std::string& label = "Vector");
  
  /**
   * @brief 打印四元数（调试用）
   * @param quat 四元数
   * @param label 标签
   */
  static void printQuaternion(const Eigen::Vector4d& quat, const std::string& label = "Quaternion");
  
  /**
   * @brief 检查向量是否有效（无NaN或Inf）
   * @param vector 输入向量
   * @return 是否有效
   */
  static bool isVectorValid(const Eigen::Vector3d& vector);
  
  /**
   * @brief 检查四元数是否有效
   * @param quat 四元数
   * @return 是否有效
   */
  static bool isQuaternionValid(const Eigen::Vector4d& quat);

 private:
  // 静态工具类，禁止实例化
  DSUtils() = delete;
  ~DSUtils() = delete;
  DSUtils(const DSUtils&) = delete;
  DSUtils& operator=(const DSUtils&) = delete;
  
  // 数学常量
  static constexpr double EPSILON = 1e-12;
  static constexpr double DEFAULT_QUATERNION_TOLERANCE = 1e-6;
};

}  // namespace franka_ds 