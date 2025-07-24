// Copyright (c) 2024 Franka DS Project
// Use of this source code is governed by the Apache-2.0 license
#pragma once

#include <Eigen/Dense>
#include <string>
#include <memory>
#include <vector>

namespace franka_ds {

/**
 * @brief DS原语基类，统一DS计算接口
 * 
 * 所有DS实现都继承此基类，确保接口一致性
 */
class DSPrimitive {
 public:
  virtual ~DSPrimitive() = default;
  
  /**
   * @brief 计算DS速度输出
   * @param current_position 当前位置
   * @param target_position 目标位置
   * @return 期望速度向量
   */
  virtual Eigen::Vector3d computeVelocity(const Eigen::Vector3d& current_position,
                                          const Eigen::Vector3d& target_position) = 0;
  
  /**
   * @brief 判断是否收敛到目标
   * @param current_position 当前位置
   * @param target_position 目标位置
   * @param tolerance 收敛容差
   * @return 是否已收敛
   */
  virtual bool isConverged(const Eigen::Vector3d& current_position,
                          const Eigen::Vector3d& target_position,
                          double tolerance = 0.01) const = 0;
  
  /**
   * @brief 获取DS原语名称
   */
  virtual std::string getName() const = 0;
  
  /**
   * @brief 设置最大速度限制
   */
  virtual void setMaxVelocity(double max_vel) { max_velocity_ = max_vel; }
  
  /**
   * @brief 获取最大速度限制
   */
  double getMaxVelocity() const { return max_velocity_; }

 protected:
  double max_velocity_ = 0.1;  // 默认最大速度 10cm/s
  
  /**
   * @brief 应用速度限制
   */
  Eigen::Vector3d limitVelocity(const Eigen::Vector3d& velocity) const;
};

/**
 * @brief 线性DS原语
 * 
 * 实现线性收敛：f(x) = -λ(x - x*)
 * 用于自由空间的点到点运动
 */
class LinearDS : public DSPrimitive {
 public:
  /**
   * @brief 构造函数
   * @param lambda 收敛率矩阵
   * @param max_vel 最大速度
   */
  LinearDS(const Eigen::Matrix3d& lambda, double max_vel = 0.05);
  
  /**
   * @brief 构造函数（标量收敛率）
   * @param lambda_scalar 标量收敛率（对角矩阵）
   * @param max_vel 最大速度
   */
  LinearDS(double lambda_scalar, double max_vel = 0.05);
  
  Eigen::Vector3d computeVelocity(const Eigen::Vector3d& current_position,
                                 const Eigen::Vector3d& target_position) override;
  
  bool isConverged(const Eigen::Vector3d& current_position,
                  const Eigen::Vector3d& target_position,
                  double tolerance = 0.01) const override;
  
  std::string getName() const override { return "LinearDS"; }
  
  /**
   * @brief 设置收敛率矩阵
   */
  void setLambda(const Eigen::Matrix3d& lambda) { lambda_ = lambda; }
  
  /**
   * @brief 获取收敛率矩阵
   */
  const Eigen::Matrix3d& getLambda() const { return lambda_; }

 private:
  Eigen::Matrix3d lambda_;  // 收敛率矩阵
};

/**
 * @brief 圆周DS原语
 * 
 * 实现圆周运动：参考SurfacePolishing.cpp的getCircularMotionVelocity
 * 用于接触空间的抛光运动
 */
class CircularDS : public DSPrimitive {
 public:
  /**
   * @brief 构造函数
   * @param radius 圆周半径
   * @param omega 角速度
   * @param max_vel 最大速度
   */
  CircularDS(double radius, double omega, double max_vel = 0.1);
  
  Eigen::Vector3d computeVelocity(const Eigen::Vector3d& current_position,
                                 const Eigen::Vector3d& center_position) override;
  
  bool isConverged(const Eigen::Vector3d& current_position,
                  const Eigen::Vector3d& center_position,
                  double tolerance = 0.01) const override;
  
  std::string getName() const override { return "CircularDS"; }
  
  /**
   * @brief 设置圆周参数
   */
  void setCircularParams(double radius, double omega);
  
  /**
   * @brief 获取圆周半径
   */
  double getRadius() const { return radius_; }
  
  /**
   * @brief 获取角速度
   */
  double getOmega() const { return angular_velocity_; }

 private:
  double radius_;           // 圆周半径 [m]
  double angular_velocity_; // 角速度 [rad/s]
};

/**
 * @brief 姿态控制DS
 * 
 * 基于四元数误差的姿态控制
 * 用于维持末端工具姿态
 */
class OrientationDS {
 public:
  /**
   * @brief 构造函数
   * @param stiffness 姿态刚度
   */
  explicit OrientationDS(double stiffness = 5.0);
  
  /**
   * @brief 计算角速度命令
   * @param current_quat 当前四元数 [w,x,y,z]
   * @param desired_quat 期望四元数 [w,x,y,z]
   * @return 角速度向量 [rad/s]
   */
  Eigen::Vector3d computeAngularVelocity(const Eigen::Vector4d& current_quat,
                                        const Eigen::Vector4d& desired_quat);
  
  /**
   * @brief 判断姿态是否收敛
   * @param current_quat 当前四元数
   * @param desired_quat 期望四元数
   * @param tolerance 角度容差 [rad]
   * @return 是否已收敛
   */
  bool isOrientationConverged(const Eigen::Vector4d& current_quat,
                             const Eigen::Vector4d& desired_quat,
                             double tolerance = 0.1) const;
  
  /**
   * @brief 设置姿态刚度
   */
  void setStiffness(double stiffness) { stiffness_ = stiffness; }
  
  /**
   * @brief 获取姿态刚度
   */
  double getStiffness() const { return stiffness_; }

 private:
  double stiffness_;  // 姿态刚度
  double max_angular_velocity_ = 0.5;  // 最大角速度 [rad/s]
};

/**
 * @brief 组合DS原语
 * 
 * 用于混合多个DS，实现复杂运动模式
 * 例如：水平接近 + 垂直下探
 */
class CompositeDS : public DSPrimitive {
 public:
  CompositeDS() = default;
  
  /**
   * @brief 添加DS组件
   * @param ds DS原语
   * @param weight 权重
   */
  void addComponent(std::shared_ptr<DSPrimitive> ds, double weight = 1.0);
  
  /**
   * @brief 设置组件权重
   * @param index 组件索引
   * @param weight 新权重
   */
  void setComponentWeight(size_t index, double weight);
  
  Eigen::Vector3d computeVelocity(const Eigen::Vector3d& current_position,
                                 const Eigen::Vector3d& target_position) override;
  
  bool isConverged(const Eigen::Vector3d& current_position,
                  const Eigen::Vector3d& target_position,
                  double tolerance = 0.01) const override;
  
  std::string getName() const override { return "CompositeDS"; }
  
  /**
   * @brief 清空所有组件
   */
  void clearComponents();
  
  /**
   * @brief 获取组件数量
   */
  size_t getComponentCount() const { return components_.size(); }

 private:
  struct DSComponent {
    std::shared_ptr<DSPrimitive> ds;
    double weight;
  };
  
  std::vector<DSComponent> components_;
};

}  // namespace franka_ds 