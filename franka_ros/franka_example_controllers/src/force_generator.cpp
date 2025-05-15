// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/force_generator.h>

#include <cmath>
#include <ros/ros.h>

namespace franka_example_controllers {

ForceGenerator::ForceGenerator()
    : force_profile_type_(FORCE_CONSTANT),
      force_profile_type_str_("constant"),
      base_force_(10.0),
      force_sine_amplitude_(0.0),
      force_sine_frequency_(1.0),
      force_step_time_(5.0),
      force_step_value_(0.0),
      force_noise_enable_(false),
      force_noise_min_(-2.0),
      force_noise_max_(2.0),
      gen_(rd_()),
      noise_distribution_(force_noise_min_, force_noise_max_) {
}

bool ForceGenerator::init(ros::NodeHandle& node_handle) {
  // 创建力控制参数命名空间
  ros::NodeHandle force_nh(node_handle, "force_controller");
  
  // 读取基准力
  if (!force_nh.getParam("target_force", base_force_)) {
    ROS_INFO_STREAM("ForceGenerator: Using default base force: " << base_force_ << "N");
  } else {
    ROS_INFO_STREAM("ForceGenerator: Base force set to: " << base_force_ << "N");
  }

  // 读取力轨迹类型参数
  std::string force_profile_type_str;
  if (force_nh.getParam("force_profile_type", force_profile_type_str)) {
    force_profile_type_str_ = force_profile_type_str;
    if (force_profile_type_str == "constant") {
      force_profile_type_ = FORCE_CONSTANT;
      ROS_INFO_STREAM("ForceGenerator: Using constant force profile");
    } else if (force_profile_type_str == "sine") {
      force_profile_type_ = FORCE_SINE;
      ROS_INFO_STREAM("ForceGenerator: Using sine force profile");
    } else if (force_profile_type_str == "step") {
      force_profile_type_ = FORCE_STEP;
      ROS_INFO_STREAM("ForceGenerator: Using step force profile");
    } else {
      ROS_WARN_STREAM("ForceGenerator: Unknown force_profile_type: " << force_profile_type_str 
                      << ", using constant.");
      force_profile_type_ = FORCE_CONSTANT;
      force_profile_type_str_ = "constant";
    }
  } else {
    ROS_INFO_STREAM("ForceGenerator: No force profile type specified. Using default constant force profile.");
    force_profile_type_ = FORCE_CONSTANT;
    force_profile_type_str_ = "constant";
  }

  // 读取正弦力轨迹参数
  if (!force_nh.getParam("force_sine_amplitude", force_sine_amplitude_)) {
    ROS_INFO_STREAM("ForceGenerator: Using default sine amplitude: " << force_sine_amplitude_ << "N");
  }
  if (!force_nh.getParam("force_sine_frequency", force_sine_frequency_)) {
    ROS_INFO_STREAM("ForceGenerator: Using default sine frequency: " << force_sine_frequency_ << "Hz");
  }

  // 读取阶跃力轨迹参数
  if (!force_nh.getParam("force_step_time", force_step_time_)) {
    ROS_INFO_STREAM("ForceGenerator: Using default step time: " << force_step_time_ << "s");
  }
  if (!force_nh.getParam("force_step_value", force_step_value_)) {
    ROS_INFO_STREAM("ForceGenerator: Using default step value: " << force_step_value_ << "N");
  }

  // 读取力噪音参数
  if (!force_nh.getParam("force_noise_enable", force_noise_enable_)) {
    ROS_INFO_STREAM("ForceGenerator: Force noise is " << (force_noise_enable_ ? "enabled" : "disabled"));
  }
  if (!force_nh.getParam("force_noise_min", force_noise_min_)) {
    ROS_INFO_STREAM("ForceGenerator: Using default force noise min: " << force_noise_min_ << "N");
  }
  if (!force_nh.getParam("force_noise_max", force_noise_max_)) {
    ROS_INFO_STREAM("ForceGenerator: Using default force noise max: " << force_noise_max_ << "N");
  }

  // 更新噪声分布
  noise_distribution_ = std::uniform_real_distribution<>(force_noise_min_, force_noise_max_);

  return true;
}

double ForceGenerator::generateForceProfile(double time) {
  double force = 0.0;
  switch (force_profile_type_) {
    case FORCE_CONSTANT:
      force = base_force_;
      break;
    case FORCE_SINE:
      force = base_force_ + force_sine_amplitude_ * std::sin(2 * M_PI * force_sine_frequency_ * time);
      break;
    case FORCE_STEP:
      force = (time < force_step_time_) ? base_force_ : force_step_value_;
      break;
    default:
      force = base_force_;
  }
  
  // 添加力噪声（如果启用）
  if (force_noise_enable_) {
    force += generateForceNoise().norm();
  }
  
  return force;
}

Eigen::Vector3d ForceGenerator::generateForceNoise() {
  return Eigen::Vector3d(
      noise_distribution_(gen_),
      noise_distribution_(gen_),
      noise_distribution_(gen_));
}

void ForceGenerator::setBaseForce(double base_force) {
  base_force_ = base_force;
}

ForceProfileType ForceGenerator::getForceProfileType() const {
  return force_profile_type_;
}

std::string ForceGenerator::getForceProfileTypeStr() const {
  return force_profile_type_str_;
}

void ForceGenerator::enableForceNoise(bool enable) {
  force_noise_enable_ = enable;
}

}  // namespace franka_example_controllers 