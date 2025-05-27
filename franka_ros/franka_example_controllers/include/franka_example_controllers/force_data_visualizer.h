#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace franka_example_controllers {

/**
 * @brief 力数据可视化类，负责rqt力曲线绘制
 */
class ForceDataVisualizer {
public:
  /**
   * @brief 构造函数
   * @param nh ROS节点句柄
   */
  ForceDataVisualizer(ros::NodeHandle& nh) : nh_(nh) {
    // 创建用于rqt绘图的发布者
    force_z_pub_ = nh_.advertise<std_msgs::Float64>("/force_plot/force_z", 10);
    theoretical_force_pub_ = nh_.advertise<std_msgs::Float64>("/force_plot/theoretical_force", 10);
    
    ROS_INFO("Force visualization initialized - topics: /force_plot/force_z, /force_plot/theoretical_force");
  }
  
  /**
   * @brief 发布力数据用于rqt绘图
   * @param force_z 实际测量的Z轴力值
   * @param theoretical_force 理论计算的力值
   */
  void publishForceData(double force_z, double theoretical_force) {
    std_msgs::Float64 force_z_msg;
    std_msgs::Float64 theoretical_force_msg;
    
    force_z_msg.data = force_z;
    theoretical_force_msg.data = theoretical_force;
    
    force_z_pub_.publish(force_z_msg);
    theoretical_force_pub_.publish(theoretical_force_msg);
    
  }
  
private:
  ros::NodeHandle& nh_;
  ros::Publisher force_z_pub_;
  ros::Publisher theoretical_force_pub_;
};

}  // namespace franka_example_controllers 