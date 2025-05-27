// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace franka_example_controllers {

/**
 * @brief Trajectory point structure, used for custom trajectories
 */
struct TrajectoryPoint {
  double time;
  Eigen::Vector3d position;
};

/**
 * @brief Trajectory type enumeration
 */
enum TrajectoryType {
  CIRCULAR,
  RECTANGULAR,
  FIGURE_EIGHT,
  LINE,
  CUSTOM
};

/**
 * @brief Trajectory generator class, handles different trajectory types and generation
 */
class TrajectoryGenerator {
public:
  /**
   * @brief Constructor for trajectory generator
   */
  TrajectoryGenerator();

  /**
   * @brief Initialize trajectory generator with parameters
   * 
   * @param node_handle ROS node handle to read parameters from
   * @param publisher Path publisher for visualization
   * @return True if initialization was successful, false otherwise
   */
  bool init(ros::NodeHandle& node_handle, ros::Publisher& path_publisher);

  /**
   * @brief Generate trajectory based on current trajectory type
   * 
   * @param time Current time
   * @return Eigen::Vector3d Generated position
   */
  Eigen::Vector3d generateTrajectory(double time);

  /**
   * @brief Generate trajectory based on current trajectory type with contact state
   * 
   * @param time Current time
   * @param in_contact Whether robot is in contact with surface
   * @param current_depth Current contact depth
   * @return Eigen::Vector3d Generated position
   */
  Eigen::Vector3d generateTrajectory(double time, bool in_contact = false, double current_depth = 0.0);

  /**
   * @brief Generate circular trajectory
   * 
   * @param time Current time
   * @return Eigen::Vector3d Generated position
   */
  Eigen::Vector3d generateCircularTrajectory(double time);

  /**
   * @brief Generate rectangular trajectory
   * 
   * @param time Current time
   * @return Eigen::Vector3d Generated position
   */
  Eigen::Vector3d generateRectangularTrajectory(double time);

  /**
   * @brief Generate figure eight trajectory
   * 
   * @param time Current time
   * @return Eigen::Vector3d Generated position
   */
  Eigen::Vector3d generateFigureEightTrajectory(double time);

  /**
   * @brief Generate line trajectory
   * 
   * @param time Current time
   * @return Eigen::Vector3d Generated position
   */
  Eigen::Vector3d generateLineTrajectory(double time);

  /**
   * @brief Generate custom trajectory
   * 
   * @param time Current time
   * @return Eigen::Vector3d Generated position
   */
  Eigen::Vector3d generateCustomTrajectory(double time);

  /**
   * @brief Load custom trajectory from file
   * 
   * @param filename Path to custom trajectory file
   * @return bool True if loading was successful, false otherwise
   */
  bool loadCustomTrajectory(const std::string& filename);

  /**
   * @brief Publish trajectory path for visualization
   * 
   * @param current_time Current time
   * @param path_z_height Z height for visualization path
   */
  void publishTrajectoryPath(double current_time, double path_z_height);

  /**
   * @brief Set center of trajectory
   * 
   * @param center Center position for trajectory
   */
  void setCenter(const Eigen::Vector3d& center);

  /**
   * @brief Set axis for circular motion
   * 
   * @param x_axis X axis for circular motion
   * @param y_axis Y axis for circular motion
   */
  void setAxis(const Eigen::Vector3d& x_axis, const Eigen::Vector3d& y_axis);

  /**
   * @brief Set soft block parameters for depth control
   * 
   * @param surface_z Soft block surface z-coordinate
   * @param target_depth Target contact depth
   */
  void setSoftBlockParameters(double surface_z, double target_depth);

  /**
   * @brief Set target contact depth
   * 
   * @param depth Target contact depth
   */
  void setTargetDepth(double depth);

  /**
   * @brief Set whether to preserve current height during trajectory generation
   * 
   * @param preserve Whether to preserve current height
   */
  void setPreserveHeight(bool preserve);

  /**
   * @brief Set whether contact is required for trajectory execution
   * 
   * @param required Whether contact is required
   */
  void setContactRequired(bool required);

  /**
   * @brief Get trajectory type
   * 
   * @return TrajectoryType Current trajectory type
   */
  TrajectoryType getTrajectoryType() const;

  /**
   * @brief Get trajectory type as string
   * 
   * @return std::string Trajectory type as string
   */
  std::string getTrajectoryTypeStr() const;

private:
  // Trajectory parameters
  TrajectoryType trajectory_type_;          // Current trajectory type
  std::string trajectory_type_str_;         // Current trajectory type as string
  double circle_frequency_;                 // Frequency of circular motion
  double circle_radius_;                    // Radius of circular motion
  std::string circle_plane_;                // Plane of circular motion (xy, xz, yz)
  double speed_factor_;                     // Speed factor to adjust trajectory speed
  
  // Circle center and axes
  Eigen::Vector3d circle_center_;           // Center position for circular motion
  Eigen::Vector3d circle_x_axis_;           // X axis for circular motion
  Eigen::Vector3d circle_y_axis_;           // Y axis for circular motion
  
  // Additional trajectory parameters
  double rect_length_;                      // Length of rectangular trajectory
  double rect_width_;                       // Width of rectangular trajectory
  double eight_radius_x_;                   // X radius for figure eight trajectory
  double eight_radius_y_;                   // Y radius for figure eight trajectory
  double line_length_;                      // Length of line trajectory
  
  // Custom trajectory
  std::string custom_trajectory_file_;      // Path to custom trajectory file
  std::vector<TrajectoryPoint> custom_trajectory_points_; // Custom trajectory points
  
  // Depth control parameters
  bool contact_required_;                   // Whether contact should be maintained
  double soft_block_surface_z_;             // Z-coordinate of soft block surface
  double target_depth_;                     // Target contact depth
  bool preserve_height_;                    // Whether to preserve current height
  
  // Visualization
  ros::Publisher path_pub_;                 // Publisher for trajectory visualization
};

} // namespace franka_example_controllers 