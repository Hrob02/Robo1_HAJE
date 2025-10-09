#ifndef DETECTION_H
#define DETECTION_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <vector>
#include <memory>

/**
 * @brief Detects nearby trees and publishes visualization markers.
 * 
 * Subscribes to /scan and /odom, processes points within a threshold range,
 * and publishes cylinder markers representing detected trees.
 */
class DroneTreeDetector : public rclcpp::Node
{
public:
  DroneTreeDetector();

private:
  // === Callbacks ===
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  // === Utility ===
  visualization_msgs::msg::Marker makeMarker(const geometry_msgs::msg::Point &p);

  // === Members ===
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  nav_msgs::msg::Odometry current_odom_;
  std::vector<geometry_msgs::msg::Point> detected_points_;
  int marker_id_ = 0;
};

#endif
