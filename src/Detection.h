#ifndef DETECTION_H
#define DETECTION_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tinyxml2.h>
#include <filesystem>
#include <fstream>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <mutex>
#include <map>
#include <vector>
#include <memory>
#include <cmath>

class DroneTreeDetector : public rclcpp::Node
{
public:
  DroneTreeDetector();

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void loadWorldModels(const std::string &path);
  visualization_msgs::msg::Marker makeMarker(const geometry_msgs::msg::Point &p);

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tree_pub_;

  // State
  nav_msgs::msg::Odometry current_odom_;
  std::map<std::string, geometry_msgs::msg::Point> model_positions_;
  std::vector<geometry_msgs::msg::Point> marked_positions_;
  std::mutex model_mutex_;
  int marker_id_ = 0;
};

#endif // DETECTION_H
