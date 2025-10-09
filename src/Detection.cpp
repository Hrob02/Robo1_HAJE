#include "Detection.h"
#include <cmath>
#include <memory>

DroneTreeDetector::DroneTreeDetector() : Node("tree_detector")
{
  using std::placeholders::_1;

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&DroneTreeDetector::scanCallback, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&DroneTreeDetector::odomCallback, this, _1));

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/visualization_marker", 10);

  RCLCPP_INFO(this->get_logger(), "Tree detector node started!");
}

void DroneTreeDetector::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = *msg;
}

void DroneTreeDetector::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  const double exclusion_min_z = -0.2;  // Ground-plane exclusion band
  const double exclusion_max_z = 0.2;
  const double duplicate_threshold = 0.5; // m — don’t spawn markers too close together

  for (size_t i = 0; i < scan->ranges.size(); ++i)
  {
    float range = scan->ranges[i];
    if (std::isinf(range) || range > 8.0)
      continue;

    // Detect trees roughly within 3 m
    if (range < 3.0)
    {
      float angle = scan->angle_min + i * scan->angle_increment;

      geometry_msgs::msg::Point p;
      p.x = current_odom_.pose.pose.position.x + range * std::cos(angle);
      p.y = current_odom_.pose.pose.position.y + range * std::sin(angle);
      p.z = current_odom_.pose.pose.position.z;

      // 🧱 Exclude points near the ground plane (avoid floor detections)
      if (p.z > exclusion_min_z && p.z < exclusion_max_z)
        continue;

      // 🧭 Check if a marker already exists close by (avoid spamming)
      bool duplicate = false;
      for (const auto &existing : detected_points_)
      {
        double dist = std::hypot(p.x - existing.x, p.y - existing.y);
        if (dist < duplicate_threshold)
        {
          duplicate = true;
          break;
        }
      }
      if (duplicate)
        continue;

      detected_points_.push_back(p);
      marker_pub_->publish(makeMarker(p));
    }
  }
}

visualization_msgs::msg::Marker DroneTreeDetector::makeMarker(const geometry_msgs::msg::Point &p)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "trees";
  marker.id = marker_id_++;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = p;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.4;
  marker.scale.y = 0.4;
  marker.scale.z = 1.0;
  marker.color.a = 0.9;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  return marker;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneTreeDetector>());
  rclcpp::shutdown();
  return 0;
}
