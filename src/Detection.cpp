#include "Detection.h"

DroneTreeDetector::DroneTreeDetector() : Node("tree_detector")
{
  using std::placeholders::_1;

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image", 10, std::bind(&DroneTreeDetector::imageCallback, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", 10, std::bind(&DroneTreeDetector::odomCallback, this, _1));

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/visualization_marker", 10);

  std::string world_path = this->declare_parameter<std::string>(
      "world_path", "/home/hallie/ros2_ws/src/Robo1_HAJE/worlds/simple_trees.sdf");

  loadWorldModels(world_path);

  RCLCPP_INFO(this->get_logger(),
              "Tree detector started with %zu models loaded from SDF.",
              model_positions_.size());
}

void DroneTreeDetector::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = *msg;
}

void DroneTreeDetector::loadWorldModels(const std::string &path)
{
  using namespace tinyxml2;

  if (!std::filesystem::exists(path))
  {
    RCLCPP_ERROR(this->get_logger(), "World file not found: %s", path.c_str());
    return;
  }

  XMLDocument doc;
  if (doc.LoadFile(path.c_str()) != XML_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load world file: %s", path.c_str());
    return;
  }

  XMLElement *world = doc.FirstChildElement("sdf")->FirstChildElement("world");
  if (!world)
  {
    RCLCPP_ERROR(this->get_logger(), "No <world> element found in %s", path.c_str());
    return;
  }

  for (XMLElement *include = world->FirstChildElement("include");
       include;
       include = include->NextSiblingElement("include"))
  {
    const char *name = include->FirstChildElement("name")
                           ? include->FirstChildElement("name")->GetText()
                           : nullptr;
    const char *pose_text = include->FirstChildElement("pose")
                                ? include->FirstChildElement("pose")->GetText()
                                : nullptr;

    if (!name || !pose_text)
      continue;

    if (std::string(name).find("tree") != std::string::npos ||
        std::string(name).find("oak") != std::string::npos ||
        std::string(name).find("pine") != std::string::npos)
    {
      std::istringstream ss(pose_text);
      geometry_msgs::msg::Point p;
      ss >> p.x >> p.y >> p.z;
      model_positions_[name] = p;

      RCLCPP_INFO(this->get_logger(),
                  "Loaded model %s at (%.2f, %.2f, %.2f)",
                  name, p.x, p.y, p.z);
    }
  }
}

void DroneTreeDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat label_img;
  try
  {
    label_img = cv_bridge::toCvCopy(msg, "mono8")->image;
  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Compute dominant label
  int histSize = 256;
  float range[] = {0, 256};
  const float *histRange = {range};
  cv::Mat hist;
  cv::calcHist(&label_img, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);
  double maxVal;
  cv::Point maxLoc;
  cv::minMaxLoc(hist, nullptr, &maxVal, nullptr, &maxLoc);

  int dom_label = maxLoc.y;
  int grass_label = 231;
  if (dom_label == grass_label)
    return;

  geometry_msgs::msg::Point drone_pos = current_odom_.pose.pose.position;
  geometry_msgs::msg::Point closest_tree;
  double min_dist = std::numeric_limits<double>::max();
  bool found_tree = false;

  {
    std::lock_guard<std::mutex> lock(model_mutex_);
    for (auto &[name, pos] : model_positions_)
    {
      double dx = pos.x - drone_pos.x;
      double dy = pos.y - drone_pos.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < min_dist && dist < 15.0)
      {
        min_dist = dist;
        closest_tree = pos;
        found_tree = true;
      }
    }
  }

  if (found_tree)
  {
    RCLCPP_INFO(this->get_logger(),
                "Dominant label=%d (≠ %d) → nearest tree at (%.2f, %.2f, %.2f), dist=%.2f m",
                dom_label, grass_label,
                closest_tree.x, closest_tree.y, closest_tree.z, min_dist);
    marker_pub_->publish(makeMarker(closest_tree));
  }
  else
  {
    RCLCPP_WARN(this->get_logger(),
                "Detected non-grass region but no nearby trees within 15 m.");
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
  marker.scale.x = 0.6;
  marker.scale.y = 0.6;
  marker.scale.z = 2.0;
  marker.color.a = 0.9;
  marker.color.r = 0.1;
  marker.color.g = 1.0;
  marker.color.b = 0.1;
  return marker;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneTreeDetector>());
  rclcpp::shutdown();
  return 0;
}
