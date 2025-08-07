/**
Resources:
  - https://github.com/Goodbudy/waitforme/blob/main/issy/src/Movement.cpp - Movement Logic for a single robot given a path
  - https://github.com/Goodbudy/waitforme/blob/integration_namespace/issy/src/Movement.cpp - Movement logic for multiple robots using unique namespace given a path
*/

// Begin Code Here
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>

class Movement : public rclcpp::Node {
public:
  Movement() : Node("movement_node") {
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/drone/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Waiting for user input...");
    std::thread(&Movement::listen_for_input, this).detach();
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  void listen_for_input() {
    while (rclcpp::ok()) {
      std::string input;
      std::getline(std::cin, input);

      if (input == "go") {
        RCLCPP_INFO(this->get_logger(), "Sending forward command");

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.5; // Forward speed

        vel_pub_->publish(cmd);
      } else if (input == "stop") {
        RCLCPP_INFO(this->get_logger(), "Stopping");

        geometry_msgs::msg::Twist cmd;
        // All zeros stop the drone
        vel_pub_->publish(cmd);
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Movement>());
  rclcpp::shutdown();
  return 0;
}