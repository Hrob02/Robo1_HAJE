
// testing testing

// /**
// Resources:
//   - https://github.com/Goodbudy/waitforme/blob/main/issy/src/Movement.cpp - Movement Logic for a single robot given a path
//   - https://github.com/Goodbudy/waitforme/blob/integration_namespace/issy/src/Movement.cpp - Movement logic for multiple robots using unique namespace given a path
// */

// // Begin Code Here
// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "nav2_msgs/action/navigate_to_pose.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include <iostream>
// #include <string>
// #include <thread>
// #include <chrono>
// #include <cmath>

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include <iostream>
// #include <string>
// #include <thread>
// #include <chrono>

// class Movement : public rclcpp::Node {
// public:
//     Movement() : Node("movement_node") {
//         // Publisher to the drone's cmd_vel topic
//         vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/simple_drone/cmd_vel", 10);

//         RCLCPP_INFO(this->get_logger(), "Movement node ready. Type 'go' or 'stop'.");
//         std::thread(&Movement::listen_for_input, this).detach();
//     }

// private:
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

//     void listen_for_input() {
//         while (rclcpp::ok()) {
//             std::string input;
//             std::getline(std::cin, input);

//             if (input == "go") {
//                 RCLCPP_INFO(this->get_logger(), "🚁 GO command received: moving forward");
//                 geometry_msgs::msg::Twist cmd;
//                 cmd.linear.x = 0.5;  // forward velocity

//                 for (int i = 0; i < 20; ++i) { // Publish for ~2 seconds
//                     vel_pub_->publish(cmd);
//                     rclcpp::sleep_for(std::chrono::milliseconds(100));
//                 }

//             } else if (input == "stop") {
//                 RCLCPP_INFO(this->get_logger(), "🛑 STOP command received: halting");
//                 geometry_msgs::msg::Twist cmd;

//                 for (int i = 0; i < 10; ++i) {
//                     vel_pub_->publish(cmd);
//                     rclcpp::sleep_for(std::chrono::milliseconds(100));
//                 }
//             } else {
//                 RCLCPP_WARN(this->get_logger(), "Unknown command. Type 'go' or 'stop'.");
//             }
//         }
//     }
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<Movement>());
//     rclcpp::shutdown();
//     return 0;
// }
