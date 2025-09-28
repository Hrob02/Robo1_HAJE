/**
@class Movement
@breif Handles movement execution for autonomous robots.

  Core Functions and Responsibilities:
    - Movement Execution 
    - etc

@note
@author
@date
*/

/**
Resources:
  - https://github.com/Goodbudy/waitforme/blob/main/issy/src/movementlogic.h - Movement Logic for a single robot given a path
  - https://github.com/Goodbudy/waitforme/blob/integration_namespace/issy/src/movementlogic.h - Movement logic for multiple robots using unique namespace given a path
  - https://github.com/NovoG93/sjtu_drone/blob/ros2/README.md drone to use
*/

// Begin Code Here
#ifndef MOVEMENT_HPP
#define MOVEMENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>

class Movement : public rclcpp::Node {
  public:
      using NavigateToPose = nav2_msgs::action::NavigateToPose;
      using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  
      Movement();
  
  private:
      rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
      double x_home, y_home;
      double x_goal, y_goal;
      bool move_now;
  
      void listen_for_input();
      void navigate_to(double x, double y);
  };
  
  #endif // MOVEMENT_HPP