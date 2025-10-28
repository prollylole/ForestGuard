/**
* @file amcl_localisation.cpp
* @brief Implementation of AMCL localisation management node
* @details This file contains the implementation of the Amcllocalisation class,
* which manages the initialization and fallback localisation behavior for a robot
* using Adaptive Monte Carlo localisation (AMCL) in ROS2.

* The node first attempts to localize using an initial pose estimate. If the AMCL
* filter does not converge within a reasonable time, the node commands the robot to
* rotate slowly in place to gather more sensor data and help AMCL refine its position.
*/

#include "forestguard_localisation/amcl_localisation.h"

#include <chrono>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include <cmath>

/**
* @brief Constructor initializes AMCL localisation management node
* @details Sets up publishers, subscribers, and timers. Declares parameters for
* initial pose, covariance threshold, and rotation speed for fallback motion.
* The node automatically publishes an initial pose and monitors AMCL convergence.
*/
Amcllocalisation::Amcllocalisation()
: Node("amcl_localisation_node"), localized_(false), initial_pose_sent_(false)
{
  // Declare and read parameters
  this->declare_parameter("initial_x", 0.0);
  this->declare_parameter("initial_y", 0.0);
  this->declare_parameter("initial_yaw", 0.0);
  this->declare_parameter("cov_threshold", 0.05);
  this->declare_parameter("rotation_speed", 0.2);

  this->get_parameter("initial_x", init_x_);
  this->get_parameter("initial_y", init_y_);
  this->get_parameter("initial_yaw", init_yaw_);
  this->get_parameter("cov_threshold", cov_threshold_);
  this->get_parameter("rotation_speed", rotation_speed_);

  // Create publishers and subscribers
  initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10, std::bind(&Amcllocalisation::amclPoseCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(1s, std::bind(&Amcllocalisation::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "AMCL localisation Manager started.");
}

/**
* @brief Callback function to process AMCL pose updates
* @param msg Pose with covariance data from AMCL
* @details Evaluates covariance of position and orientation to check whether
* the robot has sufficiently localized. If localisation is stable, the robot stops moving.
*/
void Amcllocalisation::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  double cov_x = msg->pose.covariance[0];
  double cov_y = msg->pose.covariance[7];
  double cov_yaw = msg->pose.covariance[35];
  double avg_cov = (cov_x + cov_y + cov_yaw) / 3.0;

  if (avg_cov < cov_threshold_) {
    localized_ = true;
    stopMotion();
    RCLCPP_INFO(this->get_logger(), "localisation converged. Avg cov: %.5f", avg_cov);
    RCLCPP_INFO(this->get_logger(), "Initial pose published (%.2f, %.2f, %.2f rad)", init_x_, init_y_, init_yaw_);
  }
}

/**
* @brief Timer callback executed periodically
* @details If no initial pose has been sent, it publishes one. If localisation has not yet converged,
* the node commands the robot to rotate slowly to help AMCL collect better sensor readings.
*/
void Amcllocalisation::timerCallback()
{
  if (!initial_pose_sent_) {
    publishInitialPose();
    initial_pose_sent_ = true;
    RCLCPP_INFO(this->get_logger(), "Initial pose published (%.2f, %.2f, %.2f rad)", init_x_, init_y_, init_yaw_);
    return;
  }

  if (!localized_) {
    geometry_msgs::msg::Twist twist;
    twist.angular.z = rotation_speed_;
    cmd_vel_pub_->publish(twist);
    RCLCPP_WARN(this->get_logger(), "Rotating to help AMCL localize...");
  }
}

/**
* @brief Publishes the initial pose to the AMCL node
* @details Sends an initial pose with low covariance to initialize the particle filter
* and improve convergence speed.
*/
void Amcllocalisation::publishInitialPose()
{
  geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
  init_pose.header.stamp = this->now();
  init_pose.header.frame_id = "map";
  init_pose.pose.pose.position.x = init_x_;
  init_pose.pose.pose.position.y = init_y_;
  init_pose.pose.pose.orientation.z = sin(init_yaw_ / 2.0);
  init_pose.pose.pose.orientation.w = cos(init_yaw_ / 2.0);

  init_pose.pose.covariance[0] = 0.25;
  init_pose.pose.covariance[7] = 0.25;
  init_pose.pose.covariance[35] = 0.0685;

  initial_pose_pub_->publish(init_pose);
}

/**
* @brief Stops the robot’s motion
* @details Sends a zero-velocity command to the /cmd_vel topic to stop all movement.
*/
void Amcllocalisation::stopMotion()
{
  geometry_msgs::msg::Twist twist;
  cmd_vel_pub_->publish(twist);
}
