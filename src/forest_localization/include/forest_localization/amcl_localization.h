#ifndef AMCL_LOCALIZATION_H
#define AMCL_LOCALIZATION_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>

using namespace std::chrono_literals;

class AmclLocalization : public rclcpp::Node
{
public:
  AmclLocalization();

private:
  void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void timerCallback();
  void publishInitialPose();
  void stopMotion();

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State flags
  bool localized_;
  bool initial_pose_sent_;

  // Parameters
  double init_x_;
  double init_y_;
  double init_yaw_;
  double cov_threshold_;
  double rotation_speed_;
};

#endif  // AMCL_LOCALIZATION_H
