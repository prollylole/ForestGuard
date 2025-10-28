#ifndef CONTROLLER_HUSKY_H
#define CONTROLLER_HUSKY_H

#include <memory>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64.hpp"

#include "nav2_msgs/action/follow_waypoints.hpp"

struct GoalStats {
  geometry_msgs::msg::Point position;
  geometry_msgs::msg::Quaternion orientation;
};

class Controller : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollow = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  Controller();
  ~Controller();

private:
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mission_progress_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mission_distance_pub_;

  // Action client (Nav2 follow_waypoints)
  rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;
  std::string follow_waypoints_action_name_;

  // State
  std::mutex goalMtx_;
  std::vector<GoalStats> goals_;
  std::vector<double> segment_distances_;
  std::size_t current_goal_idx_;
  double total_mission_distance_;
  double completed_mission_distance_;
  bool goal_set_;

  // Odometry pose and lock
  std::mutex poseMtx_;
  geometry_msgs::msg::Pose current_pose_;

  // Laser
  std::mutex laserMtx_;
  sensor_msgs::msg::LaserScan last_scan_;
  bool laser_received_;

  // Utilities
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void goalPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  // Action helpers
  void sendFollowWaypointsAction(const std::vector<geometry_msgs::msg::PoseStamped> &waypoints);
  void handle_goal_response(rclcpp_action::ClientGoalHandle<FollowWaypoints>::SharedPtr goal_handle);
  void handle_feedback(typename GoalHandleFollow::SharedPtr, const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
  void handle_result(const rclcpp_action::ClientGoalHandle<FollowWaypoints>::WrappedResult &result);

  // markers & telemetry
  void publishGoalMarkers();
  void publishTelemetry();

  // helpers
  geometry_msgs::msg::Pose getCurrentPose();
  double compute_distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
};

#endif // CONTROLLER_HUSKY_H
