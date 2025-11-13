/**
 * @file controller_husky.cpp
 * @brief Implementation of Husky robot controller for forest navigation
 * @details This file contains the implementation of the Controller class,
 * which manages navigation, sensor processing, and mission execution for
 * a Husky robot in a forest environment using ROS2 and Nav2.
 * 
 * this file receives ros2 data from various topics and send them to the telemetry processor
 * also sets goal and send them to nav2 to follow waypoints
 * service provides changable status between autonomous and teleoperation
 */

#include "forestguard_controller/controller_husky.h"

#include <chrono>
#include <sstream>
#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std::chrono_literals;

/**
 * @brief Constructor initializes Husky controller node and ROS2 infrastructure
 * @details Sets up subscribers for sensor data, action clients for navigation,
 * publishers for visualization/telemetry, and timers for periodic tasks
 * Initializes mission tracking variables and connects to Nav2 waypoint server
 */
Controller::Controller()
: Node("husky_controller_node"),
  current_goal_idx_(0),
  total_mission_distance_(0.0),
  completed_mission_distance_(0.0),
  goal_set_(false),
  laser_received_(false)
{
  // Parameters
  this->declare_parameter<std::string>("goal_action_name", "follow_waypoints");
  this->get_parameter("goal_action_name", follow_waypoints_action_name_);

  // Subscriptions
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&Controller::laserCallback, this, std::placeholders::_1));

    // goal provided externally as PoseArray: from rviz or planner node, or file, teleoperation
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/goal_pose", 10, std::bind(&Controller::goalPoseCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 10, std::bind(&Controller::odomCallback, this, std::placeholders::_1));

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image", 5, std::bind(&Controller::imageCallback, this, std::placeholders::_1));

  // Publishers
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker", 10);
  mission_progress_pub_ = this->create_publisher<std_msgs::msg::Float64>("/mission_progress", 10);
  mission_distance_pub_ = this->create_publisher<std_msgs::msg::Float64>("/mission_distance", 10);

  /* ---------- Nav2 Action Client ---------- */

  // bridging to nav2 functionality, communicates by sending action goal to Nav2 action server
  // Action client (nav2 follow_waypoints) called follow_waypoints_client
  // this client communicates with nav2 waypoint follower, node name is /waypoint_follower
  // so action type is nav2_msgs::action::FollowWaypoints

  // get follow_waypoints action name by running nav2 bringup launch file, 
  // this activates the /waypoint_follower node which provides the follow_waypoints action server
  this->declare_parameter<std::string>("follow_waypoints_action_name", "/follow_waypoints");
  this->get_parameter("follow_waypoints_action_name", follow_waypoints_action_name_);

  // connect to follow_waypoints_client using server action name from node /waypoint_follower/follow_waypoints
  // client uses message type /follow_waypoints: nav2_msgs/action/FollowWaypoints
  follow_waypoints_client_ = rclcpp_action::create_client<FollowWaypoints>(this, follow_waypoints_action_name_);

  // Wait for action server to come up (async but do a short wait)
  if (!follow_waypoints_client_->wait_for_action_server(3s)) {
    RCLCPP_WARN(this->get_logger(), "FollowWaypoints action server '%s' not available yet.", follow_waypoints_action_name_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "FollowWaypoints action client connected to '%s'.", follow_waypoints_action_name_.c_str());
  }

  // Periodic telemetry + markers
  auto timer_cb = [this]() {
    publishGoalMarkers();
    publishTelemetry();
  };

  this->create_wall_timer(200ms, timer_cb);

  RCLCPP_INFO(this->get_logger(), "Husky controller node started");
}

Controller::~Controller()
{
  RCLCPP_INFO(this->get_logger(), "Husky controller node shutting down");
}

/* ---------- Callbacks ---------- */

/**
 * @brief Processes incoming laser scan data for obstacle detection
 * @param msg Shared pointer to laser scan message containing range measurements
 * @details Stores latest scan data and updates laser reception flag for
 * availability checking in other components
 */
void Controller::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  last_scan_ = *msg;
  laser_received_ = true;
}

/**
 * @brief Updates robot's current position from odometry data
 * @param msg Shared pointer to odometry message containing pose information
 * @details Maintains current pose state for navigation and progress calculation
 */
void Controller::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;
}

/**
 * @brief Processes incoming camera images for visual perception
 * @param msg Shared pointer to image message from camera
 * @details Converts ROS image to OpenCV format for potential visual processing
 * and analysis. Currently serves as a stub for future visual capabilities.
 */
void Controller::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Minimal OpenCV processing stub. Expand for color / depth detection.
  try {
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    // small debug thumbnail or processing could be added here
    // e.g., cv::GaussianBlur(image, image, cv::Size(5,5), 0);
    // cv::imshow("camera", image); cv::waitKey(1);
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_WARN(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

/**
 * @brief Receives and processes navigation waypoints mission
 * @param msg Shared pointer to PoseArray containing goal positions
 * @details Parses waypoint array, calculates segment distances, initializes
 * mission tracking, and triggers navigation action to Nav2 server
 */
void Controller::goalPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  if (!msg || msg->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty goal pose array received");
    return;
  }

  // Convert PoseArray -> vector<PoseStamped> (frame_id set to msg.header.frame_id or "map")
  // ensure all waypoints are expressed in the map coordinate frame so nav2 can plan path using its costmaps
  // nav2 uses use loaded map and amcl running to compute a plan to each waypoint
  std::string frame = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;

  // storing posestamped waypoints into vector for nav2
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  waypoints.reserve(msg->poses.size());

  // Reset mission tracking variables: goals, distance between goals, 
  // total distance, distance travelled, which waypoint we're heading to
  {
    goals_.clear();
    segment_distances_.clear();
    total_mission_distance_ = 0.0;
    completed_mission_distance_ = 0.0;
    current_goal_idx_ = 0;
  }

  geometry_msgs::msg::Pose prev_pose;
  {
    prev_pose = current_pose_;
  }

  // use the latest known pose as prev_pose
  // If current pose uninitialized, set have_prev false to skip first segment distance
  bool have_prev = true;
  if (std::isnan(prev_pose.position.x) && std::isnan(prev_pose.position.y)) {
    have_prev = false;
  }

  // loop through each pose and convert to PoseStamped, adds to waypoint for nav2
  for (const auto &p : msg->poses) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame;
    ps.header.stamp = this->now();
    ps.pose = p;
    waypoints.push_back(ps);

    // bookkeeping for telemetry
    // struct to hold position + orientation, each goals_ has position + orientation
    GoalStats gs;
    gs.position = p.position;
    gs.orientation = p.orientation;
    {
      goals_.push_back(gs);
    }

    // compute segment distance, sum it up
    double seg = 0.0;
    if (have_prev) {
      seg = compute_distance(prev_pose.position, p.position);
    }
    segment_distances_.push_back(seg);
    total_mission_distance_ += seg;

    // update prev
    prev_pose = p;
    have_prev = true;
  }

  // mark that a mission is ready
  {
    goal_set_ = !goals_.empty();
  }
  // Send action to Nav2
  sendFollowWaypointsAction(waypoints);
}

/* ---------- Action helpers ---------- */

/**
 * @brief Sends waypoint navigation request to Nav2 action server
 * @param waypoints Vector of pose-stamped waypoints for navigation
 * @details Constructs and transmits FollowWaypoints action goal to Nav2
 * with appropriate response, feedback, and result callbacks
 */
void Controller::sendFollowWaypointsAction(const std::vector<geometry_msgs::msg::PoseStamped> &waypoints)
{
  if (!follow_waypoints_client_) {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    return;
  }

  if (!follow_waypoints_client_->wait_for_action_server(2s)) {
    RCLCPP_WARN(this->get_logger(), "FollowWaypoints action server not available. Will retry later.");
    return;
  }

  // create goal message and assign waypoints in, this is the route nav2 will follow
  auto goal_msg = FollowWaypoints::Goal();
  goal_msg.poses = waypoints;

  RCLCPP_INFO(this->get_logger(), "Sending FollowWaypoints goal with %zu waypoints", waypoints.size());

  auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();

  // action servers are used for long running tasks, so we provide callbacks to handle responses, feedback, and results

  // goal_response_callback called when server accepts/rejects goal after goal is sent by async_send_goal
  send_goal_options.goal_response_callback =
    std::bind(&Controller::handle_goal_response, this, std::placeholders::_1);

  // feedback_callback activated when the goal is being processed, 
  // sends periodic updates - current waypoint index, distance remaining, etc.
  // feedback_callback called periodically during execution to provide updates on progress
  send_goal_options.feedback_callback =
    std::bind(&Controller::handle_feedback, this, std::placeholders::_1, std::placeholders::_2);

  // result_callback called when goal is complete - success, failure, or canceled
  send_goal_options.result_callback =
    std::bind(&Controller::handle_result, this, std::placeholders::_1);

  // send the goal to the action server
  follow_waypoints_client_->async_send_goal(goal_msg, send_goal_options);
}

/**
 * @brief Handles response from Nav2 server after goal submission
 * @param goal_handle Shared pointer to action goal handle
 * @details Processes acceptance/rejection of navigation goal and logs
 * appropriate status messages
 */
void Controller::handle_goal_response(rclcpp_action::ClientGoalHandle<FollowWaypoints>::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "FollowWaypoints goal accepted by server, waiting for result");
}

/**
 * @brief Processes real-time feedback during waypoint navigation
 * @param goal_handle Shared pointer to active goal handle (unused but required by interface)
 * @param feedback Shared pointer to feedback message from Nav2
 * @details Updates current waypoint index based on navigation progress
 * and triggers telemetry publication
 */
void Controller::handle_feedback(
  typename Controller::GoalHandleFollow::SharedPtr,
  const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
{
  // Feedback contains current_waypoint_index etc. Use to update current_goal_idx_
  if (!feedback) return;
  {
    current_goal_idx_ = std::min<std::size_t>(feedback->current_waypoint, goals_.empty() ? 0 : goals_.size()-1);
  }
  // publish telemetry periodically if desired
  publishTelemetry();
}

/**
 * @brief Handles completion of waypoint navigation mission
 * @param result Wrapped result containing navigation outcome
 * @details Processes final result status (success/failure/cancellation)
 * and updates mission state accordingly
 */
void Controller::handle_result(const rclcpp_action::ClientGoalHandle<FollowWaypoints>::WrappedResult &result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "FollowWaypoints action succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(this->get_logger(), "FollowWaypoints action aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "FollowWaypoints action canceled");
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "FollowWaypoints action unknown result code");
      break;
  }
  // When result finished, update completed distance.
  {
    goal_set_ = false;
  }
}

/* ---------- Markers & Telemetry ---------- */

/**
 * @brief Publishes visualization markers for goals and navigation aids
 * @details Creates and publishes RViz markers showing waypoint positions
 * and directional arrows from robot to current active goal
 */
void Controller::publishGoalMarkers()
{
  visualization_msgs::msg::MarkerArray arr;
  rclcpp::Time now = this->now();

  // Publish all goals as spheres
  int id = 0;
  for (std::size_t i = 0; i < goals_.size(); ++i) {
    const auto &g = goals_[i];
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = now;
    m.ns = "goals";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position = g.position;
    m.pose.orientation = g.orientation;
    m.scale.x = 0.3;
    m.scale.y = 0.3;
    m.scale.z = 0.3;
    m.color.r = (i==current_goal_idx_) ? 0.0f : 0.0f;
    m.color.g = (i==current_goal_idx_) ? 1.0f : 0.5f;
    m.color.b = (i==current_goal_idx_) ? 0.0f : 0.2f;
    m.color.a = 1.0f;
    m.lifetime = rclcpp::Duration(1s);
    arr.markers.push_back(m);

    // arrow from robot to current goal (only for the active goal)
    if (i == current_goal_idx_) {
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = "map";
      arrow.header.stamp = now;
      arrow.ns = "current_goal_arrow";
      arrow.id = id++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      geometry_msgs::msg::Point start;
      {
        start = current_pose_.position;
      }
      arrow.points.push_back(start);
      arrow.points.push_back(g.position);
      arrow.scale.x = 0.05; // shaft diameter
      arrow.scale.y = 0.1;  // head diameter
      arrow.scale.z = 0.1;  // head length
      arrow.color.r = 1.0f;
      arrow.color.g = 0.2f;
      arrow.color.b = 0.2f;
      arrow.color.a = 1.0f;
      arrow.lifetime = rclcpp::Duration(200ms);
      arr.markers.push_back(arrow);
    }
  }

  marker_pub_->publish(arr);
}


/**
 * @brief Publishes mission progress and distance telemetry
 * @details Calculates and publishes current mission completion percentage
 * and total mission distance for monitoring and visualization
 */
void Controller::publishTelemetry()
{
  std_msgs::msg::Float64 dist_msg;
  std_msgs::msg::Float64 prog_msg;

  double progress = 0.0;
  {
    if (!goal_set_ || goals_.empty() || total_mission_distance_ <= 0.000001) {
      progress = 0.0;
    } else {
      // compute current progress
      // completed_mission_distance_ + progress in current segment
      geometry_msgs::msg::Pose cur;
      {
        cur = current_pose_;
      }
      double current_segment_completed = 0.0;
      // compute distance to current goal
      if (current_goal_idx_ < goals_.size()) {
        double seg_len = segment_distances_.at(current_goal_idx_);
        double to_goal = compute_distance(cur.position, goals_.at(current_goal_idx_).position);
        if (seg_len > 1e-6) {
          double part = std::max(0.0, seg_len - to_goal);
          current_segment_completed = part;
        }
      }
      double completed = completed_mission_distance_ + current_segment_completed;
      progress = (total_mission_distance_ > 1e-6) ? (completed / total_mission_distance_) * 100.0 : 0.0;
      if (progress < 0.0) progress = 0.0;
      if (progress > 100.0) progress = 100.0;
    }
  }

  dist_msg.data = total_mission_distance_;
  prog_msg.data = progress;
  mission_distance_pub_->publish(dist_msg);
  mission_progress_pub_->publish(prog_msg);
}

/* ---------- Helpers ---------- */

/**
 * @brief Retrieves current robot pose from internal state
 * @return Current pose of the robot in map frame
 * @details Provides thread-safe access to latest odometry data
 */
geometry_msgs::msg::Pose Controller::getCurrentPose()
{
  return current_pose_;
}

/**
 * @brief Calculates Euclidean distance between two 2D points
 * @param a First point coordinates
 * @param b Second point coordinates  
 * @return Straight-line distance between points in meters
 * @details Computes 2D distance ignoring Z-coordinate for ground robot navigation
 */
double Controller::compute_distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx*dx + dy*dy);
}
