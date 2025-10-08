#ifndef HUSKY_H
#define HUSKY_H

/**
 * @file husky.h
 * @brief Header for the Husky derived class for ground robot control
 * 
 * This class implements specific Husky functionality including:
 * - Autonomous navigation between goals
 * - Obstacle avoidance using laser data
 * - Human detection using laser data
 * - Real-time visualization of detections
 * 
 * The Husky navigates through environments, detecting humans while
 * maintaining safe ground operations and goal tracking.
 */

#include "controller_husky.h"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

class Husky: public Controller
{
public:
  /**
   * @brief Constructor initializes Husky parameters and ROS publishers
   * @details Sets up publishers for velocity commands and detection visualization
   */
  Husky();

  /**
   * @brief Destructor ensures safe shutdown of Husky
   */
  ~Husky(); 

  /**
   * @brief Checks if path to destination is feasible
   * @param origin Current pose of the Husky
   * @param goalPosition Target position to reach
   * @param[out] distance Calculated distance to goal
   * @param[out] time Estimated time to reach goal
   * @param[out] estimatedGoalPose Expected pose at goal
   * @return true if path is clear, false if blocked
   */
  bool checkOriginToDestination(geometry_msgs::msg::Pose origin, 
    geometry_msgs::msg::Point goalPosition,
    double& distance, double& time,
    geometry_msgs::msg::Pose& estimatedGoalPose); 

protected:
  /**
   * Instructs the underlying platform to recalculate a goal, and set any internal variables as needed
   *
   * Called when goal or tolerance changes
   * @return Whether goal is reachable by the platform
   */
  GoalStats calcNewGoal(void);

private:
  // Constants
  const double TARGET_SPEED; //<-- Maximum movement speed

  // State variables
  double target_angle_; //<-- Angle required for Husky to have a straight shot at the goal
  double distance_toGoal_;  //!< Current Euclidean distance to the navigation goal in meters

  // ROS Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmdVel_;//!< Publisher for velocity commands
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr PubHuman_;//!< Publisher for detected human visualization
  rclcpp::TimerBase::SharedPtr timer_;//!< Timer for control loop execution

  // Teleop control
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVel_sub_;
  geometry_msgs::msg::Twist currentTeleopCommand_; //!< Stores the latest teleop command
  bool teleopActive_; //!< Flag indicating if teleop command is active
  rclcpp::Time lastTeleopTime_; //!< Timestamp of the last received teleop command

  /**
   * @brief Timer callback that implements goal-reaching behavior
   * @return true if goal was reached successfully
   */
  bool reachGoal(void);

  /**
   * @brief Sends movement commands to Husky
   * @param linear_x Linear velocity along X axis
   * @param angular_z Angular velocity around Z axis
   */
  void sendCmd(double linear_x, double angular_z);

  /**
   * @brief Switches to Teleop mode and sends teleop command
   */
  void sendTeleopCmd(const geometry_msgs::msg::Twist::SharedPtr msg);
  
#endif // HUSKY_H
};