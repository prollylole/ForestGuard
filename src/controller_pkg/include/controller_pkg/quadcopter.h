#ifndef QUADCOPTER_H
#define QUADCOPTER_H

/**
 * @file new_quadcopter.cpp
 * @brief Implementation of the Quadcopter derived class for aerial robot control
 * 
 * This class implements specific quadcopter functionality including:
 * - Autonomous navigation between goals
 * - Takeoff and landing operations
 * - Human detection using laser data
 * - Real-time visualization of detections
 * 
 * The quadcopter navigates through rooms, detecting humans while
 * maintaining safe flight operations and goal tracking.
 */

#include "controller.h"

#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// #include <pfms_types.h>


class Quadcopter: public Controller
{
public:
  /**
   * @brief Constructor initializes quadcopter parameters and ROS publishers
   * @details Sets up publishers for velocity commands, takeoff/landing signals,
   * and detection visualization
   */
  //Default constructor - should set all sensor attributes to a default value
  Quadcopter();

  /**
   * @brief Destructor ensures safe shutdown of quadcopter
   */
  ~Quadcopter(); 

  /**
   * @brief Checks if path to destination is feasible
   * @param origin Current pose of the quadcopter
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
   * Instructs the underlying platform to recalcuate a goal, and set any internal variables as needed
   *
   * Called when goal or tolerance changes
   * @return Whether goal is reachable by the platform
   */
  GoalStats calcNewGoal(void);

private:
  // Constants
  const double TARGET_SPEED; //<-- Maximum movement speed
  const double TARGET_HEIGHT_TOLERANCE; //<-- Acceptable height error margin

  // State variables
  double target_angle_ = 0; //<-- Angle required for quadcopter to have a straight shot at the goal
  bool liftoff_; //<-- indicate if the quadcopter is in the air

  // ROS Pubishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmdVel_;//!< Publisher for velocity commands
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubTakeOff_;//!< Publisher for takeoff signal
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubLanding_;//!< Publisher for landing signal
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr PubDoor_;//!< Publisher for detected door visualization
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr PubHuman_;//!< Publisher for detected human visualization
  rclcpp::TimerBase::SharedPtr timer_;//!< Timer for control loop execution

  // Teleop control
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVel_sub_;
  geometry_msgs::msg::Twist currentTeleopCommand_; //!< Stores the latest teleop
  bool teleopActive_; //!< Flag indicating if teleop command is active
  rclcpp::Time lastTeleopTime_; //!< Timestamp of the last received teleop

  /**
   * @brief Timer callback that implements goal-reaching behavior
   * @return true if goal was reached successfully
   */
  bool reachGoal(void);

  /**
   * @brief Sends movement commands to quadcopter
   * @param turn_l_r Angular velocity around Z axis
   * @param move_l_r Linear velocity along Y axis
   * @param move_u_d Linear velocity along Z axis
   * @param move_f_b Linear velocity along X axis
   */
  void sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b);

  /**
   * @brief Switches to Teleop mode and sends teleop command
   */
  void sendTeleopCmd(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Initiates takeoff sequence
   */
  void sendTakeOff(void);

  /**
   * @brief Initiates landing sequence
     */
  void sendLanding(void);
};

#endif