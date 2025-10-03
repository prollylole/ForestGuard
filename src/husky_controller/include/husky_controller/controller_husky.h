#ifndef CONTROLLER_HUSKY_H
#define CONTROLLER_HUSKY_H

/**
 * @file controller.cpp
 * @brief Implementation of the Controller base class for robot control
 * 
 * This class serves as the foundation for robot controllers, providing:
 * - Goal management and tracking
 * - Sensor data processing (laser, odometry)
 * - Mission control through ROS services
 * - Status monitoring and reporting
 */

#include <cmath>
#include <thread>

#include <functional>
#include <memory>
// #include <pfms_types.h>

#include "rclcpp/rclcpp.hpp"
#include <tf2/utils.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "std_srvs/srv/set_bool.hpp"
// #include "controller_pkg/laserprocessing.h"


enum PlatformStatus {
    IDLE,      // Platform waiting for commands
    RUNNING,   // Platform actively executing goals
    READY,    // Platform performing landing sequence
    STOPPED,   // Platform has been stopped
    TELEOP   // Platform under manual control
};

/**
 * @brief Structure containing goal position and orientation information
 */
struct GoalStats {
  geometry_msgs::msg::Point position;
  geometry_msgs::msg::Quaternion orientation;
};

/**
 * @brief Abstract base class for robot controllers
 * @details Provides core functionality for navigation, goal tracking, and sensor processing
 */
class Controller: public rclcpp::Node
{
public:

  /**
   * @brief Constructor initializes ROS node and default values
   */
  Controller(); 

  /**
   * @brief Virtual destructor for proper cleanup of derived classes
   */
  virtual ~Controller() = 0; // Destructor to abort

  /**
   * @brief Sets new navigation goal
   * @param msg Array of poses representing goals
   */
  void setGoal(const geometry_msgs::msg::PoseArray& msg); 
  
   /**
   * @brief Checks if path to destination is feasible
   * @param origin Current pose of the quadcopter
   * @param goalPosition Target position to reach
   * @param[out] distance Calculated distance to goal
   * @param[out] time Estimated time to reach goal
   * @param[out] estimatedGoalPose Expected pose at goal
   * @return true if path is clear, false if blocked
   */
  virtual bool checkOriginToDestination(geometry_msgs::msg::Pose origin, 
    geometry_msgs::msg::Point goalPosition,
    double& distance, double& time,
    geometry_msgs::msg::Pose& estimatedGoalPose) = 0;

  /**
   * @brief Sets acceptable distance tolerance for reaching goals
   * @param tolerance Distance in meters
   * @return true if tolerance was set successfully
   */
  bool setTolerance(double tolerance);

  /**
   * @brief Gets current odometry reading
   * @return Current pose of the platform
   */
  geometry_msgs::msg::Pose getOdometry(void);

  /**
 * @brief Service callback for controlling quadcopter mission
 * @param req Service request containing control command
 * @param res Service response indicating success/failure
 */
  void control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,std::shared_ptr<std_srvs::srv::SetBool::Response> res);


protected:

  std::chrono::time_point<std::chrono::system_clock> startTime; //!< The time when the platform started moving
  std::vector<GoalStats> goals_; //!< Store all goals
  size_t current_goal_idx_; //!< Track current goal index

  double distance_toGoal_; //!< Distance to the current goal   
  // double time_toGoal_; //!< Time to the current goal   
  bool goalSet_; //!< Flag indicating if a goal has been set
  double tolerance_; //!< Radius of tolerance
  double total_mission_distance_;      // Total planned mission distance
  double completed_mission_distance_;  // Distance of completed segments
  std::vector<double> segment_distances_;  // Stores distances for each segment

  PlatformStatus status_; //!< The current platform stats
  std::mutex goalMtx_; //!< Mutex for controlling access to goal
  std::mutex laserMtx_; //!< Mutex for controlling access to laser data
  std::mutex poseMtx_; //!< Mutex for controlling access to pose

//   std::unique_ptr<LaserProcessing> laserProcessingPtr_; //!< Pointer to the laser processing object
  sensor_msgs::msg::LaserScan laserData_; //!< Laser data obtained by callback
  std::atomic<bool> laserDataReceived_ ; // Flag to indicate laser data received

  /**
   * Instructs the underlying platform to recalcuate a goal, and set any internal variables as needed
   *
   * Called when goal or tolerance changes
   * @return Whether goal is reachable by the platform
   */
  virtual GoalStats calcNewGoal() = 0;

  /**
   * Checks if the goal has been reached.
   *
   * Update own pose before calling!
   * @return true if the goal is reached
   */
  bool goalReached();

  /**
   * @brief Gets current goal information
   * @return GoalStats structure containing goal position and orientation
   */
  GoalStats getGoalStats(void);

  /**
   * @brief Gets formatted status string for debugging
   * @return String containing current controller status
   */
  std::string getInfoString(void);

  double status();


private:

  geometry_msgs::msg::Pose pose_;//!< The current pose of platform
  nav_msgs::msg::Odometry odo_;//!< The current odometry data
  GoalStats goal_;//!< The current goal data

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;//!< Pointer to the laser scan subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;//!< Pointer to the odometry subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goal_sub_;//!< Pointer to the goal subscriber
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;//!< Service for mission control

  double distance_travelled_; //!< Total distance travelled for this program run
  double time_travelled_; //!< Total time spent travelling for this program run
  long unsigned int seq_; //!<The sequence number of the command

  /**
   * @brief Callback for processing laser scan data
   * @param msg LaserScan message from ROS topic
   */
  void laserCallback(const sensor_msgs::msg::LaserScan& msg);

   /**
   * @brief Callback for processing odometry data
   * @param msg Odometry message from ROS topic
   */
  void odoCallback(const nav_msgs::msg::Odometry& msg);
};


#endif // CONTROLLER_H