#ifndef CONTROLLER_HUSKY_H
#define CONTROLLER_HUSKY_H

/**
 * @file controller_husky.h
 * @brief Header for the Controller base class for Husky ground robot control
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <memory>
#include <vector>
#include <mutex>

// Forward declaration
class LaserProcessing;

/**
 * @brief Platform status enum for Husky robot states
 */
enum PlatformStatus {
    IDLE,       // Robot is stopped, waiting for commands
    RUNNING,    // Robot is actively executing goals
    TELEOP      // Robot is under teleoperation control
};

/**
 * @brief Structure to store goal statistics
 */
struct GoalStats {
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Quaternion orientation;
};

/**
 * @brief Base controller class for Husky ground robot
 * 
 * Provides foundation for goal management, sensor processing,
 * and mission control for Husky ground robot operations.
 */
class Controller : public rclcpp::Node
{
public:
    Controller();
    virtual ~Controller();

    // Core functionality
    virtual bool reachGoal(void) = 0;
    virtual GoalStats calcNewGoal(void) = 0;
    virtual bool checkOriginToDestination(geometry_msgs::msg::Pose origin, 
                                        geometry_msgs::msg::Point goalPosition,
                                        double& distance, double& time,
                                        geometry_msgs::msg::Pose& estimatedGoalPose) = 0;

    // Goal management
    void setGoal(const geometry_msgs::msg::PoseArray& msg);
    bool goalReached();
    bool setTolerance(double tolerance);
    GoalStats getGoalStats(void);
    geometry_msgs::msg::Pose getOdometry(void);

    // Status and info
    double status();
    std::string getInfoString();

protected:
    // ROS communication
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool mission_active_;

    // Callbacks
    void laserCallback(const sensor_msgs::msg::LaserScan& msg);
    void odoCallback(const nav_msgs::msg::Odometry& msg);
    void control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> res);
    void timerCallback();

    // Member variables
    std::vector<GoalStats> goals_;
    std::vector<double> segment_distances_;
    std::unique_ptr<LaserProcessing> laserProcessingPtr_;
    
    geometry_msgs::msg::Pose pose_;
    sensor_msgs::msg::LaserScan laserData_;
    
    bool goalSet_;
    bool laserDataReceived_;
    unsigned int current_goal_idx_;
    double tolerance_;
    double total_mission_distance_;
    double completed_mission_distance_;
    
    PlatformStatus status_;

    // Mutexes for thread safety
    std::mutex goalMtx_;
    std::mutex poseMtx_;
    std::mutex laserMtx_;
};

#endif // CONTROLLER_HUSKY_H