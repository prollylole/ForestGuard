/**
 * @file controller_husky.cpp
 * @brief Implementation of the Controller base class for Husky ground robot control
 * 
 * This class serves as the foundation for Husky robot controllers, providing:
 * - Goal management and tracking
 * - Sensor data processing (laser, odometry)
 * - Mission control through ROS services
 * - Status monitoring and reporting
 */

#include "husky_controller/controller_husky.h"
#include "husky_controller/laserprocessing.h"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <cmath>
#include <iostream>

using std::placeholders::_1;

/**
 * @brief Constructor initializes core robot control infrastructure
 * @details Sets up essential ROS communication channels:
 * The subscribers and service are initialized in constructor to ensure
 * the robot can immediately begin receiving sensor data and commands
 */
Controller::Controller()  : 
  Node("husky_node"), 
  goalSet_(false),    
  current_goal_idx_(0),
  tolerance_(0.5),
  laserDataReceived_(false),
  status_(IDLE) 
{
  // Initialize laserProcessingPtr_ 
   laserProcessingPtr_ = std::make_unique<LaserProcessing>();

  // Laser subscriber: Processes environmental data for obstacle detection
   laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&Controller::laserCallback,this,std::placeholders::_1));    
   
   // Goal subscriber: Receives navigation waypoints
   goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/goal_pose", 10, std::bind(&Controller::setGoal,this,std::placeholders::_1)); 
         
   // Odometry subscriber: Tracks robot position and movement
   odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
       "/odometry/filtered", 10, std::bind(&Controller::odoCallback,this,std::placeholders::_1));
   
   // Mission service: Handles external control commands
   service_ = this->create_service<std_srvs::srv::SetBool>(
       "/husky/mission", std::bind(&Controller::control,this,std::placeholders::_1, std::placeholders::_2)); 
}

/**
 * @brief Virtual destructor ensures proper cleanup of derived classes
 * @details Cleans up any dynamic resources and ensures safe shutdown
 */
Controller::~Controller()
{
    // Clean up any remaining goals
    goals_.clear();
    
    // Reset laser processing
    laserProcessingPtr_.reset();
    
    // Reset all subscribers and service
    laser_sub_.reset();
    goal_sub_.reset();
    odom_sub_.reset();
    service_.reset();
}

/**
 * @brief Sets acceptable distance tolerance for reaching goals
 * @param tolerance Distance in meters
 * @return true if tolerance was set successfully
 */
bool Controller::setTolerance(double tolerance) {
  tolerance_ = tolerance;
  return true;
}

/**
 * @brief Gets current goal information
 * @return GoalStats structure containing goal position and orientation
 */ 
GoalStats Controller::getGoalStats(void) {
  std::lock_guard<std::mutex> lock(goalMtx_);
  return goals_.at(current_goal_idx_);
}

/**
 * Checks if the goal has been reached.
 *
 * Update own pose before calling!
 * @return true if the goal is reached
 */
bool Controller::goalReached() {
  std::lock_guard<std::mutex> lock(goalMtx_);

  // if there is no goal or goal is not set, goal not reached
  if (!goalSet_ || goals_.empty()) return false;

  // otherwise, set to current goal index from heaps of goals
  GoalStats current_goal = goals_.at(current_goal_idx_);  
  geometry_msgs::msg::Pose pose = getOdometry();

  double dx = current_goal.position.x - pose.position.x;
  double dy = current_goal.position.y - pose.position.y;

  // current distance from updated odometry (2D only for ground robot)
  double distance = std::sqrt(dx*dx + dy*dy);

  // if distance to goal is within threshold, goal is reached
  if (distance < tolerance_) {
    // when goal is reached, update distance travelled accumulated (keep track)
    completed_mission_distance_ += segment_distances_.at(current_goal_idx_);
    return true;
  }
  return false;
}

/**
 * @brief Gets current odometry reading
 * @return Current pose of the platform
 */
geometry_msgs::msg::Pose Controller::getOdometry(void){
  std::lock_guard<std::mutex> lock(poseMtx_);
  return pose_;
}

/**
 * This callback creates a new laserProcessingPtr object if it does not exist
 * or updates the scan in the existing object
 */
void Controller::laserCallback(const sensor_msgs::msg::LaserScan& msg)
{
  std::lock_guard<std::mutex> lock(laserMtx_);
  // Store latest scan
  laserData_ = msg; 
  // set laser received flag
  laserDataReceived_ = true;

  //If we have not created the laserProcessingPtr object, create it
  if (!laserProcessingPtr_) {
     RCLCPP_WARN(this->get_logger(), "LaserProcessingPtr should never be null now!");
     laserProcessingPtr_ = std::make_unique<LaserProcessing>();
  }
  // update laserscan data
  std::cout << "DEBUG: Updating laser scan" << std::endl;
  laserProcessingPtr_->newScan(laserData_);
}

/**
 * @brief Sets new navigation goal
 * @param msg Array of poses representing goals
 */
void Controller::setGoal(const geometry_msgs::msg::PoseArray& msg){    
  std::lock_guard<std::mutex> lock(goalMtx_);

  // clear the previous goals before adding new ones
  goals_.clear();
  segment_distances_.clear();

  // reset index
  current_goal_idx_ = 0;
  total_mission_distance_ = 0.0;
  completed_mission_distance_ = 0.0;

  // update odometry, only used once for distance calculating at the very first goal
  geometry_msgs::msg::Pose current_pose = getOdometry();

  // for each goal 
  for (const auto& pose : msg.poses) {
    // allocate goal data according to the goal subscribed
    GoalStats goal;
    goal.position = pose.position;
    goal.orientation = pose.orientation;

    // accumulate each goal into vector of goals
    goals_.push_back(goal);
    RCLCPP_INFO(get_logger(), "Received goal at (%f, %f)", goal.position.x, goal.position.y);

    // calculate distance from goal to odometry (2D only for ground robot)
    double dx = goal.position.x - current_pose.position.x;
    double dy = goal.position.y - current_pose.position.y;
    // distance to goal
    double distance = std::sqrt(dx*dx + dy*dy);

    // Store segment distance and update total    
    segment_distances_.push_back(distance);    
    // update next distance to goal in this loop 
    total_mission_distance_ += distance; 

    // Update current pose for next segment calculation
    current_pose.position = goal.position;
    current_pose.orientation = goal.orientation;
  }

  // if goal is set, print out amount of goals
  goalSet_ = !goals_.empty();
  RCLCPP_INFO(get_logger(), "Received %zu goals", goals_.size());
}

/**
 * @brief Callback for processing odometry data
 * @param msg Odometry message from ROS topic
 */
void Controller::odoCallback(const nav_msgs::msg::Odometry& msg){
  std::lock_guard<std::mutex> lock(poseMtx_);
  pose_ = msg.pose.pose;
}

/**
 * @brief Gets formatted status string for debugging
 * @return String containing current controller status
 */
std::string Controller::getInfoString()
{
    std::stringstream ss;
    switch(status_)
    {
        // Platform waiting for commands
        case IDLE   : ss << "IDLE ";    break;

        // Platform actively executing goals
        case RUNNING : ss << "RUNNING ";  break;

        // Platform under teleoperation control
        case TELEOP : ss << "TELEOP ";  break;
    }
    return ss.str(); // This command converts stringstream to string
}

double Controller::status() 
{
  // percentCompletion = ((distance travelled previous goals + 
  //                      distance to current goal* distance travelled decimal progress to current goal))/
  //                      distance to all goals
  // distance travelled decimal progress to current goal = 
  //      1- (distance to current goal/ estimated initial distance from pre goal to current goal)
  
  // if goal is not set or no goal return 0
  if (!goalSet_ || goals_.empty()) {
    return 0.0;  
  }

  // If all goals completed
  if (current_goal_idx_ >= goals_.size()) {
    return 100.0;
  }
  
  // Calculate progress for current goal
  geometry_msgs::msg::Pose pose = getOdometry();
  // using current goal index
  GoalStats current_goal = goals_.at(current_goal_idx_);
  
  double dx = current_goal.position.x - pose.position.x;
  double dy = current_goal.position.y - pose.position.y;
  // distance to current goal (2D only)
  double distToCurrGoal = std::sqrt(dx*dx + dy*dy);

  // Get current segment length
  double segment_length = segment_distances_.at(current_goal_idx_);

  // Calculate progress on current segment (in decimal)
  double segment_progress = 0.0;

  // avoid division by 0
  if (segment_length > 0.001) { //NEAR ZERO VALUE

    // distance travelled decimal progress to current goal = 
    //      1- (distance to current goal/ estimated initial distance from pre goal to current goal)
    segment_progress = 1.0 - (distToCurrGoal / segment_length);
    segment_progress = std::min(100.0, std::max(0.0, segment_progress));  
  }

  // percentCompletion = ((distance travelled previous goals + 
  //                      distance to current goal* distance travelled decimal progress to current goal))/
  //                      distance to all goals
  double completed_distance = completed_mission_distance_ + (segment_length * segment_progress);

  // completedTrack/ total distance to all goals * 100 is the final value
  double percentCompletion = (completed_distance / total_mission_distance_)* 100.0;
  
  return std::min(100.0, std::max(0.0, percentCompletion));
}

/**
 * @brief Service callback for controlling Husky mission
 * @param req Service request containing control command
 * @param res Service response indicating success/failure
 */
void Controller::control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> res) {

    // Check if laser processing is available
    // if (!laserDataReceived_) {
    //    res->success = false;
    //    res->message = "Waiting for first laser scan...";
    //    RCLCPP_WARN(this->get_logger(), "Mission requested before laser scan arrived");
    //    return;
    // }

    // perform detect human
    // auto humans = laserProcessingPtr_->detectHumans();
    // // flag if there is human
    // bool person_visible = !humans.empty();

    // calculate status
    double percentageCompletion = status();

    std::string message;

    // if received request data from service
    if (req->data) {
        // if goal is set
        if (goalSet_) {
            // set status running
            status_ = RUNNING;
            // message = person_visible ? "Person spotted! Mission started. " 
                                    //  : "No person detected at current location. ";
        }
        else {
            status_ = IDLE;
            message = "No goal set. Cannot start mission.";
        }

        message += "Mission completion: " + std::to_string(percentageCompletion) + "%";
        res->message = message;
    } 
    else {
        status_ = IDLE;
        // message = person_visible ? "Mission stopped, but person still in view! "
        //                          : "Mission stopped. No person detected. ";
        
        message += "Mission completion: " + std::to_string(percentageCompletion) + "%";
    }    
    
    // res->success = person_visible;
    res->message = message;
  }