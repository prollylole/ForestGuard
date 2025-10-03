/**
 * @file husky.cpp
 * @brief Implementation of the Husky derived class for ground robot control
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
#include "husky.h"
#include "laserprocessing.h"
#include <pfms_types.h>

#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>

#define DEBUG 1
#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_DEBUG RCUTILS_LOG_DEBUG

using std::cout;
using std::endl;
using namespace std::chrono_literals;

/**
 * @brief Constructor initializes Husky parameters and ROS publishers
 * @details Sets up publishers for velocity commands and detection visualization
 */
Husky::Husky() :  
    Controller(), // initialize base class first
    TARGET_SPEED(0.4),
    target_angle_(0.0)
{
    // We set tolerance to be default of 0.5
    tolerance_=0.5;

    // Initialize publishers
    pubCmdVel_  = this->create_publisher<geometry_msgs::msg::Twist>("husky/cmd_vel", 3);  
    PubHuman_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/husky/detected_humans", 3);  

    // Initialize control loop timer
    timer_ = this->create_wall_timer(100ms, std::bind(&Husky::reachGoal, this));
}

/**
 * @brief Destructor ensures safe shutdown of Husky systems
 */
Husky::~Husky() {
    // Stop all motion
    if (pubCmdVel_) {
        sendCmd(0, 0);
    }

    // Reset all publishers
    pubCmdVel_.reset();
    PubHuman_.reset();
    timer_.reset();
}

/**
 * @brief Checks if path to destination is feasible, involves laser detection
 * @param origin Current pose of the Husky
 * @param goalPosition Target position to reach
 * @param[out] distance Calculated distance to goal
 * @param[out] time Estimated time to reach goal
 * @param[out] estimatedGoalPose Expected pose at goal
 * @return true if path is clear, false if blocked
 */
bool Husky::checkOriginToDestination(geometry_msgs::msg::Pose origin, 
    geometry_msgs::msg::Point goalPosition,
    double& distance, double& time,
    geometry_msgs::msg::Pose& estimatedGoalPose) {
    double CLAMP_VAL = 2.0*M_PI;

    // Use pythagorean theorem to get direct distance to goal
    double dx = goalPosition.x - origin.position.x;
    double dy = goalPosition.y - origin.position.y;

    // world view angle
    double angle = atan2(dy, dx);

    // distance to goal and time to goal
    distance = std::hypot(dx, dy);
    time = distance / TARGET_SPEED;
    distance_toGoal_ = distance;

    // The estimated goal pose would be the goal, at the angle we had at the origin
    estimatedGoalPose.position.x = goalPosition.x;
    estimatedGoalPose.position.y = goalPosition.y;
 
    // local view angle
    double robotYaw = tf2::getYaw(origin.orientation);
    double angleLocal = angle - robotYaw;

    // wrap into [–π, +π):
    angleLocal = std::fmod(angleLocal, CLAMP_VAL);
    if (angleLocal >= M_PI)      angleLocal -= CLAMP_VAL;
    else if (angleLocal < -M_PI) angleLocal += CLAMP_VAL;

    // implementing laser detection in here 
    // if laser data is received
    if (laserDataReceived_) {

        // use laser to check if there is obstruction, parse in target distance and angle
        bool obstacleExist = laserProcessingPtr_->checkObstruction(distance, angleLocal);

        // use laser to detect if human exists
        auto detectedHumans = laserProcessingPtr_->detectHumans(); 
        
        // if there is obstacle, goal is not reachable
        if (obstacleExist) {
            return false;
        }

        // if detect human return value in vector
        if (!detectedHumans.empty()) {

            // store human data into a pose array
            auto humansArray = geometry_msgs::msg::PoseArray();
            // tell RVIZ to frame the door in the world frame
            humansArray.header.frame_id = "world";
            // update visual marker after lifetime ends
            humansArray.header.stamp = this->now();

            // for each human detected
            for (auto& human : detectedHumans) {

                // update human data into a pose
                geometry_msgs::msg::Pose humanPose;
                humanPose.position.x = human.position.x;
                humanPose.position.y = human.position.y;
                humanPose.position.z = 0.0; // Ground level for Husky
                humanPose.orientation.w = 1.0; // No rotation

                // accumulate all pose to pose array
                humansArray.poses.push_back(humanPose);
            }
            // publish visual marker of the detected human pose array
            PubHuman_->publish(humansArray);
        }
    }

    return true;
}

/**
 * Instructs the underlying platform to recalculate a goal, and set any internal variables as needed
 *
 * Called when goal or tolerance changes
 * @return Whether goal is reachable by the platform
 */
GoalStats Husky::calcNewGoal(void) {
    
    // check if there is goal is not available, return empty array
    if (!goalSet_ || goals_.empty()) {
        return GoalStats{};
    }

    // update current odometry
    geometry_msgs::msg::Pose pose = getOdometry();

    // going over all the available goals
    while ((current_goal_idx_) < goals_.size()) {

        // set current goal individually
        GoalStats currentGoal = goals_.at(current_goal_idx_);
        
        double distance, time;
        geometry_msgs::msg::Pose est_final_pose;

        // flag if checkOriginToDestination return true or false
        bool reachable = checkOriginToDestination(pose, currentGoal.position, distance, time, est_final_pose);

        // if goal is reachable
        if (reachable) {

            // Calculate absolute travel angle required to reach goal
            double dx = currentGoal.position.x - pose.position.x;
            double dy = currentGoal.position.y - pose.position.y;
            target_angle_ = std::atan2(dy, dx);
            return currentGoal;
        }
        
        // if goal is unreachable, skip to the next index
        std::cout << "GOAL IS BLOCKED" << std::endl;
        // increment index to next goal index
        current_goal_idx_++;
        std::cout << "GOAL INDEX: " << current_goal_idx_ << std::endl;
    }

    // all goals_.size() has been visited, flag there is no more goal
    goalSet_ = false;

    // return empty array
    return GoalStats{};
}

/**
 * @brief Sends movement commands to Husky
 * @param linear_x Linear velocity along X axis
 * @param angular_z Angular velocity around Z axis
 */
void Husky::sendCmd(double linear_x, double angular_z) {
    geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();

    // parse in data subscribed to movement
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;

    // publish movement to cmdVel
    pubCmdVel_->publish(msg);
}

/**
 * @brief Timer callback that implements goal-reaching behavior
 * Reach goal - execute control to reach goal, blocking call until goal reached or abandoned
 * @return goal reached (true - goal reached, false - goal abandoned : not reached)
 * @details this function takes in a vector of goals and reach all goals in the vector within tolerance and time
 */
bool Husky::reachGoal(void) {

    switch(status_){
        // if idle
        case pfms::PlatformStatus::IDLE:
            // if goal is set, set status run
            if(goalSet_){
                status_=pfms::PlatformStatus::RUNNING;
            }
            return false;
        case pfms::PlatformStatus::RUNNING:
            break;
        default:
            return false;
    }

    if(!goalSet_){return false;};

    // goalStats updates new goal from calcNewGoal, checkOTD with updated laserScan data - if there is obstruction
    // account for any drift between setGoal call and now, by getting odo and angle to drive in
    GoalStats goalStats = calcNewGoal(); 
    std::cout << "GOALSTAT: " << goalStats.position.x << ", " << goalStats.position.y << std::endl;

    // update odometry
    geometry_msgs::msg::Pose pose = getOdometry();

    // Get relative target angle
    double current_yaw = tf2::getYaw(pose.orientation);
    double angle_error = target_angle_ - current_yaw;

    // Normalize angle error to [-pi, pi]
    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

    // Calculate distance to goal
    double dx = goalStats.position.x - pose.position.x;
    double dy = goalStats.position.y - pose.position.y;
    double distance_to_goal = std::sqrt(dx*dx + dy*dy);

    // Simple proportional controller for Husky
    double linear_vel = 0.0;
    double angular_vel = 0.0;

    // If we're close to the goal, stop
    if (distance_to_goal < tolerance_) {
        linear_vel = 0.0;
        angular_vel = 0.0;
    } else {
        // Only move forward if we're roughly facing the goal
        if (std::abs(angle_error) < M_PI/4) { // Within 45 degrees
            linear_vel = TARGET_SPEED * std::min(1.0, distance_to_goal / 2.0);
        }
        // Turn toward goal
        angular_vel = 2.0 * angle_error; // P-controller for angle
    }

    // Send command
    sendCmd(linear_vel, angular_vel);

    // set flag to check if goal is reached
    bool reached = goalReached();  

    // if goal is reached
    if(reached){
        // if there are more goals, move to the next goal
        if (current_goal_idx_ < goals_.size() - 1) {
            current_goal_idx_++;    
            std::cout << "MOVE TO NEXT GOAL " << current_goal_idx_ << std::endl;
            sendCmd(0, 0); // Stop briefly between goals
        }
        // if all goals are reached and no goals left
        else {
            // if all goals are reached 
            goalSet_=false;
            sendCmd(0, 0); // Stop completely
            std::cout << "ALL GOALS REACHED!!!" << std::endl;
            return true;
        }
    }

    return reached;
}

/**
 * @brief Service callback for controlling Husky mission
 * @param req Service request containing control command
 * @param res Service response indicating success/failure
 */
void Husky::control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> res){

    // if received request data from service
    if (req->data)
    {
        // if goal is set
        if (goalSet_) 
        {
            // set status running
            status_ = pfms::PlatformStatus::RUNNING;
            // calculate status
            double percentageCompletion = status();
            // perform detect human
            auto humans = laserProcessingPtr_->detectHumans();

            // flag if there is human
            bool person_visible = !humans.empty();
            // send flag to response
            res->success = person_visible; 
            // if there is human send string detected
            if (person_visible) {
                res->message = "Person spotted! Mission started.";
            } 
            // otherwise no spotted  
            else {
                res->message = "No person detected at current location.";
            }

            // send status response
            res->message += " Mission completion: " + std::to_string(percentageCompletion) + "%";
        }
        else
        {
            res->success = false;
            res->message = "No goal set. Cannot start mission.";
        }
    }
    else
    {
        status_ = pfms::PlatformStatus::IDLE;
        // call for status
        double percentageCompletion = status();
        // check for humans before stopping
        auto humans = laserProcessingPtr_->detectHumans();

        // set flag human detected or not
        bool person_visible = !humans.empty();
        // send data to res
        res->success = person_visible;

        if (person_visible) {
            res->message = "Mission stopped, but person still in view!";
        } 
        else {
            res->message = "Mission stopped. No person detected.";
        }

        // send status response
        res->message += " Mission completion: " + std::to_string(percentageCompletion) + "%";
        
        // Stop the robot
        sendCmd(0, 0);
    }
}