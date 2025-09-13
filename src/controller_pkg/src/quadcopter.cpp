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

#include "controller_pkg/controller.h"
#include "controller_pkg/quadcopter.h"
// #include "laserprocessing.h"
// #include <pfms_types.h>

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
 * @brief Constructor initializes quadcopter parameters and ROS publishers
 * @details Sets up publishers for velocity commands, takeoff/landing signals,
 * and detection visualization
 */
Quadcopter::Quadcopter() :  
    Controller(), //initialise base class first
    liftoff_(false),TARGET_SPEED(0.4),
    TARGET_HEIGHT_TOLERANCE(0.2),
    target_angle_(0.0)
{
    // We set tolerance to be default of 0.5
    tolerance_=0.5;

    // Initialise publishers
    pubCmdVel_  = this->create_publisher<geometry_msgs::msg::Twist>("drone/cmd_vel",3);  
    pubTakeOff_ = this->create_publisher<std_msgs::msg::Empty>("drone/takeoff",3);  
    pubLanding_ = this->create_publisher<std_msgs::msg::Empty>("drone/land",3);  
    PubDoor_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/drone/detected_doors",3);  
    PubHuman_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/drone/detected_humans",3);  

    // Initialize control loop timer
    timer_ = this->create_wall_timer(100ms, std::bind(&Quadcopter::reachGoal, this));
}

/**
 * @brief Destructor ensures safe shutdown of quadcopter systems
 * @details For safe landing if airborn, cleanup of ROS publishers
 */
Quadcopter::~Quadcopter() {
    // Ensure safe landing if airborne
    if (liftoff_) {
        sendLanding();
    }

    // Stop all motion
    if (pubCmdVel_) {
        sendCmd(0, 0, 0, 0);
    }

    // Reset all publishers
    pubCmdVel_.reset();
    pubTakeOff_.reset();
    pubLanding_.reset();
    PubDoor_.reset();
    PubHuman_.reset();
    timer_.reset();
}

/**
   * @brief Checks if path to destination is feasible, involves laser detection
   * @param origin Current pose of the quadcopter
   * @param goalPosition Target position to reach
   * @param[out] distance Calculated distance to goal
   * @param[out] time Estimated time to reach goal
   * @param[out] estimatedGoalPose Expected pose at goal
   * @return true if path is clear, false if blocked
   */
bool Quadcopter::checkOriginToDestination(geometry_msgs::msg::Pose origin, 
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
    // as we are not rotating the platform, simple moving it left/right and fwd/backward
    estimatedGoalPose.position.x = goalPosition.x;
    estimatedGoalPose.position.y = goalPosition.y;
    // estimatedGoalPose.orientation = origin.orientation; // - How do we deal with yaw to quaternion?
 
    // local view angle
    double robotYaw = tf2::getYaw(origin.orientation);
    double angleLocal = angle - robotYaw;

    // wrap into [–π, +π):
    angleLocal = std::fmod(angleLocal, CLAMP_VAL);
    if (angleLocal >= M_PI)      angleLocal -= CLAMP_VAL;
    else if (angleLocal < -M_PI) angleLocal += CLAMP_VAL;

    // laser processing in here will be used to detect humans and obstacles
    // if there is door publish /detect_door topic and get into the room to detect human or cupboard
    // if there is human publish /detect_human topic
    // then move to the next goal (for now i will add the goal myself so middle room goal will be added manually)
    // same as next tier and middle room

    // then ask this and adjust 
    // if goal is provided by them, i will need to detect wall and position quad to the middle room
    // according to the coordinates, also adjust the angle turning 90 deg

    // i also need checkobstruction from laser to see if goal is reachable, if there is obstruction, return false
    // then setgoal will move on to the next goal, so setGoal will have chekOTD (refer to ackerman)
    

    // implementing laser detection in here 
    // if laser data is received
    if (laserDataReceived_) {

        // use laser to check if there is obstruction, parse in tardet distance and angle
        bool obstacleExist = laserProcessingPtr_->checkObstruction(distance, angleLocal);

        // use later to detect if human exists
        auto detectedHumans = laserProcessingPtr_->detectHumans(); 

        // auto detectedDoors = laserProcessingPtr_->detectDoors(); 
        
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
                humanPose.position.z = human.position.z;
                humanPose.orientation.w = 1.0; // No rotation

                // accumulate all pose to pose array
                humansArray.poses.push_back(humanPose);
                // std::cout << "CHECK VISUAL SHOULD BE PUBLISHING HUMANS" << std::endl;
            }
            // publish visual marker of the detected human pose array
            PubHuman_->publish(humansArray);
        }
    }

    return true;
}

/**
   * Instructs the underlying platform to recalcuate a goal, and set any internal variables as needed
   *
   * Called when goal or tolerance changes
   * @return Whether goal is reachable by the platform
   */
// check if current goal index is reachable by checkOriginToDestination
// if goal is reachable, set goal, if goal is not reachable, set to the next goal
GoalStats Quadcopter::calcNewGoal(void) {
    
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

        // flag if checkOriginToDestiantion return true or false
        bool reachable = checkOriginToDestination(pose, currentGoal.position, distance, time, est_final_pose);

        // if goal is rechable
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
 * @brief Sends movement commands to quadcopter
 * @param turn_l_r Angular velocity around Z axis
 * @param move_l_r Linear velocity along Y axis
 * @param move_u_d Linear velocity along Z axis
 * @param move_f_b Linear velocity along X axis
 */
void Quadcopter::sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b) {
    geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();

    // parse in data subscribed to movement
    msg.linear.x= move_f_b;
    msg.linear.y= move_l_r;
    msg.linear.z= move_u_d;
    msg.angular.z = turn_l_r;

    // publish movement to cmdVel
    pubCmdVel_->publish(msg);
}

/**
 * @brief Initiates takeoff sequence
 */
void Quadcopter::sendTakeOff(void) {
    std_msgs::msg::Empty msg;
    // publish movement to cmdVel
    pubTakeOff_->publish(msg);
}

/**
 * @brief Initiates landing sequence
 */
void Quadcopter::sendLanding(void) {
    std_msgs::msg::Empty msg;
    // publish movement to cmdVel
    pubLanding_->publish(msg);
}

/**
 * @brief Timer callback that implements goal-reaching behavior
  Reach reach goal - execute control to reach goal, blocking call until goal reached or abandoned
  @return goal reached (true - goal reached, false - goal abandoned : not reached)
    @details this function takes in a vector of goals and reach all goals in the vector within tolerance and time
  */
 bool Quadcopter::reachGoal(void) {

    switch(status_){
        // if idle
        case pfms::PlatformStatus::IDLE:
            // if goal is set, set status run
            if(goalSet_){
                status_=pfms::PlatformStatus::RUNNING;
            }
            return false;
        // if takeoff
        case pfms::PlatformStatus::TAKEOFF:
            // publish takeoff, publish takeoff
            sendTakeOff();
            // if goal is set, keep running
            if(goalSet_){
                status_=pfms::PlatformStatus::RUNNING;
            }
            else{
                // if goal is not set, status idle
                status_=pfms::PlatformStatus::IDLE;
            }
            return true;
            // if land, send publish land
        case pfms::PlatformStatus::LANDING:
            sendLanding();
            // then turn idle
            status_=pfms::PlatformStatus::IDLE;
            return true;
        case pfms::PlatformStatus::RUNNING:
            break;
    }

    if(!goalSet_){return false;};

    // goalStats updates new goal from calcNewGoal, checkOTD with updated laserScan data - if there is obstruction
    // account for any drift between setGoal call and now, by getting odo and angle to drive in
    GoalStats goalStats = calcNewGoal(); 
    std::cout << "GOALSTAT: " << goalStats.position.x << ", " << goalStats.position.y << ", " << goalStats.position.z << ", " << std::endl;

    // update odometry
    geometry_msgs::msg::Pose pose = getOdometry();

    // Get relative target angle
    double theta = tf2::getYaw(pose.orientation) - target_angle_;

    // Move at `speed` in target direction
    double dx = TARGET_SPEED * std::cos(theta);
    double dy = TARGET_SPEED * std::sin(theta);

    //What about the height?
    double dz=0;

    // Proportional controller for dz
    double error = goalStats.position.z - pose.position.z;
    double Kp = 0.5; // Proportional gain
    dz = Kp * error;

    // Clip dz to range [-1.0, 1.0]
    if (dz > 1.0) {
        dz = 1.0;
    } else if (dz < -1.0) {
        dz = -1.0;
    }

    //Let's send command with these parameters
    sendCmd(0, -dy, dz, dx);  

    // set flag to check if goal is reached
    bool reached = goalReached();  

    // if goal is reached
    if(reached){
        // if there are more goals, move to the next goal
        // std::lock_guard<std::mutex> lock(goalMtx_);
        if (current_goal_idx_ < goals_.size() - 1) {
            current_goal_idx_++;    
            // std::cout << "CURRENT_GOAL_IDX: " << current_goal_idx_ << " GOALS.SIZE: " << goals_.size() << std::endl;
            // std::cout << "MOVE TO NEXT GOAL " << current_goal_idx_ << " should hover btw" << std::endl;
            sendCmd(0, 0, 0, 0); 
        }
        // if all goals are reached and no goals left
        else {
            // if all goals are reached 
            goalSet_=false;
            // Send the quadcopter to land
            sendCmd(0, 0, -1.0, 0);
            std::cout << "ALL GOALS REACHED!!!" << std::endl;
            sendCmd(0, 0, 0, 0); 
            // Clip dz to range [-1.0, 1.0]
            if (dz > 1.0) {
                dz = 1.0;
            } else if (dz < -1.0) {
                dz = -1.0;
            }
            return true;
        }
    }
    // if goal is not reached, continue moving
    else {
        //Let's send command with these parameters
        sendCmd(0, -dy, dz, dx);
        // RCLCPP_INFO_STREAM(get_logger(),"GOAL NOT REACHED YET" << dx << " " << -dy << " " << dz);
    }

    return reached;
}

/**
 * @brief Service callback for controlling quadcopter mission
 * @param req Service request containing control command
 * @param res Service response indicating success/failure
 */
void Quadcopter::control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,std::shared_ptr<std_srvs::srv::SetBool::Response> res){

    // if received request data from service
    if (req->data)
    {
        // if goal is set
        if (goalSet_) 
        {
            // set status takeoff
            status_ = pfms::PlatformStatus::TAKEOFF;
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
                res->message = "Person spotted! Takeoff OK.";
              } 
            // otherwise no spotted  
            else {
                res->message = "No person detected at current location.";
              }

            // send status response
            res->message = "Mission completion: " + std::to_string(percentageCompletion) + "%";
        }
        else
        {
            res->success = false;
            res->message = "No goal set. Cannot TAKEOFF.";
        }
    }
    else
    {
        status_ = pfms::PlatformStatus::LANDING;
        // call for statuss
        double percentageCompletion = status();
        // check for humans before landing (optional—depends on your requirements)
        auto humans = laserProcessingPtr_->detectHumans();

        // set flag human detected or not
        bool person_visible = !humans.empty();
        // send data to res
        res->success = person_visible;

        if (person_visible) {
        res->message = "Landing initiated, but person still in view!";
        } 
        else {
        res->message = "Landing initiated. No person detected.";
        }

        // send status response
        res->message = "Landing initiated. Mission completion: " + std::to_string(percentageCompletion) + "%";
    }
}