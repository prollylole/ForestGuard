#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

/**
 * @brief Library for processing laser scan data to detect objects and features
 * @details This library provides functionality for:
 *  - Processing raw laser scan data
 *  - Detecting doors (2m wide gaps between walls)
 *  - Detecting humans (non-aligned segments < 1.8m)
 *  - Checking for obstructions in navigation paths
 *  - Converting between polar and cartesian coordinates
 */

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <math.h>
#include <mutex>


/**
 * @brief Class for processing laser scan data to detect objects, doors, and humans
 * @details Provides functionality to:
 *  - Detect doors in walls based on gap analysis
 *  - Detect humans based on segment shape analysis
 *  - Check for obstructions in paths
 *  - Track objects and segment detection
 */
class LaserProcessing
{
public:

  /**
   * @brief Structure representing a detected door
   */
  struct Door {
    geometry_msgs::msg::Point center;///< Center point of door
    double width;///< Width of door opening
  };

  struct SegmentResult{
    unsigned int count;
    std::vector<std::pair<double, double>> centroids;
  };

  /**
   * @brief Constructor initializes laser processing with initial scan
   * @param laserScan Initial laser scan data
   */
  LaserProcessing(sensor_msgs::msg::LaserScan laserScan);

  /**
   * @brief Detects doors in the environment
   * @param odometry Current pose of the robot
   * @return Vector of detected doors
   */
  std::vector<Door> detectDoors(const geometry_msgs::msg::Pose& odometry); 

  /**
   * @brief Detects humans in the environment
   * @return Vector of poses where humans are detected
   */
  std::vector<geometry_msgs::msg::Pose> detectHumans(); 
  
  /**
   * @brief Checks if there's an obstruction at given angle and distance
   * @param distance Distance to check
   * @param angleLocal Angle to check (modified by function)
   * @return true if obstruction detected
   */
  bool checkObstruction(double& distance, double& angleLocal);

  /*! @brief Count number of readings belonging to objects (not infinity, nan or max range) from the last
  * laser scan provided (either via @sa newScan or @sa LaserProcessing constructor)
  * thread-safe function, internally creates a copy fo laserScan_ 
  *
  * @return the number of laser readings that belong to objects
  */
  SegmentResult countSegments();

  /*! @brief Accepts a new laserScan, threadsafe function
   *  @param[in]    laserScan  - laserScan supplied
   */
  void newScan(sensor_msgs::msg::LaserScan laserScan);


private:
  /*! @brief Returns the cartesian position of laer reading at specific index
   * converted from polar coordinats stored in the #laserScan_
   *  @param[in] index - the reading needing conversion
   *  @return position cartesian values
   */
   geometry_msgs::msg::Point polarToCart(unsigned int index);

private:
  sensor_msgs::msg::LaserScan laserScan_;///< Current laser scan data
  std::mutex mtx; //!< Mutex to protect the laserScan_ from being accessed by multiple threads
  std::vector<Door> detectedDoors_;///< Currently detected doors
  std::vector<geometry_msgs::msg::Pose> detectedHumans_;///< Currently detected humans
  std::vector<std::pair<double,double>> cones_;
};

#endif // LASERPROCESSING_H