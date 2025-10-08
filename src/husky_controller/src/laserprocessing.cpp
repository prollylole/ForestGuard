/**
 * @brief Library for processing laser scan data to detect objects and features
 * @details This library provides functionality for:
 *  - Processing raw laser scan data
 *  - Detecting doors (2m wide gaps between walls)
 *  - Detecting humans (non-aligned segments < 1.8m)
 *  - Checking for obstructions in navigation paths
 *  - Converting between polar and cartesian coordinates
 */

#include "husky_controller/laserprocessing.h"
#include <algorithm>
#include <numeric>
#include <iostream>

#include <tf2/utils.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;

/**
 * @brief Constructor initializes laser processing with first scan
 * @param laserScan Initial laser scan data to process
 */
LaserProcessing::LaserProcessing()
{
    // initialize with empty scan
    laserScan_ = sensor_msgs::msg::LaserScan();
}

// to make detect human will go together with detect cupboard, coming woth the similar concept, 
// but this time we will check if segment >= 6 the drone is inside room
// (drone will be placed in the mid room), for human or cupboard itself is one segment
// for each segment, take the x,y coordinate of the start and mid of segment (world coordinate)
// check if x,y of both coordinates align, if not align == human, if aligns not human

/**
   * @brief Detects humans in the environment
   * @return Vector of poses where humans are detected
   */
  std::vector<geometry_msgs::msg::Pose> LaserProcessing::detectHumans()
  {
      // to detect human, we will fulfill these conditions:
      // 1. make sure drone is inside the room, using countSegment, segment should be > 4 - if
      // by using countSegment, obtain st, mid, en point of a segemnt - for
      // 2. calculate the length of st and en segment, if length > 1.8m break
      // 3. line deviation for flat objects are under 0.1, humans should be > 0.1
      // 4. compare the alignment of x,y 
      //    human has (st x,y != mid x,y) && (mid x,y != en x,y); door has (st x = mid x = en x) || (st y = mid y = en y)
      // detectedHumans_.clear();  // Clear previous detections
    //   unsigned int countSegment = countSegments();
  
      // 1. make sure drone is inside the room, using countSegment, segment should be > 4 - if
    //   if (countSegment < 4) {
    //       return detectedHumans_;  // Drone not in a room
    //   }
  
      std::cout << "DETECT: passed first condition" << std::endl;
  
      std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> startMidCoordinates;
    std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> endMidCoordinates;
    geometry_msgs::msg::Point startCoordinate, midCoordinate, endCoordinate;

    // while laser scan 360
    unsigned int i = 0;
    while (i < laserScan_.ranges.size()) {
        // check if range is valid, not infinity, and within the range limits
        // NOT SURE FOR THE INFINITY HERE, AS IT MIGHT NOT GO TO WHILE LOOP
        if ((laserScan_.ranges.at(i) > laserScan_.range_max) || 
            (laserScan_.ranges.at(i) < laserScan_.range_min) || 
            isnan(laserScan_.ranges.at(i)) || 
            !isfinite(laserScan_.ranges.at(i))) {
                i++;
                continue; // skip to next reading
            }
        
        // find the start and end point of segment
        unsigned int st = i, mid = i, en = i;
        bool countFlag = true;
        while (countFlag && (i < laserScan_.ranges.size())) {

            // if laser reads infinity, segment has ended
            // can potentially read of distance from here for further use: calc multiple distances if object is flat or smth
            if (!isfinite(laserScan_.ranges.at(i))) {
                // flag to stop counting current dot of the segment
                countFlag = false;
            }
            else {
                // mark end of segment at current index
                en = i;
            }
            mid = (st + en)/ 2;
            i++;
        }

        // find coordinates of the start and end point of the segment
        // if end index is greater than start index, we have a segment
        if (en > st) {
            startCoordinate = polarToCart(st);
            midCoordinate = polarToCart(mid);
            endCoordinate = polarToCart(en);

            // segmentCoordinates = (startppoint of dsegment (x,y), endpoint of segment (x,y))
            startMidCoordinates.push_back(std::make_pair(startCoordinate, midCoordinate));
            endMidCoordinates.push_back(std::make_pair(endCoordinate, midCoordinate));
        }

        // if we're not starting st=i, or ending en=i, then we're adding the segment
        if (st != en) { // which, supposably never be, so this is just "a third condition to add segment up"
            // increment segment count
            // count++;
            // std::cout << "count++: adding: " << count << std::endl;
            
            // we have segment now, this is to compute centre of all points in segment
            double xC = 0, yC = 0;
            for (unsigned j = st; j < en; j++) {
                std::cout << "st: " << st << "j: " << j << "en: " << en << std::endl;

                float angle = laserScan_.angle_min + laserScan_.angle_increment * j;
                xC += laserScan_.ranges.at(j-1) * cos(angle);
                yC += laserScan_.ranges.at(j-1) * sin(angle);

                std::cout << "angle: " << angle << "xC: " << xC << "yC: " << yC << std::endl;
            }

            // compute average
            xC = xC/(en-st);
            yC = yC/(en-st);
        }
    }

    std::cout << "DETECT: prior second condition" << std::endl;

    // 2. calculate the length of st and en segment, if length > 1.8m break
    // for each segment
    for (size_t j = 0; j < startMidCoordinates.size(); j++) {
        double ALIGNMENT_THRESHOLD = 0.5;
        double MAX_CUPBOARD_LENGTH = 1.8;
        double MIN_HUMAN_LENGTH = 0.4;

        geometry_msgs::msg::Pose human;

        // std::cout << "segment start: " << segmentCoordinates.at(j).first.x << ", " 
        //             << segmentCoordinates.at(j).first.y << " segment 1 mid: " << segmentCoordinates.at(j).second.x 
        //             << ", " << segmentCoordinates.at(j).second.y << std::endl;

        // to calculate distance, needs st point and en point of segment
        // start point of segment j (first) at startMidCoordinates and en point of segment j (first) anst endMidCoordinates
        double lengthSegment = std::hypot(startMidCoordinates.at(j).first.x - endMidCoordinates.at(j).first.x,
                                            startMidCoordinates.at(j).first.y - endMidCoordinates.at(j).first.y);

        std::cout << "length segment: " << lengthSegment << std::endl;
            
        if ((lengthSegment > MIN_HUMAN_LENGTH) && (lengthSegment < MAX_CUPBOARD_LENGTH)) { //1.8 MAX CUPBOARD LENGTH

            std::cout << "DETECT: passed second condition" << std::endl;
            
            // only human and cupboard fulfill this condition
            // 3. compare the alignment of x,y 
            //    human has (st x,y != mid x,y) && (mid x,y != en x,y); door has (st x = mid x = en x) || (st y = mid y = en y)
            
            // get difference of x,y of start and mid point for alignment check
            double startMidDiffX = fabs(startMidCoordinates.at(j).second.x - startMidCoordinates.at(j).first.x);
            double startMidDiffY = fabs(startMidCoordinates.at(j).second.y - startMidCoordinates.at(j).first.y);
            // get difference of x,y of mid and end point for alignment check
            double endMidDiffX = fabs(endMidCoordinates.at(j).first.x - endMidCoordinates.at(j).second.x);
            double endMidDiffY = fabs(endMidCoordinates.at(j).first.y - endMidCoordinates.at(j).second.y);

            bool is_x_aligned = (startMidDiffX < ALIGNMENT_THRESHOLD) && (endMidDiffX < ALIGNMENT_THRESHOLD);
            bool is_y_aligned = (startMidDiffY < ALIGNMENT_THRESHOLD) && (endMidDiffY < ALIGNMENT_THRESHOLD);
            bool is_aligned = is_x_aligned || is_y_aligned;

            // door has (st x = mid x = en x) || (st y = mid y = en y) 0.5 ALIGNMENT THRESHOLD
            if (!is_aligned) {
                // std::cout << "CUPBOARD DETECTED WOW" << std::endl;
                std::cout << "HUMAN DETECTED!!!!! YURRRRRRRRRRRRRRRRRRRR DODGY CODEEEEEE" << std::endl;
                human.position.x = (startMidCoordinates.at(j).first.x + endMidCoordinates.at(j).first.x)/ 2;
                human.position.y = (startMidCoordinates.at(j).first.y + endMidCoordinates.at(j).first.y)/ 2;
                std::cout << "Detected human in DRONE FRAME: (" 
                << human.position.x << ", " 
                << human.position.y << ")" << std::endl;
                detectedHumans_.push_back(human);
            }
        }
    }
    std::cout << "detected humans: " << detectedHumans_.size() << std::endl;
    // return vector of humans
    return detectedHumans_; 
  }
  
/**
   * @brief Checks if there's an obstruction at given angle and distance
   * @param distance Distance to check
   * @param angleLocal Angle to check (modified by function)
   * @return true if obstruction detected
   */
// takes in disatance and angle returns true if there is an obstruction
// compare the laserscan range according to the angle and distance to goal
bool LaserProcessing::checkObstruction(double& distance, double& angleLocal) 
{
    double CLAMP_VAL = 2.0*M_PI;
    const double ANGLE_TOLERANCE = 5.0 * M_PI/180;  // 5 degrees

    // clamp the lcal angle value
    angleLocal = std::fmod(angleLocal, CLAMP_VAL);
    if (angleLocal >= M_PI)      angleLocal -= CLAMP_VAL;
    else if (angleLocal < -M_PI) angleLocal += CLAMP_VAL;

    int laserIndex = -1;
    // set infinity threshold
    double leastIndex = std::numeric_limits<double>::infinity();

    for (int i = 0; i < laserScan_.ranges.size(); i++) {

        // using the laserScan_.angle_min from RVIZ,
        // find the index of the laserScan range that corresponds to target angle
        double beamAngle = laserScan_.angle_min+ i* laserScan_.angle_increment;
        
        // clamp the laser angle value
        beamAngle = std::fmod(beamAngle, CLAMP_VAL);
        if (beamAngle >= M_PI)      beamAngle -= CLAMP_VAL;
        else if (beamAngle < -M_PI) beamAngle += CLAMP_VAL;

        // calculate target angle
        double difference = std::abs(beamAngle - angleLocal);
        double angleDiff = std::min(difference, CLAMP_VAL - difference);
        
        // get to minimum angle
        if (angleDiff < leastIndex) {
            leastIndex = angleDiff;
            laserIndex = i;
        }
    }
    
    // then get the laserScan_range according to the index as laser distance
    float laserRange = laserScan_.ranges.at(laserIndex);
    std::cout << "laserDist: " << laserRange << " goalDist: " << distance << " angleDiff: " << leastIndex * 180/M_PI << std::endl;

    // quick check if there is obstacle, if infinity, there is no obstacle
    if (!isfinite(laserRange)) {
        return false; 
    }

    // compare laser distance with the distance to goal
    // if laser distance < distance, there is an obstruction
    if ((laserRange < distance) && (leastIndex < ANGLE_TOLERANCE)) {
        std::cout << "THERE IS OBSTRUCTION, NEXT!!" << std::endl;
        return true;
    }
    return false;
}

/*! @brief Count number of readings belonging to objects (not infinity, nan or max range) from the last
  * laser scan provided (either via @sa newScan or @sa LaserProcessing constructor)
  * thread-safe function, internally creates a copy fo laserScan_ 
  *
  * @return the number of laser readings that belong to objects
  */
//  count the amount of segment detected by laser, includes the length of st and en point of a segment, 
// find the coordinate of centroid of a segment
LaserProcessing::SegmentResult LaserProcessing::countSegments()
{
  // reset segment amount and clear centroid
  SegmentResult result;
  result.count = 0;
  result.centroids.clear();

  // if there is cone, update size and set centroids
  if (cones_.size() > 0) {
    result.count = cones_.size();
    result.centroids = cones_;
    return result;
  }

  unsigned int i = 1;
  
  // initialise index, all index gets in here
  while (i < laserScan_.ranges.size()) {
    bool countFlag = true;
    bool init = false; // to initialise the st index of a segment
    unsigned int st = i, en = i;
    double distance = 0;

    // skip to next index if infinity
    if (isinf(laserScan_.ranges.at(i-1))) {
        i++; 
        continue;
    }

    // all finite index gets in here
    while (countFlag && (i < laserScan_.ranges.size())) {

        // if there is infinity index, leave this loop
        if (isinf(laserScan_.ranges.at(i))) {
            countFlag = false;
        }

        // if the laser scan index is finite
        else {
            // angle and coordinate of where prev index hits
            geometry_msgs::msg::Point prev;
            float prevAngle = laserScan_.angle_min + laserScan_.angle_increment*(i-1);
            prev.x = laserScan_.ranges.at(i-1)* cos(prevAngle);
            prev.y = laserScan_.ranges.at(i-1)* sin(prevAngle);
            prev.z = 0;

            // angle and coordinate of where curr index hits
            geometry_msgs::msg::Point curr;
            float currAngle = laserScan_.angle_min + laserScan_.angle_increment*i;
            curr.x = laserScan_.ranges.at(i)* cos(currAngle);
            curr.y = laserScan_.ranges.at(i)* sin(currAngle);
            curr.z = 0;

            // distance from previous index to current index
            distance = sqrt(pow(curr.x - prev.x,2) + pow(curr.y - prev.y, 2));

            // if distance between prev and curr index < 0.3
            if (distance < 0.3) {

                // if new index of segment hasn't been initialised
                if (!init) {
                    // set st index and initialise 
                    st = i;
                    init = true;
                }
            }

            // if distance is more than 0.3
            else {
                // leave this loop once while updated this countFlag
                countFlag = false;
            }
        }

        // just before countFlag false leave this while loop
        if (!countFlag) {
            // we are ending a segment, so set en to current index
            // we will have a complete segment with st and en leaving this loop
            en = i;
        }

        // just before exiting the loop from the current index, add index to the next one
        i++;
    }

    // a segment 
    if (st != en) {
        // increment counter
        result.count++;

        // initialise for centroid
        double xC = 0, yC = 0;
        unsigned int pointCount = 0;

        // for every dot inside a segment, start from st, end at en
        for (unsigned j = st; j < en; j++) {

            // find angle and coordinate of current dot index j
            float angle = laserScan_.angle_min + laserScan_.angle_increment* j;
            xC += laserScan_.ranges.at(j)* cos(angle);
            yC += laserScan_.ranges.at(j)* sin(angle);

            // increment pointCount everytime xC and yC is calculated
            // pointCount is the amount of dots inside a segment
            pointCount++;
        }

        // if there's dots inside a segment
        if (pointCount > 0) {
            // average centroid coordinate of a segment
            xC /= pointCount;
            yC /= pointCount;
            
            // add centroid pair coordinates to result.centroids
            result.centroids.push_back(std::make_pair(xC, yC));
            // add centroid pair coordinates to cones size
            cones_.push_back(std::make_pair(xC, yC));
        }
    }
  }
  
  return result;
}

/*! @brief Accepts a new laserScan, threadsafe function
   *  @param[in]    laserScan  - laserScan supplied
   */
void LaserProcessing::newScan(sensor_msgs::msg::LaserScan laserScan) {
    // std::unique_lock<std::mutex> lck(mtx);
    laserScan_ = laserScan;    
}

/*! @brief Returns the cartesian position of laer reading at specific index
   * converted from polar coordinats stored in the #laserScan_
   *  @param[in] index - the reading needing conversion
   *  @return position cartesian values
   */
geometry_msgs::msg::Point LaserProcessing::polarToCart(unsigned int index)
{
    if (index >= laserScan_.ranges.size()) {
        // return an empty point if index is out of bounds
        return geometry_msgs::msg::Point();
    }
     
    float angle = laserScan_.angle_min + laserScan_.angle_increment*index;// + angle_range/2;
    float range = laserScan_.ranges.at(index);
    geometry_msgs::msg::Point cart;
    cart.x = static_cast<double>(range*cos(angle));
    cart.y = static_cast<double>(range*sin(angle));
    return cart;
}