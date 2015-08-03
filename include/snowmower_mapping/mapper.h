/* mapper.h */
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <string>

#ifndef __MAPPER_H_INCLUDED__
#define __MAPPER_H_INCLUDED__

class Mapper {
 private:
  // Parameters
  double mapWidth_;  // Map Width in meters
  double mapHeight_; // Map Height in meters

  double ppm_;       // pixels per meter

  int numCols_;      // Number of columns in the OccupancyGrid
  int numRows_;      // Number of rows in the OccupancyGrid

  double r_;         // Radius of the footprint in meters

  std::string base_frame_; // Frame of the robot
  std::string map_frame_;  // Frame of the map

  // Member Variables
  // penDown_ determines whether the map is marked or not. The map is marked when penDown_ == true and not marked when penDown_ == false
  bool penDown_;      // Boolean value to indicate whether the robot is marking

  geometry_msgs::Pose lastPose_; // Stores the last pose of the robot

  nav_msgs::OccupancyGrid mowed_map_; // Stores the map of the mowed area

  // Member Fucntions
  void odomCB(const nav_msgs::Odometry& msg); // This is the callback function that runs when a nav_msgs/Odometry message is published to the odom topic. The callback function populates the OccupancyGrid message with the new area and publishes it. In the function the percent mowed is also calculated.

  void init(); // Used to initialize the parameters and an empty map

  ros::NodeHandle public_nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher occupancyGrid_pub_;
  ros::Subscriber odom_sub_;

 public:
  Mapper();
  ~Mapper();

  // penDown_ determines whether the map is marked or not. The map is marked when penDown_ == true and not marked when penDown_ == false
  void penDown(); // Changes penDown_ to true
  void penUp();   // Changes penDown_ to false

};

#endif
