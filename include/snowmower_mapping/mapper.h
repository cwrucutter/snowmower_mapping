/* mapper.h */
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

#ifndef __MAPPER_H_INCLUDED__
#define __MAPPER_H_INCLUDED__

class Mapper {
 private:
  double mapWidth_;  // Map Width in meters
  double mapHeight_; // Map Height in meters

  double ppm_;       // pixels per meter

  int numCols_;      // Number of columns in the OccupancyGrid
  int numRows_;      // Number of rows in the OccupancyGrid

  double r_;         // Radius of the footprint in meters

  geometry_msgs::Pose lastPose_; // Stores the last pose of the robot

  nav_msgs::OccupancyGrid mowed_map_; // Stores the map of the mowed area

  void odomCB(const nav_msgs::Odometry& msg); // This is the callback function that runs when a nav_msgs/Odometry message is published to the odom topic. The callback function populates the OccupancyGrid message with the new area and publishes it. In the function the percent mowed is also calculated.

  ros::NodeHandle nh_;
  ros::Publisher occupancyGrid_pub_;
  ros::Subscriber odom_sub_;

 public:
  Mapper();
  ~Mapper();

};

#endif
