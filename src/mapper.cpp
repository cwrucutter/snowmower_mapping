#include <ros/ros.h>
#include <cmath>
#include <vector>
#include "snowmower_mapping/mapper.h"


void Mapper::odomCB(const nav_msgs::Odometry& msg){

  // The point (x1,y1) is set to the last known pose
  double x1 = lastPose_.position.x;
  double y1 = lastPose_.position.y;

  // The point (x2,y2) is set to the new pose
  double x2 = msg.pose.pose.position.x;
  double y2 = msg.pose.pose.position.y;

  // Set the last pose to the new pose for the next iteration
  lastPose_.position.x = x2;
  lastPose_.position.y = y2;

  // Now begin filling in the OccupancyGrid.
  // Start with the circle around the first point.
  // Find the minimum and maximum x values
  double xMin1 = x1-r_;
  double xMax1 = x1+r_;

  // Find the maximum and minimum columns of the OccupancyGrid corresponding to the max and min x values. The "round" function is used. So a column is selected if the circle extends more than half way into it. Note: The first column is column number 0.
  int xMinMap1 = std::max(int(round(xMin1*ppm_)),0);
  int xMaxMap1 = std::min(int(round(xMax1*ppm_))-1,numCols_-1);

  for (int i = xMinMap1; i <= xMaxMap1; i++) {
    // Choose x that is in the middle of a column
    double x = (i - 0.5)/ppm_;
    // FInd the corresponding ymin and ymax values for the current x
    double ymin1 = y1 - sqrt(fabs(pow(r_,2)-pow(x-x1,2)));
    double ymax1 = y1 + sqrt(fabs(pow(r_,2)-pow(x-x1,2)));
    // And what row in the OccupancyGrid they are
    int yMinMap1 = std::max(int(round(ymin1*ppm_)),0);
    int yMaxMap1 = std::min(int(round(ymax1*ppm_))-1,numRows_-1);
    for (int j = yMinMap1; j <= yMaxMap1; j++) {
      // todo: Fill in the OccupancyGrid Here
      mowed_map_.data[j*numCols_+i] = 100;
    }
  }

  // Publish the newly populated map
  occupancyGrid_pub_.publish(mowed_map_);
}

/* Constructor */
Mapper::Mapper() {

  // TODO: Get all these parameters from the ROS parameter server.
  // Make options for defining number of cells, ppm, and meters.
  // Any two of those should deterimine the other three.
  // Cells = ppm * meters

  ppm_ = 10;
  numCols_ = 100;
  numRows_ = 100;
  r_ = 0.5;

  occupancyGrid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("mowed_map",1);
  odom_sub_ = nh_.subscribe("odom",10,&Mapper::odomCB,this);

  // Set the frame of the OccupancyGrid. It should be in the global /map frame
  mowed_map_.header.frame_id = "/odom";

  // Set the resolution, load time, width and height of the OccupancyGrid
  mowed_map_.info.resolution = 1.0/ppm_; //
  mowed_map_.info.map_load_time = ros::Time::now();
  mowed_map_.info.width = numCols_;
  mowed_map_.info.height = numRows_;

  // Set the pose of the (0,0)
  geometry_msgs::Pose mapOrigin;
  mowed_map_.info.origin = mapOrigin;

  // Make an array of all zeros that's numCOls_*numRows_ 
  std::vector<signed char> a(numCols_*numRows_);
  mowed_map_.data = a;
  // Note: Data in OccupancyGrid is stored in row-major order. Thus, consecutive elements of the rows of the grid are contigious in the vector.

  mowed_map_.data[0] = 100;
  mowed_map_.data[numCols_*numRows_-1] = 100;
};

/* Destructor */
Mapper::~Mapper() {

};

int main(int argc, char **argv) {

  //Initialize ROS
  ros::init(argc, argv, "mapper");

  // Create a Mapper object
  Mapper mapper;

  // And spin!
  ros::spin();

  return 0;

}
