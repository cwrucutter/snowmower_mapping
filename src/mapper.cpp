#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include "snowmower_mapping/mapper.h"


void Mapper::odomCB(const nav_msgs::Odometry& msg){

  // The point (x1,y1) is set to the last known pose
  double x1 = lastPose_.position.x;
  double y1 = lastPose_.position.y;

  // The point (x2,y2) is set to the new pose
  double x2 = msg.pose.pose.position.x;
  double y2 = msg.pose.pose.position.y;

  // If the robot changed its position
  if (x1 != x2 || y1 != y2) {

    // Set the last pose to the new pose for the next iteration
    lastPose_.position.x = x2;
    lastPose_.position.y = y2;

    // Now begin filling in the OccupancyGrid.
    // Start with the circle around the first point.
    // Find the minimum and maximum x values
    double xMin1 = x2-r_;
    double xMax1 = x2+r_;

    // Find the maximum and minimum columns of the OccupancyGrid corresponding to the max and min x values. The "round" function is used. So a column is selected if the circle extends more than half way into it. Note: The first column is column number 0.
    int xMinMap1 = std::max(int(round(xMin1*ppm_)),0);
    int xMaxMap1 = std::min(int(round(xMax1*ppm_))-1,numCols_-1);

    for (int i = xMinMap1; i <= xMaxMap1; i++) {
      // Choose x that is in the middle of a column
      double x = (i + 0.5)/ppm_;
      // FInd the corresponding ymin and ymax values for the current x
      double yMin1 = y2 - sqrt(fabs(pow(r_,2)-pow(x-x2,2)));
      double yMax1 = y2 + sqrt(fabs(pow(r_,2)-pow(x-x2,2)));
      // And what row in the OccupancyGrid they are
      int yMinMap1 = std::max(int(round(yMin1*ppm_)),0);
      int yMaxMap1 = std::min(int(round(yMax1*ppm_))-1,numRows_-1);
      for (int j = yMinMap1; j <= yMaxMap1; j++) {
	// Fill in the OccupancyGrid Here
	mowed_map_.data[j*numCols_+i] = 100;
      }
    }

    double rot = atan2(y2-y1,x2-x1);
    double slope = (y2-y1)/(x2-x1);

    // Create an array containing the x coordinates of the four corners
    double corXarray[4] = {x1-r_*sin(rot),
			   x2-r_*sin(rot),
			   x2+r_*sin(rot),
			   x1+r_*sin(rot)};

    std::vector<double> corX (corXarray, corXarray + sizeof(corXarray) / sizeof(int));

    // Create an array containing the y coordinates of the four corners
    double corYarray[4] = {y1+r_*cos(rot),
			   y2+r_*cos(rot),
			   y2-r_*cos(rot),
			   y1-r_*cos(rot)};

    std::vector<double> corY (corYarray, corYarray + sizeof(corYarray) / sizeof(int));

    // The four lines that connect the four corners and bound the box can be described by four lines, which can be described by four slopes and four y-intercepts.
    std::vector<double> m (4);
    std::vector<double> b (4);

    for (int i = 0; i<=3; i++) {
      // First calculate the slope (y1-y2)/(x1-x2)
      m.at(i) = (corY.at(i)-corY.at((i+1)%4))/(corX.at(i)-corX.at((i+1)%4));
      // Then calculate the y-int (b = y-m*x)
      b.at(i) = corY.at(i) - m.at(i)*corX.at(i);
    }

    // Figure out which lines are on top and which are on the bottom using the rotation angle fo the line between the points. The variables 'top' and 'bottom' will be used to store the indicies of the 'm' and 'b' to be used when determining the max and min y values to use at each x value.
    std::vector<int> top;
    std::vector<int> bottom;

    if (rot==0) {
      top.push_back(0);
      bottom.push_back(2);
    }
    else if (rot > 0 && rot < M_PI/2) {
      top.push_back(0);
      top.push_back(1);
      bottom.push_back(2);
      bottom.push_back(3);
    }
    else if (rot == M_PI/2) {
      top.push_back(1);
      bottom.push_back(3);
    }
    else if (rot > M_PI/2 && rot < M_PI) {
      top.push_back(1);
      top.push_back(2);
      bottom.push_back(3);
      bottom.push_back(0);
    }
    else if (rot == M_PI || rot == -M_PI) {
      top.push_back(2);
      bottom.push_back(0);
    }
    else if (rot > -M_PI && rot < -M_PI/2) {
      top.push_back(2);
      top.push_back(3);
      bottom.push_back(0);
      bottom.push_back(1);
    }
    else if (rot == -M_PI/2) {
      top.push_back(3);
      bottom.push_back(1);
    }
    else if (rot > -M_PI/2 && rot < 0) {
      top.push_back(3);
      top.push_back(0);
      bottom.push_back(1);
      bottom.push_back(2);
    }

    // Find the min and max x value of the corners of the rectangle.
    double xMin3 = *std::min_element(corX.begin(), corX.end());
    double xMax3 = *std::max_element(corX.begin(), corX.end());

    int xMinMap3 = std::max(int(round(xMin3*ppm_)),0);
    int xMaxMap3 = std::min(int(round(xMax3*ppm_)),numCols_-1);

    for (int i = xMinMap3; i <= xMaxMap3; i++) {
      double x = (i + 0.5)/ppm_;
      // Find the corresponding ymin and ymax values for the current x
      // Do this by finding the min values of the two top and the max values of the two bottoms
      // Do the first one
      double yMin3 = m.at(bottom.at(0))*x+b.at(bottom.at(0));
      double yMax3 = m.at(top.at(0))*x+b.at(top.at(0));
      if (top.size()>1) {
	// And if there are more than one top and bottom line, check them as well
	for (int i = 1; i < top.size(); i++) {
	  yMin3 = std::max(yMin3, m.at(bottom.at(i))*x+b.at(bottom.at(i)));
	  yMax3 = std::min(yMax3, m.at(top.at(i))*x+b.at(top.at(i)));
	}
      }
      // Now find what rows those y values are in and iterate through them
      int yMinMap3 = std::max(int(round(yMin3*ppm_)),0);
      int yMaxMap3 = std::min(int(round(yMax3*ppm_))-1,numRows_-1);
      for (int j = yMinMap3; j <= yMaxMap3; j++) {
	// Fill in the OccupancyGrid Here
	mowed_map_.data[j*numCols_+i] = 100;
      }
    }

    // Publish the newly populated map
    occupancyGrid_pub_.publish(mowed_map_);
  }
}

/* Constructor */
Mapper::Mapper() {

  // TODO: Get all these parameters from the ROS parameter server.
  // Make options for defining number of cells, ppm, and meters.
  // Any two of those should deterimine the other three.
  // Cells = ppm * meters

  ppm_ = 1;
  numCols_ = 10;
  numRows_ = 10;
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
