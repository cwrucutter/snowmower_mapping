#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include "snowmower_mapping/mapper.h"

void Mapper::fillCircle(double x1, double y1) {
  // Now begin filling in the OccupancyGrid.
  // Start with the circle around the first point.
  // Find the minimum and maximum x values
  double xMin = x1-r_;
  double xMax = x1+r_;

  // Find the maximum and minimum columns of the OccupancyGrid corresponding to the max and min x values. The "round" function is used. So a column is selected if the circle extends more than half way into it. Note: The first column is column number 0.
  int xMinMap = std::max(int(round(xMin*ppm_)),0);
  int xMaxMap = std::min(int(round(xMax*ppm_))-1,numCols_-1);

  for (int i = xMinMap; i <= xMaxMap; i++) {
    // Choose x that is in the middle of a column
    double x = (i + 0.5)/ppm_;
    // FInd the corresponding ymin and ymax values for the current x
    double yMin = y1 - sqrt(fabs(pow(r_,2)-pow(x-x1,2)));
    double yMax = y1 + sqrt(fabs(pow(r_,2)-pow(x-x1,2)));
    // And what row in the OccupancyGrid they are
    int yMinMap = std::max(int(round(yMin*ppm_)),0);
    int yMaxMap = std::min(int(round(yMax*ppm_))-1,numRows_-1);
    for (int j = yMinMap; j <= yMaxMap; j++) {
      // Fill in the OccupancyGrid Here
      mowed_map_.data[j*numCols_+i] = 100;
    }
  }
}

void Mapper::fillRectangle(double x1, double y1, double x2, double y2) {
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
  double xMin = *std::min_element(corX.begin(), corX.end());
  double xMax = *std::max_element(corX.begin(), corX.end());

  int xMinMap = std::max(int(round(xMin*ppm_)),0);
  int xMaxMap = std::min(int(round(xMax*ppm_)),numCols_-1);

  for (int i = xMinMap; i <= xMaxMap; i++) {
    double x = (i + 0.5)/ppm_;
    // Find the corresponding ymin and ymax values for the current x
    // Do this by finding the min values of the two top and the max values of the two bottoms
    // Do the first one
    double yMin = m.at(bottom.at(0))*x+b.at(bottom.at(0));
    double yMax = m.at(top.at(0))*x+b.at(top.at(0));
    if (top.size()>1) {
      // And if there are more than one top and bottom line, check them as well
      for (int i = 1; i < top.size(); i++) {
	yMin = std::max(yMin, m.at(bottom.at(i))*x+b.at(bottom.at(i)));
	yMax = std::min(yMax, m.at(top.at(i))*x+b.at(top.at(i)));
      }
    }
    // Now find what rows those y values are in and iterate through them
    int yMinMap = std::max(int(round(yMin*ppm_)),0);
    int yMaxMap = std::min(int(round(yMax*ppm_))-1,numRows_-1);
    for (int j = yMinMap; j <= yMaxMap; j++) {
      // Fill in the OccupancyGrid Here
      mowed_map_.data[j*numCols_+i] = 100;
    }
  }
}


void Mapper::odomCB(const nav_msgs::Odometry& msg){

  // The point (x1,y1) is set to the last known pose
  double x1 = lastPose_.position.x;
  double y1 = lastPose_.position.y;

  // The point (x2,y2) is set to the new pose
  double x2 = msg.pose.pose.position.x;
  double y2 = msg.pose.pose.position.y;

  // If the robot changed its position and the pen is down
  if ((x1 != x2 || y1 != y2) && penDown_ == true) {

    // Set the last pose to the new pose for the next iteration
    lastPose_.position.x = x2;
    lastPose_.position.y = y2;

    fillCircle(x2,y2);
    fillRectangle(x1,y1,x2,y2);
    // Update the map's time stamp
    ros::Time updateTime = ros::Time::now();
    mowed_map_.header.stamp = updateTime;

    // Publish the newly populated map
    occupancyGrid_pub_.publish(mowed_map_);
  }
}

/* Changes penDown_ to true*/
void Mapper::penDown() {
  penDown_ = true;
}

/* Changes penDown_ to false*/
void Mapper::penUp() {
  penDown_ = false;
}

/* Initialize the parameters and an empty map  */
void Mapper::init() {
  // Use the ROS parameter server to initilize parameters
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";

  if(!private_nh_.getParam("ppm", ppm_))
    ppm_ = 1;
  if(!private_nh_.getParam("numCols", numCols_))
    numCols_ = 10;
  if(!private_nh_.getParam("numRows", numRows_))
    numRows_ = 10;
  if(!private_nh_.getParam("radius", r_))
    r_ = 0.5;;

  // Set the frame of the OccupancyGrid. It should be in the global /map frame
  mowed_map_.header.frame_id = "/odom";

  // Set the resolution, load time, width and height of the OccupancyGrid
  mowed_map_.info.resolution = 1.0/ppm_;
  ros::Time begin = ros::Time::now();
  mowed_map_.info.map_load_time = begin;
  mowed_map_.info.width = numCols_;
  mowed_map_.info.height = numRows_;

  // Set the pose of the (0,0)
  geometry_msgs::Pose mapOrigin;
  mowed_map_.info.origin = mapOrigin;

  // Make an array of all zeros that's numCOls_*numRows_ 
  std::vector<signed char> a(numCols_*numRows_,0);
  mowed_map_.data = a;
  // Note: Data in OccupancyGrid is stored in row-major order. Thus, consecutive elements of the rows of the grid are contigious in the vector.

}

/* Constructor */
Mapper::Mapper(): private_nh_("~") {
  // Set up the publisher and subsciber objects
  occupancyGrid_pub_ = public_nh_.advertise<nav_msgs::OccupancyGrid>("mowed_map",1);
  odom_sub_ = public_nh_.subscribe("odom",10,&Mapper::odomCB,this);
  // Wait for time to not equal zero. A zero time means that no message has been received on the /clock topic
  ros::Time timeZero(0.0);
  while (ros::Time::now() == timeZero) { }
  // Sleep for a small time to make sure publishing and subscribing works.
  ros::Duration(0.1).sleep();


  // TODO: Get all these parameters from the ROS parameter server.
  // Make options for defining number of cells, ppm, and meters.
  // Any two of those should deterimine the other three.
  // Cells = ppm * meters

  // Put the pen Down (Get ready to start marking).
  penDown_ = true;

  init();
  // Publish the empty map
  occupancyGrid_pub_.publish(mowed_map_);
};

/* Destructor */
Mapper::~Mapper() {

};

int main(int argc, char **argv) {

  //Initialize ROS
  ros::init(argc, argv, "mapper");

  // Create a Mapper object
  Mapper mapper;
  //  mapper.penUp();
  // And spin!
  ros::spin();

  return 0;

}
