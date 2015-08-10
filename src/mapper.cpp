#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <fstream> // To read image
#include <map_server/image_loader.h>
#include <nav_msgs/GetMap.h>
#include <std_msgs/Float64.h>
#include "snowmower_mapping/mapper.h"

/* This function fills a circle in the OccupancyGrid at the desired point. */
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

/* This function fills a rectangle in the OccupancyGrid between the desired points. */
void Mapper::fillRectangle(double x1, double y1, double x2, double y2) {
  // Find the rotation angle of the rectangle and the slope.
  double rot = atan2(y2-y1,x2-x1);
  double slope = (y2-y1)/(x2-x1);

  // Create vectors that contain the x and y coordinates of the four corners of the rectangle
  std::vector<double> corX;
  corX.push_back(x1-r_*sin(rot));
  corX.push_back(x2-r_*sin(rot));
  corX.push_back(x2+r_*sin(rot));
  corX.push_back(x1+r_*sin(rot));

  std::vector<double> corY;
  corY.push_back(y1+r_*cos(rot));
  corY.push_back(y2+r_*cos(rot));
  corY.push_back(y2-r_*cos(rot));
  corY.push_back(y1-r_*cos(rot));

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
  // Do the first one
  double xMin = corX.at(0);
  double xMax = corX.at(0);
  // And then the other three
  for (int i = 1; i < corX.size(); i++) {
    xMin = std::min(xMin, corX.at(i));
    xMax = std::max(xMax, corX.at(i));
  }

  // Then determine which row and column those points are in
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
      for (int q = 1; q < top.size(); q++) {
	yMin = std::max(yMin, m.at(bottom.at(q))*x+b.at(bottom.at(q)));
	yMax = std::min(yMax, m.at(top.at(q))*x+b.at(top.at(q)));
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


void Mapper::odomCB(const nav_msgs::Odometry& msg) {

  // The point (x1,y1) is set to the last known pose
  double x1 = lastPose_.position.x;
  double y1 = lastPose_.position.y;

  // The point (x2,y2) is set to the new pose
  double x2 = msg.pose.pose.position.x;
  double y2 = msg.pose.pose.position.y;

  if (firstDraw_ && penDown_) {
    fillCircle(x2,y2);
    firstDraw_ = false;

    // Set the last pose to the new pose for the next iteration
    lastPose_.position.x = x2;
    lastPose_.position.y = y2;

    // Update the map's time stamp
    ros::Time updateTime = ros::Time::now();
    mowed_map_.header.stamp = updateTime;

    // Publish the newly populated map
    occupancyGrid_pub_.publish(mowed_map_);
  }

  // If the robot changed its position and the pen is down
  else if ((x1 != x2 || y1 != y2) && penDown_ == true && !firstDraw_) {

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

    // Calculated and publish percent mowed
    calculatePercentMowed();

  }
}

void Mapper::spin() {

  // The point (x1,y1) is set to the last known pose
  double x1 = lastPose_.position.x;
  double y1 = lastPose_.position.y;

  try{
    ros::Time now = ros::Time::now();
    listener_.waitForTransform(map_frame_, base_frame_, now, ros::Duration(3.0));
    listener_.lookupTransform(map_frame_, base_frame_, now, transform_);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  // The point (x2,y2) is set to the new pose
  double x2 = transform_.getOrigin().x();
  double y2 = transform_.getOrigin().y();

  if (firstDraw_ && penDown_) {
    fillCircle(x2,y2);
    firstDraw_ = false;

    // Set the last pose to the new pose for the next iteration
    lastPose_.position.x = x2;
    lastPose_.position.y = y2;

    // Update the map's time stamp
    ros::Time updateTime = ros::Time::now();
    mowed_map_.header.stamp = updateTime;

    // Publish the newly populated map
    occupancyGrid_pub_.publish(mowed_map_);
  }

  // If the robot changed its position and the pen is down
  else if ((x1 != x2 || y1 != y2) && penDown_ == true && !firstDraw_) {

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

    // Calculated and publish percent mowed
    calculatePercentMowed();

  }
}

/* Function to calculate percentage of mowed grass and publish it*/
void Mapper::calculatePercentMowed() {
  // This is a terribly inefficient way to do this. But go through all the cells, see if there was grass, if so, see if it was mowed. Then divide number of cells with mowed grass by number of cells that ever had grass to get the percent mowed.
  int mowed_cells = 0;
  int total_cells = 0;
  for (int i = 0; i < numCols_*numRows_; i++) {
    if (grass_map_.data[i] == 100) {
      total_cells += 1;
      if (mowed_map_.data[i] == grass_map_.data[i]) {
	mowed_cells += 1;
      }
    }
  }
  percent_mowed_ = (double)mowed_cells/(double)total_cells;
  // Create a message to publish
  std_msgs::Float64 percent;
  // And stuff percent_mowed_ inside of it
  percent.data = percent_mowed_;
  // Then publish it!
  percent_pub_.publish(percent);
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
  mowed_map_.header.frame_id = map_frame_;

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

  firstDraw_ = true; // The next instance will be the first time drawing on the map
}

/* Used to reset the map. */
void Mapper::resetMap() {
  // Make an array of all zeros that's numCOls_*numRows_ 
  std::vector<signed char> a(numCols_*numRows_,0);
  mowed_map_.data = a;
  // Note: Data in OccupancyGrid is stored in row-major order. Thus, consecutive elements of the rows of the grid are contigious in the vector.

  firstDraw_ = true; // The next instance will be the first time drawing on the map
}

void Mapper::importGrassMap() {
  nav_msgs::GetMap::Response resp;

  const std::string& fname = "/home/snowmower/Google Drive/Lawnmower/Code/Matlab/arenas/png/circle_inside.png";

  std::ifstream fin(fname.c_str());
  if (fin.fail()) {
    ROS_ERROR("Could not open %s.", fname.c_str());
    return;
  }

  if (!fin.fail()) {
    ROS_INFO_STREAM("Found the map.");
  }

  double res = 1.0/ppm_;
  double origin[3];
  origin[0] = origin[1] = origin[2] = 0.0;
  /*
    void loadMapFromFile(nav_msgs::GetMap::Response* resp,
                         const char* fname, double res, bool negate,
                         double occ_th, double free_th, double* origin,
                         bool trinary=true);
   */
  map_server::loadMapFromFile(&resp, fname.c_str(), res, false, 0.65, 0.196, origin, true);
  // Make an array of all zeros that's numCOls_*numRows_ in length
  // Here, we will store the scaled version of the OccupancyGrid in resp.map.
  std::vector<signed char> scaled_map(numCols_*numRows_,0);
  int numColsIm  = resp.map.info.width;
  int numRowsIm = resp.map.info.height;

  double widthScale = (double)numColsIm/(double)numCols_;
  double heightScale = (double)numRowsIm/(double)numRows_;
  
  for (int i = 0; i < numCols_; i++) {
    for (int j = 0; j < numRows_; j++) {

      // Fill in the OccupancyGrid Here
      scaled_map[j*numCols_+i] = resp.map.data[round(j*widthScale)*numColsIm+round(i*heightScale)];
    }
  }
  grass_map_.info = mowed_map_.info;
  grass_map_.data = scaled_map;

  grass_map_pub_.publish( grass_map_ );
}

/* Constructor */
Mapper::Mapper(): private_nh_("~") {
  // Set up the publisher and subsciber objects
  occupancyGrid_pub_ = public_nh_.advertise<nav_msgs::OccupancyGrid>("mowed_map",1);
  grass_map_pub_ = public_nh_.advertise<nav_msgs::OccupancyGrid>("grass_map", 1, true);
  // Publish the percent of the grass_map_ that has been mowed each update
  percent_pub_ = public_nh_.advertise<std_msgs::Float64>("percent_complete",1, true);
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

  mapper.importGrassMap();

  while (ros::ok()) {
    mapper.spin();
  }

  return 0;

}
