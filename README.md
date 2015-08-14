# snowmower_mapping
This repository handles the creation of nav_msgs/OccupancyGrid messages that mark areas with snow (or grass) and areas that have been plowed (or mowed).

## To Do
* **DONE** Use waitForTransform between frames instead of subscribing to odom.

* **DONE** Make sure the last waypoints are inititialized when the mapper begins mapping.

* **DONE** Fix the problem where the circle seems to shift to the right and the right side gets moved over to the left side.

* **DONE** Sometimes the circle doesn't show up at the end of the rectangle. Are we making the circle at the beginning or the end of the path?

* **DONE** Use the ROS parameter server to intialize variables in the class constructor.

* Allow map to be initialized with two of the following: numCols/numRows, ppm, width/height (in meters). And, if all three are used, display a warning and use width/height and whichever resolution is greater.

* **DONE** Allow the option to import an image file which indicates known a priori grass. Then also publish an occupancy grid that contains the known grass minus the mowed grass.

* *DONE* Add map server.

* *DONE* Add a service to reset the map and percentage complete, raise and lower the pen, and get the map (mowed, static grass, and remaining grass maps).

* Should the map be initialized from the grass image (number of columns/rows)? Then the resolution (ppm or mpp) or height/width in meters are the only necessary parameters. However, given a really big image, this process might take a long time.

* **DONE** Figure out why "paint drip" effect is happening.

* **DONE** Allow user to specify image name from command line or launch file.