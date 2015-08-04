# snowmower_mapping
This repository handles the creation of nav_msgs/OccupancyGrid messages that mark areas with snow (or grass) and areas that have been plowed (or mowed).

## To Do
* Use waitForTransform between frames instead of subscribing to odom.

* **DONE** Make sure the last waypoints are inititialized when the mapper begins mapping.

* **DONE** Fix the problem where the circle seems to shift to the right and the right side gets moved over to the left side.

* **DONE** Sometimes the circle doesn't show up at the end of the rectangle. Are we making the circle at the beginning or the end of the path?

* **DONE** Use the ROS parameter server to intialize variables in the class constructor.

* Allow the option to import an image file which indicates known a priori grass. Then also publish an occupancy grid that contains the known grass minus the mowed grass.