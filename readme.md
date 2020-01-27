# Path planner

ROS node path_publisher is used to set up a path for global planner.
The node returns consecutive points of a desired polygon in the topic */clicked_point*.
The node takes information about actual robot position from the topic */agribot/odometry*.
Paths are established after publishing a non-positive value to the topic */path_publisher/choice*.
The shape of path depends on published value. The value determines the number of edges of polygon. Publishing -2, -1 or 0 on this topic sets up a straight path. 
Positive value determines the length of the side of the next polygon in dm. 

**E.g. publishing "90" means that the next polygon will have a side of length 9.0m. Publishing "-8" will set up an octagon.**

The first side of polygon (or straight segment) has the same orientation as the robot.