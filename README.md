# people_tracker_2d

This repository contains the implementation of people tracking in 2D using 2D LiDARs. To be able to run this tracker you need these topics:

- A 2D occupancy grid map (0 point in occupancy map should be the same as in your map frame)
- tf tree containing map, base and laser frame

You can change the corresponding parameters in the launch file or by remapping the arguments.

Output topics are:

- people_tracker_2d/static_scan
- people_tracker_2d/dynamic_scan
- people_tracker_2d/moving_objects (nav_msgs::GridCells)
- people_tracker_2d/static_objects (nav_msgs::GridCells)
- people_tracker_2d/unknown_objects (nav_msgs::GridCells)
- people_tracker_2d/dyn_obstacles (could be used in teb_local_planner)
- people_tracker_2d/velocity_ellipses (visualization_msgs::MarkerArray)
