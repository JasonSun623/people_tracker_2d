/**
 *  people_tracker_2d.h
 *
 *  Created on: 07.12.2018
 *      Author: Dario Mammolo
 */

#ifndef PEOPLE_TRACKER_2D_H
#define PEOPLE_TRACKER_2D_H

#include <queue>

#include <ros/ros.h>
#include <object_detector.h>
#include <tracked_object.h>
#include <kalman_filter.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <costmap_converter/ObstacleMsg.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

class StaticScanExtractor{
public:
  /**
   *  Class Constructor
   */
  StaticScanExtractor(ros::NodeHandle nh, ros::NodeHandle nh_);

  /**
   *  Class Destructor
   */
  ~StaticScanExtractor();

  /**
   *  The main function which recives the raw laser scan and publishes then the
   *  extracted static laser scan for the SLAM framework.
   */
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

  /**
   *  A callback for the occupancy grid map from the SLAM framework.
   */
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);

  /**
   *  A helper function which transforms a cluster of laser scans to a cluster
   *  in the laser frame.
   */
   vector<geometry_msgs::Point> getClusterInLaserFrame(map<int, double>& cluster);


private:
  // Node handler
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Services
  ros::ServiceClient get_current_pose_client_;

  // Publisher, Subscriber
  ros::Subscriber map_sub_;
  ros::Subscriber scan_sub_;

  ros::Publisher static_scan_pub_;
  ros::Publisher dynamic_scan_pub_;
  ros::Publisher moving_objects_pub_;
  ros::Publisher static_objects_pub_;
  ros::Publisher unknown_objects_pub_;
  ros::Publisher debug_scan_pub_;
  ros::Publisher dyn_obstacles_pub_;
  ros::Publisher vel_ellipse_pub_;

  // ROS msgs
  sensor_msgs::LaserScan init_scan_;
  sensor_msgs::LaserScan static_scan_;
  sensor_msgs::LaserScan dynamic_scan_;
  sensor_msgs::LaserScan node_scan_;
  nav_msgs::Odometry odom_msg_;
  nav_msgs::OccupancyGrid map_msg_;

  // scan
  unsigned int scan_ranges_size_;
  double scan_angle_increment_;
  double scan_range_min_;
  double scan_range_max_;
  double scan_angle_min_;

  // TF
  tf::TransformListener base_to_laser_listener_;
  tf::Transform base_to_laser_;
  tf::Transform laser_to_base_;
  tf::Transform current_pose_tf_;
  tf::Transform map_to_latest_tf_;
  tf::Transform map_to_latest_laser_tf_;
  tf::Transform laser_diff_tf_;

  // Object detector
  double abd_lambda_;
  double abd_sigma_;
  ObjectDetector object_detector_;
  int upsampling_factor_;

  // Parameters
  std::string robot_name_;
  int map_width_;
  int map_height_;
  double map_resolution_;

  // variables
  bool map_callback_initialised_;
  bool initialised_first_scan_;
  std::vector<std::vector<double>> init_scan_sum_;
  ros::Time begin_time_;
  ros::Time prev_node_stamp_;
  ros::Time curr_node_stamp_;
  double prev_delta_t_;
  double std_dev_process_;
  double std_dev_range_;
  double std_dev_theta_;
  std::vector<geometry_msgs::Point> occluded_means_;
  std::vector<geometry_msgs::Point> free_means_;
  std::vector<std::vector<geometry_msgs::Point>> free_cluster_vectors_;
  std::vector<std::vector<geometry_msgs::Point>> occluded_cluster_vectors_;
  std::vector<TrackedObject> tracked_objects_;
  KalmanFilter kalman_filter_;
  std::string map_callback_topic_;
  std::string scan_callback_topic_;
  std::string map_frame_;
  std::string base_frame_;
  std::string laser_frame_;
  int init_scan_unknown_pt_;
  int init_min_time_;
  int wall_threshold_;
  double association_radius_;
  double association_radius_sq_;
  int unknown_since_threshold_;
  double static_size_threshold_;
  double static_size_threshold_sq_;
  double dynamic_vel_threshold_;
  double dynamic_vel_threshold_sq_;
  int not_seen_threshold_;
  geometry_msgs::PoseStamped current_pose_;
  ros::Time current_pose_stamp_;
  bool current_pose_intialised_;
  std::queue<sensor_msgs::LaserScan> static_scan_queue_;
  int static_cell_range_;
  int min_cluster_size_for_moving_;
  int min_velocity_count_;
};

#endif  // PEOPLE_TRACKER_2D_H
