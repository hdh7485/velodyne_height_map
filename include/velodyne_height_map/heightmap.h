/* -*- mode: C++ -*- */
/*  Copyright (C) 2010 UT-Austin & Austin Robot Technology,
 *  David Claridge, Michael Quinlan
 * 
 *  License: Modified BSD Software License 
 */


#ifndef _HEIGHT_MAP_H_
#define _HEIGHT_MAP_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>

#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>

#include <cstdlib>
#include <string>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_ros/point_cloud.h>

#include <lcm_to_ros/hyundai_mission.h>

using namespace std;

namespace velodyne_height_map {

// shorter names for point cloud types in this namespace
typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

class HeightMap
{
public:

  /** Constructor
   *
   *  @param node NodeHandle of this instance
   *  @param private_nh private NodeHandle of this instance
   */
  HeightMap(ros::NodeHandle node, ros::NodeHandle private_nh);

  ~HeightMap();

  void CSVReader(std::vector<string> vec_string_);
  /** callback to process data input
   *
   *  @param scan vector of input 3D data points
   *  @param stamp time stamp of data
   *  @param frame_id data frame of reference
   */
  void processData(const VPointCloud::ConstPtr &scan);
  void missionCallback(const lcm_to_ros::hyundai_mission::ConstPtr& mission_msg);

private:
  void constructFullClouds(const VPointCloud::ConstPtr &scan, unsigned npoints,
                           size_t &obs_count, size_t &empty_count);
  void constructGridClouds(const VPointCloud::ConstPtr &scan, unsigned npoints,
                           size_t &obs_count, size_t &empty_count, size_t &obs_contour_count);


  // Parameters that define the grids and the height threshold
  // Can be set via the parameter server
  int grid_dim_;
  double m_per_cell_;
  double height_diff_threshold_;
  double negative_diff_threshold_;
  bool full_clouds_;
  double back_looking_dist_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  // Point clouds generated in processData
  VPointCloud obstacle_cloud_;            
  VPointCloud clear_cloud_;
  VPointCloud obstacle_cloud_contour;
  int obs_contour_cnt;
  nav_msgs::OccupancyGrid obstacle_grid_;

  // ROS topics
  ros::Subscriber velodyne_scan_;
  ros::Subscriber mission_subscriber_;
  ros::Publisher obstacle_publisher_;
  ros::Publisher obstacle_contour_publisher_;
  ros::Publisher clear_publisher_;
  ros::Publisher grid_publisher_;

  VPointCloud csv_cloud_; 
  VPointCloud csv_transformed_cloud_; 

  std::string csv_first_lane_center;
  std::string csv_left_lane;
  std::string csv_right_lane;
  
  std::vector<std::vector<double> > csv_points;
  std::vector<string> filenames;

};

} // namespace velodyne_height_map

#endif
