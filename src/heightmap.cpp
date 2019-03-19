/*  Copyright (C) 2010 UT-Austin &  Austin Robot Technology,
 *  David Claridge, Michael Quinlan
 *  Copyright (C) 2012 Jack O'Quin
 * 
 *  License: Modified BSD Software License 
 */

/** @file

    @brief ROS class for detecting obstacles in a point cloud.

   This class produces a point cloud containing all points that lie on
   an obstacle and are taller than the @c height_threshold parameter.

Subscribes:

- @b velodyne_points [sensor_msgs::PointCloud2] data from one
  revolution of the Velodyne LIDAR

Publishes:

- @b veloydne_obstacles [sensor_msgs::PointCloud2] grid cells that
  contain an obstacle

- @b veloydne_clear [sensor_msgs::PointCloud2] grid cells with no
  obstacles


@author David Claridge, Michael Quinlan 

*/

#include <velodyne_height_map/heightmap.h>

namespace velodyne_height_map {

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))

HeightMap::HeightMap(ros::NodeHandle node, ros::NodeHandle priv_nh)
{
  // get parameters using private node handle
  priv_nh.param("cell_size", m_per_cell_, 0.25);
  priv_nh.param("full_clouds", full_clouds_, false);
  priv_nh.param("grid_dimensions", grid_dim_, 320);
  priv_nh.param("height_threshold", height_diff_threshold_, 0.12);
  priv_nh.param("negative_threshold", negative_diff_threshold_, -0.5);
  
  ROS_INFO_STREAM("height map parameters: "
                  << grid_dim_ << "x" << grid_dim_ << ", "
                  << m_per_cell_ << "m cells, "
                  << height_diff_threshold_ << "m threshold, "
                  << negative_diff_threshold_ << "m negative_threshold, "
                  << (full_clouds_? "": "not ") << "publishing full clouds");

  // Set up publishers  
  obstacle_publisher_ = node.advertise<VPointCloud>("velodyne_obstacles",1);
  clear_publisher_ = node.advertise<VPointCloud>("velodyne_clear",1);  
  grid_publisher_ = node.advertise<nav_msgs::OccupancyGrid>("map",1);  

  // subscribe to Velodyne data points
  velodyne_scan_ = node.subscribe("velodyne_points", 10,
                                  &HeightMap::processData, this,
                                  ros::TransportHints().tcpNoDelay(true));

  obstacle_grid_.info.resolution = m_per_cell_;
  obstacle_grid_.info.width = grid_dim_;
  obstacle_grid_.info.height = grid_dim_;
  geometry_msgs::Pose map_origin;
  map_origin.position.x = -(int)((float)(obstacle_grid_.info.width/2)*m_per_cell_);
  map_origin.position.y = -(int)((float)(obstacle_grid_.info.height/2)*m_per_cell_);
  map_origin.position.z = 0;
  map_origin.orientation.x = 0;
  map_origin.orientation.y = 0;
  map_origin.orientation.z = 0;
  map_origin.orientation.w = 1;
  obstacle_grid_.info.origin = map_origin;
  obstacle_grid_.data.resize(obstacle_grid_.info.width * obstacle_grid_.info.height);
}

HeightMap::~HeightMap() {}

void HeightMap::constructFullClouds(const VPointCloud::ConstPtr &scan,
                                    unsigned npoints, size_t &obs_count,
                                    size_t &empty_count)
{
  float min[grid_dim_][grid_dim_];
  float max[grid_dim_][grid_dim_];
  bool init[grid_dim_][grid_dim_];
  memset(&init, 0, grid_dim_*grid_dim_);
  
  // build height map
  for (unsigned i = 0; i < npoints; ++i) {
    int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) {
      if (!init[x][y]) {
        min[x][y] = scan->points[i].z;
        max[x][y] = scan->points[i].z;
        init[x][y] = true;
      } else {
        min[x][y] = MIN(min[x][y], scan->points[i].z);
        max[x][y] = MAX(max[x][y], scan->points[i].z);
      }
    }
  }

  // display points where map has height-difference > threshold
  for (unsigned i = 0; i < npoints; ++i) {
    int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) {
      if ((max[x][y] - min[x][y] > height_diff_threshold_) || max[x][y] < negative_diff_threshold_ ) {   
        obstacle_cloud_.points[obs_count].x = scan->points[i].x;
        obstacle_cloud_.points[obs_count].y = scan->points[i].y;
        obstacle_cloud_.points[obs_count].z = scan->points[i].z;
        obstacle_grid_.data[y*obstacle_grid_.info.width + x] = 100;
        //obstacle_cloud_.channels[0].values[obs_count] = (float) scan->points[i].intensity;
        obs_count++;
      } else {
        clear_cloud_.points[empty_count].x = scan->points[i].x;
        clear_cloud_.points[empty_count].y = scan->points[i].y;
        clear_cloud_.points[empty_count].z = scan->points[i].z;
        obstacle_grid_.data[y*obstacle_grid_.info.width + x] = 0;
        //clear_cloud_.channels[0].values[empty_count] = (float) scan->points[i].intensity;
        empty_count++;
      }
    }
  }
}

void HeightMap::constructGridClouds(const VPointCloud::ConstPtr &scan,
                                    unsigned npoints, size_t &obs_count,
                                    size_t &empty_count)
{
  float min[grid_dim_][grid_dim_];
  float max[grid_dim_][grid_dim_];
  float num_obs[grid_dim_][grid_dim_];
  float num_clear[grid_dim_][grid_dim_];
  bool init[grid_dim_][grid_dim_];

  //memset(&init, 0, grid_dim_*grid_dim_);
  
  for (int x = 0; x < grid_dim_; x++) {
    for (int y = 0; y < grid_dim_; y++) {
      init[x][y]=false;
      num_obs[x][y]=0;
      num_clear[x][y]=0;
    }
  }

  // build height map
  for (unsigned i = 0; i < npoints; ++i) {
    int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_) {
      if (!init[x][y]) {
        min[x][y] = scan->points[i].z;
        max[x][y] = scan->points[i].z;
        num_obs[x][y] = 0;
        num_clear[x][y] = 0;
        init[x][y] = true;
      } else {
        min[x][y] = MIN(min[x][y], scan->points[i].z);
        max[x][y] = MAX(max[x][y], scan->points[i].z);
      }
    }
  }

  // calculate number of obstacles in each cell
  for (unsigned i = 0; i < npoints; ++i) {
    int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
    if (x >= 0 && x < grid_dim_ && y >= 0 && y < grid_dim_ && init[x][y]) {
      if ((max[x][y] - min[x][y] > height_diff_threshold_) || max[x][y] < negative_diff_threshold_ ) {   
        num_obs[x][y]++;
      } else {
        num_clear[x][y]++;
      }
    }
  }
  for (int i = 0; i < obstacle_grid_.info.width * obstacle_grid_.info.height; i++) {
    obstacle_grid_.data[i] = -1;
  }
  // create clouds from grid
  double grid_offset=grid_dim_/2.0*m_per_cell_;
  for (int x = 0; x < grid_dim_; x++) {
    for (int y = 0; y < grid_dim_; y++) {
      if (num_obs[x][y]>0) {

        obstacle_cloud_.points[obs_count].x = -grid_offset + (x*m_per_cell_+m_per_cell_/2.0);
        obstacle_cloud_.points[obs_count].y = -grid_offset + (y*m_per_cell_+m_per_cell_/2.0);
        obstacle_cloud_.points[obs_count].z = height_diff_threshold_;
        //obstacle_cloud_.channels[0].values[obs_count] = (float) 255.0;
        obstacle_grid_.data[y*obstacle_grid_.info.width + x] = 100;
        obs_count++;
      }
      if (num_clear[x][y]>0) {
        clear_cloud_.points[empty_count].x = -grid_offset + (x*m_per_cell_+m_per_cell_/2.0);
        clear_cloud_.points[empty_count].y = -grid_offset + (y*m_per_cell_+m_per_cell_/2.0);
        clear_cloud_.points[empty_count].z = height_diff_threshold_;
        //clear_cloud_.channels[0].values[empty_count] = (float) 255.0;
        obstacle_grid_.data[y*obstacle_grid_.info.width + x] = 0;
        empty_count++;
      }
    }
  }
}

/** point cloud input callback */
void HeightMap::processData(const VPointCloud::ConstPtr &scan)
{
  if ((obstacle_publisher_.getNumSubscribers() == 0)
      && (clear_publisher_.getNumSubscribers() == 0))
    return;
  
  // pass along original time stamp and frame ID
  obstacle_cloud_.header.stamp = scan->header.stamp;
  obstacle_cloud_.header.frame_id = scan->header.frame_id;

  // pass along original time stamp and frame ID
  clear_cloud_.header.stamp = scan->header.stamp;
  clear_cloud_.header.frame_id = scan->header.frame_id;

  // pass along original time stamp and frame ID
  obstacle_grid_.header.stamp = ros::Time::now();
  obstacle_grid_.header.frame_id = scan->header.frame_id;
  obstacle_grid_.info.map_load_time = ros::Time::now();

  // set the exact point cloud size -- the vectors should already have
  // enough space
  size_t npoints = scan->points.size();
  obstacle_cloud_.points.resize(npoints);
  //obstacle_cloud_.channels[0].values.resize(npoints);

  clear_cloud_.points.resize(npoints);
  //clear_cloud_.channels[0].values.resize(npoints);

  size_t obs_count=0;
  size_t empty_count=0;
  // either return full point cloud or a discretized version
  if (full_clouds_)
    constructFullClouds(scan,npoints,obs_count, empty_count);
  else
    constructGridClouds(scan,npoints,obs_count, empty_count);
  
  obstacle_cloud_.points.resize(obs_count);
  //obstacle_cloud_.channels[0].values.resize(obs_count);

  clear_cloud_.points.resize(empty_count);
  //clear_cloud_.channels[0].values.resize(empty_count);
  
  //if (obstacle_publisher_.getNumSubscribers() > 0)
    obstacle_publisher_.publish(obstacle_cloud_);

  //if (grid_publisher_.getNumSubscribers() > 0)
    grid_publisher_.publish(obstacle_grid_);

  //if (clear_publisher_.getNumSubscribers() > 0)
    clear_publisher_.publish(clear_cloud_);
}

} // namespace velodyne_height_map
