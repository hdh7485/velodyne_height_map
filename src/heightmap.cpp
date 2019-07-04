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

using namespace std;

namespace velodyne_height_map {

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))
#define D2R 0.0174533
#define R2D 57.2958

	double RAY_RES = 1; // in degree
	double NUM_RAY = 180/RAY_RES;

	int cnt = 0;

	HeightMap::HeightMap(ros::NodeHandle node, ros::NodeHandle priv_nh):tfListener_(tfBuffer_)
	{
		// get parameters using private node handle
		priv_nh.param("cell_size", m_per_cell_, 0.15);
		priv_nh.param("full_clouds", full_clouds_, false);
		priv_nh.param("grid_dimensions", grid_dim_, 50);
		priv_nh.param("height_threshold", height_diff_threshold_, 0.50);
		priv_nh.param("negative_threshold", negative_diff_threshold_, -0.5);
		priv_nh.param("back_looking_dist", back_looking_dist_, 3.0); // meter scale

		ROS_INFO_STREAM("height map parameters: "
				<< grid_dim_ << "x" << grid_dim_ << ", "
				<< m_per_cell_ << "m cells, "
				<< height_diff_threshold_ << "m threshold, "
				<< negative_diff_threshold_ << "m negative_threshold, "
				<< back_looking_dist_ << "back_looking_dist, "
				<< (full_clouds_? "": "not ") << "publishing full clouds");

		// Set up publishers  
		obstacle_publisher_ = node.advertise<VPointCloud>("velodyne_obstacles",1);
		//obstacle_contour_publisher_ = node.advertise<VPointCloud>("velodyne_obstacles_contour",1);
		clear_publisher_ = node.advertise<VPointCloud>("velodyne_obstacles_contour",1);  
		grid_publisher_ = node.advertise<nav_msgs::OccupancyGrid>("map",1);  

		// subscribe to Velodyne data points
		velodyne_scan_ = node.subscribe("merged_z_filtered_velodyne", 10,
				&HeightMap::processData, this,
				ros::TransportHints().tcpNoDelay(true));
		mission_subscriber_ = node.subscribe("lcm_to_ros/LCM2ROS_mission", 10,
				&HeightMap::missionCallback, this,
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
		obs_contour_cnt = 0;

		// Read CSV, then save into pointcloud type
		std::string path = ros::package::getPath("velodyne_height_map");
		csv_first_lane_center = path + "/csv_map/geofence_first_lane_test.csv";
		csv_right_lane = path + "/csv_map/geofence_right_total_test.csv";
		csv_left_lane = path + "/csv_map/geofence_left_total_test.csv";

		filenames.push_back(csv_first_lane_center);
		filenames.push_back(csv_left_lane);
		filenames.push_back(csv_right_lane);

		CSVReader(filenames);
	}

	HeightMap::~HeightMap() {}

	void HeightMap::CSVReader(std::vector<string> vec_string_) {
		//std::vector<std::vector<double>> output;
		ifstream csvFile;
		int tmp_cnt = 0;


		for(int i =0; i< vec_string_.size(); i++) {
			csvFile.open(vec_string_[i].c_str());
			if (!csvFile.is_open())
			{
				std::cout << "Path Wrong!!!!" << std::endl;
				exit(EXIT_FAILURE);
			}
			string line;
			std::vector <string> vec;
			while (getline(csvFile,line))
			{
				if (line.empty()) // skip empty lines:
				{
					cout << "empty line!" << endl;
					continue;
				}
				std::stringstream iss(line);
				std::string lineStream;
				std::string::size_type sz;
				std::vector <double> row;
				while (getline(iss, lineStream, '\t'))
				{
					row.push_back(std::stod(lineStream, &sz)); // convert to double
				}

				VPoint tmp;
				tmp.x = row[1];
				tmp.y = row[0];
				tmp.z = 0;

				csv_cloud_.push_back(tmp);
				//csv_cloud_.points[tmp_cnt].x = row[1];
				//csv_cloud_.points[tmp_cnt].y = row[0];
				//csv_cloud_.points[tmp_cnt].z = 0;

				//output.push_back(row);
				tmp_cnt++;
			}
		}
	}

	void HeightMap::constructFullClouds(const VPointCloud::ConstPtr &scan,
			unsigned npoints, size_t &obs_count,
			size_t &empty_count)
	{
		float min[grid_dim_][grid_dim_];
		float max[grid_dim_][grid_dim_];
		bool init[grid_dim_][grid_dim_];
		memset(&init, 0, grid_dim_*grid_dim_);
		ROS_INFO("MEMSET");

		// build height map
		for (unsigned i = 0; i < npoints; ++i) {
			int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
			//int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
			int y = (scan->points[i].y/m_per_cell_); // height map for front only
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

		// display points where map has height-difference > thresholdgrid_dim_
		for (unsigned i = 0; i < npoints; ++i) {
			int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
			//int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
			int y = (scan->points[i].y/m_per_cell_); // height map for front only
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
					clear_cloud_.points[obs_count].y = scan->points[i].y;
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
			size_t &empty_count, size_t &obs_contour_count)
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
			//int x = (scan->points[i].x/m_per_cell_); // height map only for front
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

		for (int x = grid_dim_/2.0 - int(back_looking_dist_/m_per_cell_); x < grid_dim_; x++) {
			for (int y = 0; y < grid_dim_; y++) {
				if (num_obs[x][y]>0 ) {

					obstacle_cloud_.points[obs_count].x = -grid_offset + (x*m_per_cell_+m_per_cell_/2.0);
					//obstacle_cloud_.points[obs_count].x =(x*m_per_cell_+m_per_cell_/2.0); // height map only for front
					obstacle_cloud_.points[obs_count].y = -grid_offset + (y*m_per_cell_+m_per_cell_/2.0);

					obstacle_cloud_.points[obs_count].z = height_diff_threshold_;
					//obstacle_cloud_.channels[0].values[obs_count] = (float) 255.0;
					obstacle_grid_.data[y*obstacle_grid_.info.width + x] = 100;
					obs_count++;
				}
				if (num_clear[x][y]>0) {
					clear_cloud_.points[empty_count].x = -grid_offset + (x*m_per_cell_+m_per_cell_/2.0);
					//obstacle_cloud_.points[obs_count].x =(x*m_per_cell_+m_per_cell_/2.0); // height map only for front
					clear_cloud_.points[obs_count].y = -grid_offset + (y*m_per_cell_+m_per_cell_/2.0);
					clear_cloud_.points[empty_count].z = height_diff_threshold_;
					//clear_cloud_.channels[0].values[empty_count] = (float) 255.0;
					obstacle_grid_.data[y*obstacle_grid_.info.width + x] = 0;
					empty_count++;
				}
			}
		}

		std::vector<double> vec_ray_len;
		std::vector<double> vec_ray_theta;
		std::vector<int> height_map_contour_x;
		std::vector<int> height_map_contour_y;

		int num_ray = 180/RAY_RES;
		// ray init
		for (int i =0; i < num_ray; i++ )
		{
			if(i<=num_ray/2)
				vec_ray_theta.push_back(i*RAY_RES*D2R);
			else
				vec_ray_theta.push_back((num_ray-i)*RAY_RES*D2R);

			if(i*RAY_RES <= 45 || i*RAY_RES >= 135)
			{
				double len = grid_dim_/2.0*m_per_cell_/(cos(RAY_RES*i*D2R)+0.00000001);
				vec_ray_len.push_back(abs(len));
			}
			else
			{
				double len = sqrt(pow(grid_dim_/2.0*m_per_cell_,2) + pow(grid_dim_/2.0*m_per_cell_/tan(RAY_RES*i*D2R),2));
				vec_ray_len.push_back(abs(len));
			}
			//        height_map_contour_x.push_back(10000);
			//        height_map_contour_y.push_back(10000);

			//        std::cout << "len at " << i << ": " << vec_ray_len.at(i) << std::endl;
		}

		//    double cross_half_dist = m_per_cell_*1.4142135623/2;

		double ego_x = grid_dim_/2.0-0.5;
		double ego_y = grid_dim_/2.0-0.5;

		for (int x = (grid_dim_)/2.0; x < grid_dim_-1; x++)
		{
			for(int y = 0; y < grid_dim_; y++)
			{
				if(num_obs[x][y]>0)
				{
					double ray_length = sqrt(pow(x-ego_x,2) + pow(y-ego_y,2))*m_per_cell_;

					if(y > (grid_dim_-1)/2) // left side
					{
						double max_theta_ray_x = x+0.5; double max_theta_ray_y = y-0.5;
						double min_theta_ray_x = x-0.5; double min_theta_ray_y = y+0.5;

						double max_x = abs(max_theta_ray_x - ego_x);
						double max_y = abs(max_theta_ray_y - ego_y);

						double min_x = abs(min_theta_ray_x - ego_x);
						double min_y = abs(min_theta_ray_y - ego_y);

						double max_theta = atan2(max_x,max_y); double min_theta = atan2(min_x,min_y);

						//                    std::cout << "right side max theta " << max_theta << " ::: " << "min theta " << min_theta << std::endl;

						// for each ray
						for(int j=num_ray/2; j<num_ray; j++)
						{
							if(vec_ray_theta.at(j) <= max_theta && vec_ray_theta.at(j) >= min_theta)
							{
								if(vec_ray_len.at(j) > ray_length)
								{
									vec_ray_len.at(j) = ray_length;
									height_map_contour_x.push_back(x);
									height_map_contour_y.push_back(y);
								}
							}
						}
					}

					else if(y < (grid_dim_-1)/2) // right side
					{
						double max_theta_ray_x = x+0.5; double max_theta_ray_y = y+0.5;
						double min_theta_ray_x = x-0.5; double min_theta_ray_y = y-0.5;

						double max_x = abs(max_theta_ray_x - ego_x);
						double max_y = abs(max_theta_ray_y - ego_y);

						double min_x = abs(min_theta_ray_x - ego_x);
						double min_y = abs(min_theta_ray_y - ego_y);

						double max_theta = atan2(max_x,max_y); double min_theta = atan2(min_x,min_y);

						// for each ray
						for(int j=0; j<num_ray/2-1; j++)
						{
							if(vec_ray_theta.at(j) <= max_theta && vec_ray_theta.at(j) >= min_theta)
							{
								if(vec_ray_len.at(j) > ray_length)
								{
									vec_ray_len.at(j) = ray_length;
									height_map_contour_x.push_back(x);
									height_map_contour_y.push_back(y);
								}
							}
						}

					}
				}
			}
		}

		cnt = 0;

		for(int i=0; i < height_map_contour_x.size(); i++)
		{
			double x_tmp = -grid_offset + (height_map_contour_x.at(i)*m_per_cell_+m_per_cell_/2.0);
			double y_tmp = -grid_offset + (height_map_contour_y.at(i)*m_per_cell_+m_per_cell_/2.0);
			if(abs(x_tmp) > 0.5 && abs(y_tmp) > 0.5) {
				VPoint as;
				as.x = x_tmp;
				as.y = y_tmp;
				as.z = 0;
				obstacle_cloud_contour.push_back(as);
				cnt++;
			}
		}

		obs_contour_count = cnt;

		//    std::cout << "obs contour cnt : " <<  obs_contour_cnt << std::endl;
	}

	void HeightMap::missionCallback(const lcm_to_ros::hyundai_mission::ConstPtr& mission_msg){
		int mission_number = mission_msg->mission_number;
		int pos_x = mission_msg->pos_x;
		int pos_y = mission_msg->pos_y;
	}
	/** point cloud input callback */
	void HeightMap::processData(const VPointCloud::ConstPtr &scan)
	{
		// pass along original time stamp and frame ID
		obstacle_cloud_.header.stamp = scan->header.stamp;
		obstacle_cloud_.header.frame_id = scan->header.frame_id;

		// pass along original time stamp and frame ID
		clear_cloud_.header.stamp = scan->header.stamp;
		clear_cloud_.header.frame_id = scan->header.frame_id;

		obstacle_cloud_contour.header.stamp = scan->header.stamp;
		obstacle_cloud_contour.header.frame_id = scan->header.frame_id;

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

		obstacle_cloud_contour.points.resize(0);

		size_t obs_count=0;
		size_t empty_count=0;
		size_t obs_contour_count=0;
		// either return full point cloud or a discretized version
		if (full_clouds_)
		{
			//ROS_INFO("full cloud");
			constructFullClouds(scan,npoints,obs_count, empty_count);
		}
		else
		{
			//ROS_INFO("grid cloud");
			constructGridClouds(scan,npoints,obs_count, empty_count,obs_contour_count);
		}

		obstacle_cloud_.points.resize(obs_count);
		//obstacle_cloud_.channels[0].values.resize(obs_count);

		//clear_cloud_.points.resize(empty_count);
		//clear_cloud_.channels[0].values.resize(empty_count);

		//clear_cloud_.points.resize(obs_contour_cnt);
		//std::cout << "num of obs contour : " << obs_contour_cnt << std::endl;
		//ROS_INFO("%d", obs_contour_cnt);
		ROS_DEBUG("%ld",obs_contour_count);
		//obstacle_cloud_contour.points.resize(obs_contour_count);

		//if (obstacle_publisher_.getNumSubscribers() > 0)

		obstacle_publisher_.publish(obstacle_cloud_);
		//obstacle_publisher_.publish(obstacle_cloud_contour);

		//obstacle_contour_publisher_.publish(obstacle_cloud_contour);

		//if (grid_publisher_.getNumSubscribers() > 0)
		
		// only heightmap
		
		// add CSV
		for(int i = 0; i < csv_cloud_.size(); i++) {
			int idx =  int((csv_cloud_.points[i].x - obstacle_grid_.info.origin.position.y)/obstacle_grid_.info.resolution)
			       	* int(obstacle_grid_.info.width) + int((csv_cloud_.points[i].y - obstacle_grid_.info.origin.position.x)/obstacle_grid_.info.resolution);
			if(idx >= 0) {
				obstacle_grid_.data[idx] = 100;
			}
		}
		grid_publisher_.publish(obstacle_grid_);

		geometry_msgs::TransformStamped transformStamped;
		try{
			transformStamped = tfBuffer_.lookupTransform("base_footprint", "odom",
				ros::Time(0));
		} catch (tf2::TransformException &ex) {
			ROS_WARN("Could NOT transform turtle2 to turtle1: %s", ex.what());
		}

		sensor_msgs::PointCloud2 cloud_in;
		sensor_msgs::PointCloud2 cloud_out;
		pcl::toROSMsg(csv_cloud_, cloud_in);
                tf2::doTransform(cloud_in, cloud_out, transformStamped);

		//if (clear_publisher_.getNumSubscribers() > 0)
		clear_publisher_.publish(obstacle_cloud_contour);
	}

} // namespace velodyne_height_map
