/*
 * demo_pkg.h
 *
 *  Created on: May 24, 2016
 *      Author: eric
 */

#pragma once
#define TEST_MODE

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <string.h>
#include <stdio.h>
#include <stdexcept>


// DEBUG headers
#ifdef TEST_MODE
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#endif



class SafeVel
{
public:
	// Copy Constructor
	SafeVel(ros::NodeHandle nh, ros::NodeHandle nh_private);

	// Destructor
	~SafeVel();

	// Define a struct
	struct polar
	{
		int iter;
		double dist;
	};

private:

	// DEBUG variables and functions
	#ifdef TEST_MODE

	ros::Publisher marker_array; // array of dumb markers

	void add_markers(visualization_msgs::MarkerArray &markers, const std::vector<double> &array, double length, std::string prefix, std::string frame_id, int starting_marker_id=0);

	#endif


	inline double sign(double x)
	{
	  return x < 0.0 ? -1.0 : 1.0;
	}

	//ROS NodeHandle Objects
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

	// ROS Parameters
	bool initialized_;
	bool publish_profiles_;
	bool use_robot_positions_; // want to include robots into the obstacle velocity

  int robot_index_;
	int number_of_robots_;

	int max_range_;

	double robot_radius_;
	double active_radius_;
	double stop_margin_;
	double passive_radius_;

  std::vector<std::string> robot_names_;

	// Safety Ranges
	double safe_dist_;
	double slow_dist_;
	double stop_dist_;
	double flee_dist_;

	double robot_buffer_;

	// equivalent velocity data
	polar velocity_;


	// Line Equations
	double m_slow_; // slope of "slow" range
	double b_slow_; // linear offset of line

	double m_flee_;
	double b_flee_;

	// Maximum allowed velocity
	double pos_max_vel_;
	double neg_max_vel_;

	// Velocity override linear params
	double m_passive_;
	double m_active_;
	double b_passive_;
	double b_active_;

	// Number of rays used to estimate velocity bounds
	int num_meas_rays_;
	int num_half_rays_;

	//QUadrant approach (4 meas rays)
	int q1_;
	int q2_;
	int q3_;
	int q4_;

	// Max angular increment between angular ray and scan return
	double theta_max_;
	int max_num_rays_;
	int half_meas_incr_;

	// The number of scan rays for a given measurement ray (correcting for field of view)
	int meas_iter_range_;
	int num_pseudo_rays_;

	// scan begin/end indices
	int scan_begin_;
	int scan_end_;


	// ROS Pub/Sub objects
	ros::Subscriber twist_subscriber_;
	ros::Subscriber scan_subscriber_;
	ros::Publisher twist_publisher_;

	// ROS Timer
	ros::Timer transform_timer_;

	// ROS TF Listener
	tf::TransformListener tf_listener_;

	// Visualization Publishers
	ros::Publisher bounds_publisher_;
	ros::Publisher radius_publisher_;
	ros::Publisher active_publisher_;
	ros::Publisher passive_publisher_;

	std::string sub_topic_;
	std::string pub_topic_;
	std::string base_frame_;

	// Scan message reference
	sensor_msgs::LaserScan::ConstPtr scan_;

	// Initialize and validate ROS Parameters
	void initParams();
	void checkParams();

	// Subscriber Callback Functions
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void timerCallback(const ros::TimerEvent &);

	// Member Functions
	void storeTables();
	void calcProfiles();
	void lookupVehicles();
	void estimateBounds(const int i, double distance, bool high_priority = true);
	void updateBound(const double &current, double & previous);
	geometry_msgs::Polygon regularPolygon(const double &radius, int n_vert);
	void calculateRanges(const sensor_msgs::LaserScan::ConstPtr&, const int&);
	void correctRequest();
	bool calcPush(const sensor_msgs::LaserScan::ConstPtr&);
	void iterPush(const sensor_msgs::LaserScan::ConstPtr&, const int&);

	// Twist message topic
	geometry_msgs::Twist original_vel_;
	geometry_msgs::Twist filtered_vel_;

	geometry_msgs::PolygonStamped radius_poly_;
	geometry_msgs::PolygonStamped active_poly_;
	geometry_msgs::PolygonStamped passive_poly_;
	geometry_msgs::PolygonStamped vel_bounds_;

	// Vector to store quadrant indices
	std::vector<int> q_;

	// Vector to store cosines and sines of scans
	std::vector<double> cos_;
	std::vector<double> sin_;
	std::vector<double> abs_sec_;
	std::vector<double> abs_sec_n_;
	std::vector<double> rel_cos_;
	std::vector<double> abs_sec_np_; //parabola calculation
	std::vector<double> abs_csc_;
	std::vector<double> theta_;
	std::vector<double> meas_theta_;

	std::vector<double> return_iter_;
	std::vector<double> meas_rays_;
	std::vector<double> neg_meas_rays_;

	std::vector<polar> vehicle_locations_;
};
