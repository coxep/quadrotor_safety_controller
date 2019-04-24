/*
 * safe_vel.cpp
 *
 *  Created on: June 28, 2016
 *      Author: Eric Cox
 *      Email: ericpaulcox@gmail.com
 *
 *      Changelog:
 *      V00: Very simple, basic script with 4 measurement axes. Suboptimal behavior. Realistic but brittle assumptions, including:
 *      	- requested velocity in direction of x-axis ONLY
 *      	-
 */

#include "safe_vel/safe_vel.h"
#include <geometry_msgs/Twist.h>
#include <sstream>

// Safety Velocity Constructor

SafeVel::SafeVel(ros::NodeHandle nh, ros::NodeHandle nh_private) :
nh_(nh), nh_private_(nh_private) {
	ROS_INFO("Starting Velocity Safety Controller...");

	// Initialize ROS Parameters
	initParams();
	checkParams();

	// Advertise Topic to be published
	twist_publisher_ = nh_.advertise<geometry_msgs::Twist>(pub_topic_, 5);

	// Publish polygons representing the velocity bounds and vehicle footprints
	if (publish_profiles_) {
		bounds_publisher_ = nh_.advertise<geometry_msgs::PolygonStamped>(
				"vel_bounds", 1);
		radius_publisher_ = nh_.advertise<geometry_msgs::PolygonStamped>(
				"robot_radius", 1, 1);
		active_publisher_ = nh_.advertise<geometry_msgs::PolygonStamped>(
				"active_radius", 1, 1);
		passive_publisher_ = nh_.advertise<geometry_msgs::PolygonStamped>(
				"passive_radius", 1, 1);
	}

	marker_array = nh_.advertise<visualization_msgs::MarkerArray>(
			"dumb_markers", 2);

	// define ROS subscribers
	scan_subscriber_ = nh.subscribe("scan", 1, &SafeVel::scanCallback, this);
	twist_subscriber_ = nh.subscribe(sub_topic_, 1, &SafeVel::twistCallback, this);

	// define ROS timers
	if ((number_of_robots_ > 1) && (use_robot_positions_)) {
		transform_timer_ = nh_.createTimer(ros::Duration(1), &SafeVel::timerCallback, this);
	}
}

SafeVel::~SafeVel() {
	ROS_INFO("Shutting Down Safety Velocity Controller...");
}

void SafeVel::initParams() {

	// Set value of initialization to false
	initialized_ = false;
  std::string this_robot_name;

	if (!nh_private_.getParam("num_meas_rays", num_meas_rays_)) {
		num_meas_rays_ = 11;
	}

	if (!nh_private_.getParam("max_vel", pos_max_vel_)) {
		pos_max_vel_ = 1.0;
	}

	if (!nh_private_.getParam("max_push_vel", neg_max_vel_)) {
		neg_max_vel_ = 1.0;
	}

	if (!nh_private_.getParam("robot_radius", robot_radius_)) {
		robot_radius_ = 0.2;
	}
	if (!nh_private_.getParam("active_radius", active_radius_)) {
		active_radius_ = 0.3;
	}
	if (!nh_private_.getParam("passive_radius", passive_radius_)) {
		passive_radius_ = 1.4;
	}

	if (!nh_private_.getParam("publish_profiles", publish_profiles_)) {
		publish_profiles_ = false;
	}

	if (!nh_private_.getParam("use_robot_positions", use_robot_positions_)) {
		use_robot_positions_ = true;
	}

 	if (!nh_private_.getParam("robot_name", this_robot_name)) {
		this_robot_name = "quadrotor1";
	} 

	if (!nh_private_.getParam("number_of_robots", number_of_robots_)) {
		number_of_robots_ = 1;
	}

	if (!nh_private_.getParam("robot_buffer", robot_buffer_)) {
		robot_buffer_ = 1.0;
	}

	if (!nh_private_.getParam("pub_topic", pub_topic_)) {
		pub_topic_ = "cmd_vel/safety_vel";
	}

	if (!nh_private_.getParam("sub_topic", sub_topic_)) {
		sub_topic_ = "cmd_vel/navigation";
	}

  XmlRpc::XmlRpcValue robot_list;
  nh_private_.param("robots", robot_list, robot_list);

  if (robot_list.size() != number_of_robots_)
  {
    ROS_WARN("number_of_robots %i is not equal to the size of robot list %i.", number_of_robots_, robot_list.size());
  }

  for (int i = 0; i < robot_list.size(); i++)
  {
    robot_names_.push_back(robot_list[i]);
  }

  for (int i = 0; i < number_of_robots_; i++)
  {
    if (this_robot_name.compare(robot_names_[i]) == 0)
    {
      robot_index_ = i;
      break;
    }
  }

	// Set initial velocity output to zero
	original_vel_.linear.x = 0;
	original_vel_.linear.y = 0;
	velocity_.dist = 0;
	velocity_.iter = 0;

	// Store an empty vector for measurement rays
	meas_rays_.resize(num_meas_rays_);
	neg_meas_rays_.resize(num_meas_rays_);

	// Solve for linear velocity equations
	m_passive_ = pos_max_vel_ / (passive_radius_ - active_radius_);
	b_passive_ = -m_passive_ * active_radius_;

	m_active_ = -neg_max_vel_ / (active_radius_ - robot_radius_);
	b_active_ = -m_active_ * active_radius_;
}

void SafeVel::checkParams() {
	try {
		if (robot_radius_ <= 0) {
			throw std::invalid_argument(
					"robot_radius must be greater than zero");
		}

		if (active_radius_ <= robot_radius_) {
			throw std::invalid_argument(
					"active_radius must be greater than robot_radius");
		}
		if (passive_radius_ <= active_radius_) {
			throw std::invalid_argument(
					"passive_radius must be greater than active_radius");
		}
	}

	catch (const std::invalid_argument& e) {
		ROS_ERROR_STREAM("Critical Error: " << e.what()<<"!");
		ROS_WARN("Shutting down...");
		ros::shutdown();
	}
}

void SafeVel::lookupVehicles() {

	tf::StampedTransform transform;
	tf::Vector3 origin;

	std::ostringstream my_frame;
	my_frame << robot_names_[robot_index_] << "/base_footprint";

	std::vector<polar> vehicle_locations;

	for (int i = 0; i < number_of_robots_; ++i) // This assumes that max_id of all robots equals the number of robots (robots numbered 1 - num_robots).
	{
		if (i != robot_index_) {
			std::ostringstream other_frame; // Create a text stamp
			other_frame << robot_names_[i] << "/base_footprint";
			try {

				//listener.waitForTransform(other_frame.str(), my_frame.str(), ros::Time(0), ros::Duration(0.2));
				tf_listener_.lookupTransform(my_frame.str(), other_frame.str(), ros::Time(0), transform);
				// Calculate distance between vehicles
				origin = transform.getOrigin();
				double distance = pow((pow(origin[0], 2) + pow(origin[1], 2)),0.5)- active_radius_ - robot_buffer_; // Calculate distance between vehicles, factoring in active radius of vehicle
				if (distance < robot_radius_)
					distance = robot_radius_;
				if ((distance < passive_radius_)) {
					// find half_iteration associated with other vehicle
					int half_iter = (int) (num_half_rays_ *((atan2(origin[1], origin[0]))+M_PI)/(2 * M_PI) + num_half_rays_)%num_half_rays_;
					polar vehicle;
					vehicle.iter = half_iter;
					vehicle.dist = distance;

					// Store vehicle distance and half_iteration index into a vector
					vehicle_locations.push_back(vehicle);
					std::cout << "Vehicle distance: " << distance<<"/n";
					std::cout << "vehicle iter: " << half_iter << "/n/n";
				}

			} catch (tf::TransformException &ex) {
				ROS_ERROR("%s",ex.what());
				// Do nothing;
			}
		}
	}

	// Copy vehicle locations into member variable
	vehicle_locations_ = vehicle_locations;

}

void SafeVel::timerCallback(const ros::TimerEvent &t) {
	lookupVehicles();
}

void SafeVel::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

	//scan_ = msg; // copy pointer to scan message for future reference (maybe use for twist callback reference?)

	if (!initialized_) {

		// Initialize measurement rays to maximum
		meas_rays_.assign(num_meas_rays_, pos_max_vel_);
		neg_meas_rays_.assign(num_meas_rays_, -neg_max_vel_);

		//Determine the number of pseudo rays (this is the number of rays that would be in the scan if the lidar covered 360 deg)
		num_pseudo_rays_ = round(
				2 * M_PI / (msg->angle_max - msg->angle_min)
				* msg->ranges.size());

		// Determine the number of scan rays for a given measurement ray increment
		if (num_pseudo_rays_ % 2 != msg->ranges.size() % 2) // number of scan rays and measurement rays MUST share the same parity
		{
			num_pseudo_rays_ += 1; // Force it have same parity by adding one (hacky but necessary, and negligible for large number of rays)
		}

		num_half_rays_ = 2 * num_pseudo_rays_;

		/* Solve for number of measurement rays (worst case) that would be affected by an obstacle
		 *
		 * Process:
		 *
		 * 1. Solve for the angular spread in a worst-case scenario
		 *    angle = 2*acos(robot_radius/passive_radius)
		 *
		 * 2. Find the fraction of this angle to the total circle:
		 *    fraction = angle/(2*M_PI)  --> acos(robot_radius/passive_radius)/M_PI
		 *
		 * 3. Multiply fraction by total number of measurement ray intervals:
		 *    num_intervals = num_meas_rays*fraction  --> num_meas_rays_*acos(robot_radius/passive_radius)/M_PI
		 *
		 * 4. convert intervals to measurement rays by adding one:
		 *    max_range_ = num_intervals + 1; // this correctly assumes no shared endpoints
		 *       --> num_meas_rays_*acos(robot_radius/passive_radius)/M_PI + 1
		 */

		//max_range_ = num_half_rays_ * acos(robot_radius_ / passive_radius)/ (2 * M_PI); // convert to half angular increments
		max_range_ = num_half_rays_/4 +1;  // a half iter of 0 and negative vel results in two rays not being addressed... added one to fix (TODO)

		half_meas_incr_ = num_half_rays_ / num_meas_rays_; // TODO: make sure that this ratio already results in an exact integer

		for (int i = 0; i < num_meas_rays_; ++i) {
			// Number of measurement rays SHOULD be a multiple of the number of pseudo rays for this to work well (although it still might not break)

			/* Process:
			 *
			 * Each measurement ray should have an interval between its neighbors given by half_meas_incr_
			 *
			 * There should always be one measurement ray that aligns with the direction of motion for an odd number of rays
			 *   -In this case, it is the x-axis
			 *
			 *   measurement ray 0 should begin at x-axis, which is the half_ray_iteration number = (num_half_rays/2 - 1)
			 *   measurement ray 1 should begin at 1 interval above axis, or at (num_half_rays/2 - 1) + half_meas_incr_
			 *
			 *   - At some point, the iteration number will exceed num_half_rays. To fix this, use the mod operator:
			 *     j[i] = (num_half_rays/2 - 1 + i*half_meas_incr_)%num_half_rays_;
			 */

			q_.push_back((num_pseudo_rays_ / num_meas_rays_ + 1	+ i * num_half_rays_ / num_meas_rays_)%(num_half_rays_)); // currently in integer form (faster math); don't rewrite equation

		}

		// so angles need to be cached exactly once
		storeTables();

		if (publish_profiles_) {
			// calculate geometry of profiles
			// Robot radius
			radius_poly_.header.frame_id = msg->header.frame_id;
			radius_poly_.polygon = regularPolygon(robot_radius_, 32);

			// Active radius
			active_poly_.header.frame_id = msg->header.frame_id;
			active_poly_.polygon = regularPolygon(active_radius_, 48);

			// Passive radius header information
			passive_poly_.header.frame_id = msg->header.frame_id;
			passive_poly_.polygon = regularPolygon(passive_radius_, 64);

			// Velocity Bounds
			vel_bounds_.header.frame_id = msg->header.frame_id;

			geometry_msgs::Polygon polygon;
			geometry_msgs::Point32 point;

			for (int i = 0; i < num_meas_rays_; ++i) {
				double angle = 2 * M_PI * (i + 1) / (num_meas_rays_);

				point.x = passive_radius_ * (cos(angle));
				point.y = passive_radius_ * (sin(angle));

				vel_bounds_.polygon.points.push_back(point);
			}

		}

		// Set initial velocity request to zero;
		original_vel_.linear.x = 0.0;
		original_vel_.linear.y = 0.0;

		// Set to true so these calculations are not performed again
		// ans so that velocity callback will be implemented
		initialized_ = true;
	}

	// Determine the max velocity profile (using measurement rays to store values)

	// Reset measurement rays vector to max_velocity
	meas_rays_.assign(num_meas_rays_, pos_max_vel_);
	neg_meas_rays_.assign(num_meas_rays_, -neg_max_vel_);

	int offset = num_pseudo_rays_ - msg->ranges.size();

	// convert scan index to half_iteration
	for (int i = 0; i < msg->ranges.size(); ++i)
	{
			estimateBounds(offset + 2 * i, msg->ranges[i]); //- active_radius_);
	}

	// Cycle through list of vehicles and update velocity bounds
	for (int i = 0; i < vehicle_locations_.size(); ++i)
	{
		estimateBounds(vehicle_locations_[i].iter, vehicle_locations_[i].dist);
	}

	//Update the velocity request
	correctRequest();

	// Publish velocity topic
	twist_publisher_.publish(filtered_vel_);

	if (publish_profiles_) {
		// Publish robot_radius
		radius_poly_.header.stamp = msg->header.stamp;
		radius_publisher_.publish(radius_poly_);
		// Publish robot_radius
		active_poly_.header.stamp = msg->header.stamp;
		active_publisher_.publish(active_poly_);
		// Publish robot_radius
		passive_poly_.header.stamp = msg->header.stamp;
		passive_publisher_.publish(passive_poly_);

		// Must update the velocity bounds topic each callback

		geometry_msgs::Polygon polygon;
		geometry_msgs::Point32 point;

		// TODO: Table trigonometric terms
		for (int i = 0; i < num_meas_rays_; ++i) {

			double angle = -M_PI + 2 * M_PI * (i) / (num_meas_rays_)+ M_PI / (num_meas_rays_);

			point.x = meas_rays_[i] * (cos(angle));
			point.y = meas_rays_[i] * (sin(angle));

			vel_bounds_.polygon.points[i] = point;
		}

		vel_bounds_.header.stamp = msg->header.stamp;
		bounds_publisher_.publish(vel_bounds_);

		// Publish arrows representing bounds
		visualization_msgs::MarkerArray markers;
		std::string prefix = "marker";

		double offset = 0.1;
		add_markers(markers, meas_rays_, offset, prefix, msg->header.frame_id);
		//add_markers(markers, neg_meas_rays_, offset, prefix, msg->header.frame_id, 1000);
	}

}

void SafeVel::add_markers(visualization_msgs::MarkerArray &markers, const std::vector<double> &array, double text_offset,
		std::string prefix, std::string frame_id, int starting_marker_id) {
	visualization_msgs::Marker marker;

	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();
	geometry_msgs::Point arrows;

	// Assume evenly spaced increments
	double angular_increment = 2 * M_PI / array.size();
	double theta_start = 0.5 * angular_increment - M_PI;
	double theta;

	for (int i = 0; i < array.size(); ++i) {

		// Solve for the angular offset from the 0-position (-x axis)
		theta = i * angular_increment + theta_start;

		marker.type = visualization_msgs::Marker::ARROW;
		marker.id = starting_marker_id + i; 	// Unique marker ID

		// Determine x and y locations of text
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.w = cos(theta / 2); //no change to orientation (same as above)
		marker.pose.orientation.z = sin(theta / 2);
		marker.scale.x = fabs(array[i]);
		marker.action = visualization_msgs::Marker::ADD;

		marker.color.a = 1.0; // Don't forget to set the alpha!
		if (array[i] > 0) {
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			marker.scale.y = .005;
			marker.scale.z = .01;
		} else {
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;
			marker.scale.y = .01;
			marker.scale.z = .005;
		}
		marker.frame_locked = 1;

		// Push back the marker
		markers.markers.push_back(marker);
	}

	marker_array.publish(markers);
}

void SafeVel::storeTables() {

	double half_ray_incr = 2 * M_PI / num_half_rays_;

	// Store trig calcs to reduce computation at expense of memory
	for (int i = 0; i < num_half_rays_; ++i) {

		abs_sec_.push_back(fabs(1 / cos(half_ray_incr * (0.5 + i) / 2)));
		abs_csc_.push_back(fabs(1 / sin(half_ray_incr * (0.5 + i) / 2)));

		cos_.push_back(cos(half_ray_incr * i -M_PI));
		sin_.push_back(sin(half_ray_incr * i -M_PI));

		// These values approach infinity. Rather than perform calcs using infinity,
		// calculate worst-case values (ie, min values that do not limit vel calcs)
		if (abs_sec_[i] > passive_radius_ / robot_radius_)
			abs_sec_[i] = passive_radius_ / robot_radius_;
		if (abs_csc_[i] > passive_radius_ / robot_radius_)
			abs_csc_[i] = passive_radius_ / robot_radius_;
	}

	for (int i = 0; i <= max_range_; ++i) { // range doesn't include endpoints (from 0 to range), but rather intervals, so represent by adding 1
		abs_sec_n_.push_back(fabs(1 / cos(half_ray_incr * i))); // This is the half range (symmetric) centered at i=0
		rel_cos_.push_back(fabs(cos(half_ray_incr * i)));
		//if (abs_sec_n_[i] > passive_radius_ / robot_radius_)
		//	abs_sec_n_[i] = passive_radius_ / robot_radius_;

		//abs_sec_np_.push_back(2/(1+cos(half_ray_incr*(0.5+i)/2)));
		abs_sec_np_.push_back(2 / (1 + cos(half_ray_incr * (0.5 + i))));
	}
}

geometry_msgs::Polygon SafeVel::regularPolygon(const double &radius,
		int n_vert) {
	geometry_msgs::Polygon polygon;
	geometry_msgs::Point32 vertice;
	for (int i = 0; i < n_vert; ++i) {
		double angle = 2 * M_PI * (i + 1) / (n_vert);

		vertice.x = radius * (cos(angle));
		vertice.y = radius * (sin(angle));

		polygon.points.push_back(vertice);
	}
	return polygon;
}

void SafeVel::twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {

	// make sure that a scan has been received before
	if (initialized_){

		// copy linear and angular z components
		original_vel_.linear.z = msg->linear.z;
		original_vel_.angular.z = msg->angular.z;

		// convert linear x and y components into polar representation
		velocity_.dist = pow(msg->linear.x*msg->linear.x + msg->linear.y*msg->linear.y,0.5);
		velocity_.iter = (int)(num_half_rays_*(atan2(msg->linear.y, msg->linear.x)+M_PI)/(2*M_PI))%num_half_rays_;
	} else {

		// send 0 velocity
		filtered_vel_.linear.x = 0;
		filtered_vel_.linear.y = 0;
		filtered_vel_.linear.z = 0;
		filtered_vel_.angular.z = 0;

	}

	// stop the timer
	// reset the timer

}

void SafeVel::estimateBounds(const int half_iter, double distance, bool high_priority) {
	// check that distance is in margins

	if ((distance >= robot_radius_) && (distance < passive_radius_)){

		double velocity;
		if (distance > active_radius_){
			velocity = distance*m_passive_ + b_passive_;
		} else {
			velocity = -distance*m_active_ - b_active_;
		}

		// convert distance to velocity



		/* END CASE PROCESS:
		 * Assume that an obstacle occurs at half_iter = 10, and the max_range is 40.
		 * Then, the min half_iter will be half_iter - max_range = -30. However, the
		 * range is from 0 to num_half_rays. So, how to convert -30 to correct value?
		 *
		 * 1) We could add min half_iter (-30) to num_half_rays
		 * 		- This would require that we check the sign each time to ensure a negative value
		 *
		 * 2) Or, we could add all values to num_half_rays (ensuring a positive value), then take the modulus
		 * 		- i.e., min_half_iter = (num_half_rays + half_iter - half_range)%num_half_rays
		 *		        max_half_iter = (num_half_rays + half_iter + half_range)%num_half_rays
		 *
		 *		        If half_iter is positive and half_range is less than num_half_rays,
		 *		        then this will alwyas work
		 */

		/*
		 * Convert the min and max half iterations to min and max measurement rays
		 * Once the min and max half iterations are determined, the min and max
		 * measurement rays can also be determined.
		 *
		 * J_MIN CALCULATION:
		 *
		 * For example, consider the min_half_ray:
		 * To convert this to the min meas ray, first realize that int (type) inherently rounds down.
		 * Since the min_meas_ray is bounded by the min_half_ray, add 1 so that it actually rounds down
		 * to the correct value.
		 *
		 * To solve for the min measurement ray, first divide the min_half_iter (calculated above)
		 * by half_meas_incr (number of half rays per measurement interval)
		 * Note that min_half_ray is guaranteed to fall into the correct range
		 * (between 0 and num_hal_rays) due to prior use of the mod operator.
		 *
		 * One is then added to this ratio to account for int rounding down.
		 *
		 * j_min = (min_half_ray/half_meas_incr_ + 1)
		 *
		 * However, there is more endcase crap to consider. For example, suppose that num_half_rays is 1000.
		 * Suppose that min_half_ray is 999, and that half_meas_incr_ is 10 (i.e., there are 100 measurement rays).
		 * In this case, j_min would be equal to 99.9 + 1 = 100.9. Rounded down, this is 100. However, there are 100
		 * measurement rays, and the max max is 99 (min is 0). So, rather than being the 100th ray, the result should
		 * be the 0th. For this to work, the mod operator should be used once more:
		 *
		 * j_min = (min_half_ray/half_meas_incr_ + 1)%num_meas_rays
		 *
		 * This equation (hopefully) gives the correct result of 0.
		 *
		 * J_MAX CALCULATION:
		 *
		 * For j_max, the process is a bit different, as int (correctly in this case) rounds down.
		 *
		 * This time, we use the max_half_iter value. Divide this value again by half_meas_incr_ to give the max_measurement ray.
		 * Since the number of max_half_iter will always be between the correct range and always rounds down, nothing more to do!
		 *
		 */

		int j_min = (((num_half_rays_ + half_iter - max_range_) % num_half_rays_) / half_meas_incr_ + 1) % num_meas_rays_;
		int j_max = ((num_half_rays_ + half_iter + max_range_) % num_half_rays_ / half_meas_incr_) % num_meas_rays_;

//		if (! high_priority){
//
//			// shape push by permissible bounds
//			for (int j = j_min; j != j_max; j = (j + 1) % num_meas_rays_) {
//
//				// TODO: Might be possible to reduce the logic to two statements in each case
//				if ((j <= j_min) && (j <= j_max) && (half_iter >= q_[j_max]) && (half_iter >= q_[j_min])){
//					equivalent_distance = q_[(j+num_meas_rays_/2)%num_meas_rays_]/abs_sec_n_[num_half_rays_ + q_[j] - half_iter];
//
//				} else if ((j >= j_min) && (j >= j_max) && (half_iter <= q_[j_max]) && (half_iter <= q_[j_min])){
//					equivalent_distance = q_[(j+num_meas_rays_/2)%num_meas_rays_]/abs_sec_n_[num_half_rays_ - q_[j] + half_iter];
//
//				} else {
//					equivalent_distance = q_[(j+num_meas_rays_/2)%num_meas_rays_]/abs_sec_n_[abs(q_[j] - half_iter)];
//
//				}
//
//				if ((equivalent_distance < 0 && distance > 0) || (equivalent_distance > 0 && distance < 0)){
//					if (fabs(equivalent_distance) < fabs(distance) ){
//						distance = sign(distance)*equivalent_distance;
//					}
//				}
//			}
//		}

		// Cycle through measurement lines and draw perpendicular lines for each point velocity

		double local_vel;

		for (int j = j_min; j != j_max; j = (j + 1) % num_meas_rays_) {

			// TODO: Might be possible to reduce the logic to two statements in each case
			if ((j <= j_min) && (j <= j_max) && (half_iter >= q_[j_max]) && (half_iter >= q_[j_min])){
				local_vel = velocity * abs_sec_n_[num_half_rays_ + q_[j] - half_iter];

			} else if ((j >= j_min) && (j >= j_max) && (half_iter <= q_[j_max]) && (half_iter <= q_[j_min])){
				local_vel = velocity * abs_sec_n_[num_half_rays_ - q_[j] + half_iter];

			} else {
				local_vel = velocity * abs_sec_n_[abs(q_[j] - half_iter)];
			}

//			if (high_priority || new_distance > 0){
				// replace current measurement ray with the min of itself and new_distance
		if (local_vel < 0){
			if (meas_rays_[j] < 0){
				if (local_vel < meas_rays_[j]){
					meas_rays_[j] = local_vel;
				}
			} else {
				meas_rays_[j] = local_vel;
			}
		} else {
			updateBound(local_vel, meas_rays_[j]);
		}


//			}
//
//			// All low priority values must be negative
//		    // verify that distance (currently negative) has a magnitude less than or equal to magnitude of complementary ray (which must be positive)
//			else if (new_distance < 0 && meas_rays_[(j + num_meas_rays_/2)%num_meas_rays_] > 0){
//				// low priority velocity bound; only push into available areas
//				if (-new_distance <= meas_rays_[(j + num_meas_rays_/2)%num_meas_rays_]){
//					updateBound(new_distance, meas_rays_[j]);
//				}
//				else {
////					// magnitude of new distance is greater than available velocity, such just use what is available
//					updateBound(-meas_rays_[(j + num_meas_rays_/2)%num_meas_rays_], meas_rays_[j]);
//				}
//			}
		}
	}
}

inline void SafeVel::updateBound(const double &a, double &b)
{
	if (a < b)
		b = a;
}

void SafeVel::correctRequest(){

	//

	filtered_vel_=original_vel_;

	//cycle through measurement rays and figure out the best velocity
	polar min_neg;
	min_neg.dist = -100; //something huge and ridiculous (expecting negative values)
	min_neg.iter = -1; //something impossible

	//empty struct for min difference
	polar min_diff;
	min_diff.dist = 0; // something huge and ridiculous
	min_diff.iter = -1; // something impossible

	int j_min = (((num_half_rays_ + velocity_.iter - max_range_) % num_half_rays_) / half_meas_incr_ + 1) % num_meas_rays_;
	int j_max = ((num_half_rays_ + velocity_.iter + max_range_) % num_half_rays_ / half_meas_incr_) % num_meas_rays_;

	double rel_iter;
	double max;
	double min;
	double vel_req;
	double ortho_dist;
	double new_dist;
	int iter;

	// variable to hold filtered velocity information
	polar filtered_vel;
	filtered_vel.dist = 0;
	filtered_vel.iter = 0;

	// Find best execution of velocity request provided that velocity is available;
	if (fabs(velocity_.dist) != 0){

		// cycle through all affected rays (j_min through j_max)
		for (int j = j_min; j != j_max; j = (j + 1) % num_meas_rays_) {

			// Solve for the relative index value at current j-value (for use in lookup tables):
			if ((j <= j_min) && (j <= j_max) && (velocity_.iter >= q_[j_max]) && (velocity_.iter >= q_[j_min])){
				rel_iter = num_half_rays_ + q_[j] - velocity_.iter;
			} else if ((j >= j_min) && (j >= j_max) && (velocity_.iter <= q_[j_max]) && (velocity_.iter <= q_[j_min])){
				rel_iter = num_half_rays_ - q_[j] + velocity_.iter;
			} else {
				rel_iter = abs(q_[j] - velocity_.iter);
			}

			// define min, max, and current request variables for easier comprehension
			max = meas_rays_[j];
			min = -meas_rays_[(j + num_meas_rays_/2)%num_meas_rays_];
			vel_req = velocity_.dist*rel_cos_[rel_iter];

			// Determine how much velocity is available in desired direction
			if ((max > 0) && (max > min) && (vel_req > min)){
				// If there is more velocity request than velocity availability, return whatever is available
				if (vel_req >= max){

					// calculate the equivalent velocity in original direction
					new_dist = max;//*rel_cos_[rel_iter];

				} else {

					// velocity available is greater than request (good!), so return
					// request and calculate equivalent velocity in original direction
					new_dist = vel_req; //*rel_cos_[rel_iter];
				}

			} else {

				// there is no velocity available, or there is already a request greater than the current request
				new_dist = 0;
			}

			// check to see if current
			if (new_dist*rel_cos_[rel_iter] > ortho_dist){

				// update the maximum ortho velocity variable
				ortho_dist = new_dist*rel_cos_[rel_iter];

				// Update the filtered velocity structure with distance and direction
				filtered_vel.dist = new_dist;
				filtered_vel.iter = q_[j];
			}
		}
	}

	//std::cout << "filtered vel magnitude" << filtered_vel.dist <<std::endl;

	// Velocity request and velocity available
	if (filtered_vel.dist != 0){

	    filtered_vel_.linear.x = filtered_vel.dist*cos_[filtered_vel.iter];
	    filtered_vel_.linear.y = filtered_vel.dist*sin_[filtered_vel.iter];

	} else {
		// No request/overruled

		for (int i=0; i<num_meas_rays_; ++i){
			// check for three things:

			// 1. the minimum magnitude of negative measurement rays which are greater than magnitudes of complementary positive measurement rays
			//       and the index of the positive complementary ray
			// 2. (if 1 not available) the minimum difference between the magnitudes above
			// If neither 1 nor 2 are available, set velocity request to zero.


			// CASE 1:

			// TODO: SHOULD BE %(NUM_MEAS_RAYS - 1)  --or should it? check...
			if ((meas_rays_[i] < 0) && (meas_rays_[(i+num_meas_rays_/2)%num_meas_rays_] > 0)){

				// Check the see if magnitude of "push" ray is less than "available push" ray
				if (fabs(meas_rays_[i]) <= meas_rays_[(i+num_meas_rays_/2)%num_meas_rays_]){

					// Check for min
					if(meas_rays_[i] > min_neg.dist){

						min_neg.dist = meas_rays_[i];
						min_neg.iter = q_[i];
					}

				} else {

				// CASE 2:
					// push is greater than push availability, so keep track of difference
					if( meas_rays_[(i+num_meas_rays_/2)%num_meas_rays_] > min_diff.dist ){

						min_diff.dist = meas_rays_[(i+num_meas_rays_/2)%num_meas_rays_];
						min_diff.iter = i;
					}

//					if( fabs(meas_rays_[(i+num_meas_rays_/2)%num_meas_rays_] + meas_rays_[i]) < min_diff.dist ){
//
//						min_diff.dist = fabs(meas_rays_[(i+num_meas_rays_/2)%num_meas_rays_] + meas_rays_[i]);
//
//
//						std::cout << "meas_ray distance: " << meas_rays_[i] << std::endl;
//						std::cout << "opposite ray: "<< meas_rays_[(i+num_meas_rays_/2)%num_meas_rays_] << std::endl;
//
//						min_diff.iter = i;
//					}

				}

			}
		}

		if (min_neg.iter != -1){
			// create case 1 velocity request
			filtered_vel_.linear.x = min_neg.dist*cos_[min_neg.iter];
			filtered_vel_.linear.y = min_neg.dist*sin_[min_neg.iter];
		} else if (min_diff.iter != -1){

			// case 2 request
			filtered_vel_.linear.x = meas_rays_[(min_diff.iter+num_meas_rays_/2)%num_meas_rays_]*cos_[q_[(min_diff.iter+num_meas_rays_/2)%num_meas_rays_]];
			filtered_vel_.linear.y = meas_rays_[(min_diff.iter+num_meas_rays_/2)%num_meas_rays_]*sin_[q_[(min_diff.iter+num_meas_rays_/2)%num_meas_rays_]];

		} else {
			// set request to zero
			filtered_vel_.linear.x = 0;
			filtered_vel_.linear.y = 0;
		}

	}

}
