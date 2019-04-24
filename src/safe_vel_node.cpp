/*
 * safe_vel_node.cpp
 *
 *  Created on: May 24, 2016
 *      Author: Eric Cox
 */

#include "safe_vel/safe_vel.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "SafeVel");

  ros::NodeHandle nh;

  ros::MultiThreadedSpinner spinner;

  // Private NodeHandle (for params, etc)
  ros::NodeHandle nh_private("~");

  // Initialize class object
  SafeVel safe_vel(nh, nh_private);
  // Check for topic updates
  //ros::spin();
  spinner.spin();

  return 0;
}
