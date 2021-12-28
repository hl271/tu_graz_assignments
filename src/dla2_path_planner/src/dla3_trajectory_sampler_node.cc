/*
 * Simple example that shows how to sample a trajectories generated using the mav_trajectory_generation routines.
 *
 * Code mostly based on the code:
 *   mav_trajectory_generation_ros/trajectory_sampler_node.h / cpp
 * 
 * Created on: January 2020
 * Author: (P2F) Jesus Pestana Puerta <jesus.pestana@pro2future.at>
 *
 * [TODO]: 
 * 
 */

#include  "ros/ros.h"
#include <dla3_trajectory_sampler/dla3_trajectory_sampler.h>
#include <iostream>

int main(int argc, char** argv) {
  /* initialize ros */
  ros::init(argc, argv, "trajectory_sampler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("Starting DLA3TrajectorySampler...");
  DLA3TrajectorySampler trajectory_sampler(nh, pnh);
  ROS_INFO("DLA3TrajectorySampler started...");
  
  ros::spin();

  return 0;
}