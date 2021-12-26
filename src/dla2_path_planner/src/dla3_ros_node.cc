/*
 * Simple example that shows a trajectory planner using
 *  mav_trajectory_generation.
 *
 *
 * Launch via
 *   roslaunch mav_trajectory_generation_example example.launch
 *
 * Wait for console to run through all gazebo/rviz messages and then
 * you should see the example below
 *  - After Enter, it receives the current uav position
 *  - After second enter, publishes trajectory information
 *  - After third enter, executes trajectory (sends it to the sampler)
 */

#include  "ros/ros.h"
#include <dla3_trajectory_generator/dla3_trajectory_generator_ros.h>

#include <iostream>

int main(int argc, char** argv) {

  ros::init(argc, argv, "trajectory_generator");

  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  ROS_INFO("STARTING DLA3 TRAJECTORY GENERATOR..");
  TrajectoryGenerator planner(n, pn);
  ROS_INFO("DLA3 TRAJECTORY GENERATOR STARTED!");
  ros::spin();


  return 0;
}