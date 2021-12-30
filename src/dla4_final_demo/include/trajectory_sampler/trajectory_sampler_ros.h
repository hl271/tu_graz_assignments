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

#ifndef MAV_TRAJECTORY_GENERATION_DLA3_TRAJECTORY_REPLAYER_H
#define MAV_TRAJECTORY_GENERATION_DLA3_TRAJECTORY_REPLAYER_H

#include <iostream>
#include <ros/ros.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
#include <mav_trajectory_generation/polynomial.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/conversions.h>
#include <tf/transform_broadcaster.h>

class DLA3TrajectorySampler {
 public:
  DLA3TrajectorySampler(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
  ros::NodeHandle& nh_, pnh_;
  ros::Subscriber smooth_trajectory4d_sub;
  void smoothTrajectory4DCallback(const mav_planning_msgs::PolynomialTrajectory4D::ConstPtr& p_msg);

  // The trajectory to sub-sample.
  mav_trajectory_generation::Trajectory trajectory_;

  void processTrajectory();
  ros::Timer publish_timer_;
  void commandTimerCallback(const ros::TimerEvent&);
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Publisher command_pub_;
  ros::Time start_time_;
  // Trajectory sampling interval.
  double dt_;
  // Time at currently published trajectory sample.
  double current_sample_time_;
};

#endif // MAV_TRAJECTORY_GENERATION_DLA3_TRAJECTORY_REPLAYER_H
