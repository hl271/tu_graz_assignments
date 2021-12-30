#ifndef DLA3_MAV_TRAJECTORY_GENERATOR_ROS_H
#define DLA3_MAV_TRAJECTORY_GENERATOR_ROS_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

class TrajectoryGenerator {
 public:
  TrajectoryGenerator(ros::NodeHandle& nh, ros::NodeHandle& pn);

  void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

  void setMaxSpeed(double max_v);

  void planTrajectory(const mav_trajectory_generation::Vertex::Vector& vertices, mav_trajectory_generation::Trajectory* trajectory);
  // Plans a trajectory to take off from the current position and
  // fly to the given altitude (while maintaining x,y, and yaw).
  bool planTrajectory(const Eigen::VectorXd& goal_pos,
                      const Eigen::VectorXd& goal_vel,
                      mav_trajectory_generation::Trajectory* trajectory);
                      
  bool planTrajectory(const Eigen::VectorXd& goal_pos,
                      const Eigen::VectorXd& goal_vel,
                      const Eigen::VectorXd& start_pos,
                      const Eigen::VectorXd& start_vel,
                      double v_max, double a_max,
                      mav_trajectory_generation::Trajectory* trajectory);
                      
  bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

 private:
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;
  ros::Subscriber sub_odom_;
  ros::Subscriber simplified_traj_sub;
  void trajectoryCallback(const mav_planning_msgs::PolynomialTrajectory4D::ConstPtr &p_msg);


  ros::NodeHandle& nh_;
  ros::NodeHandle& pnode_;
  Eigen::Affine3d current_pose_;
  Eigen::Vector3d current_velocity_;
  Eigen::Vector3d current_angular_velocity_;
  double max_v_; // m/s
  double max_a_; // m/s^2
  double max_j_; // m/s^3
  double max_s_; // m/s^4
  double max_ang_v_;
  double max_ang_a_;

};

#endif // MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H
