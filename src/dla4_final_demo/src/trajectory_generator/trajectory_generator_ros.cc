#include <trajectory_generator/trajectory_generator_ros.h>

TrajectoryGenerator::TrajectoryGenerator(ros::NodeHandle& nh, ros::NodeHandle& pn) :
    nh_(nh),
    pnode_(pn),
    max_v_(2.0),
    max_a_(2.0),
    max_s_(0.5),
    max_j_(0.5),
    current_velocity_(Eigen::Vector3d::Zero()),
    current_pose_(Eigen::Affine3d::Identity()) {
      
  // Ros Topics
  simplified_traj_sub = nh_.subscribe<mav_planning_msgs::PolynomialTrajectory4D>(
                "/path_planner/planned_trajectory", 10,
                &TrajectoryGenerator::trajectoryCallback, this);
  // Load params
  if (!nh_.getParam(ros::this_node::getName() + "/max_v", max_v_)){
    ROS_WARN("[trajectory_generator] param max_v not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_a", max_a_)){
    ROS_WARN("[trajectory_generator] param max_a not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_j", max_j_)){
    ROS_WARN("[trajectory_generator] param max_a not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_s", max_s_)){
    ROS_WARN("[trajectory_generator] param max_a not found");
  }

  // create publisher for RVIZ markers
  pub_markers_ =
      pnode_.advertise<visualization_msgs::MarkerArray>("smooth_trajectory_markers", 0);

  pub_trajectory_ =
      pnode_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("smooth_trajectory",
                                                              0);

  // subscriber for Odometry
  sub_odom_ =
      nh.subscribe("uav_pose", 1, &TrajectoryGenerator::uavOdomCallback, this);
}

// trajectory Callback for simplified Path
void TrajectoryGenerator::trajectoryCallback(const mav_planning_msgs::PolynomialTrajectory4D::ConstPtr &p_msg) {
  ROS_INFO("Received simplified path. Generating Trajectory...");
  // *** Conversion to mav_trajectory_generation::Vertex::Vector ***
    // Create the vertices for the points and lines
    const mav_planning_msgs::PolynomialTrajectory4D &msg = *p_msg;
    size_t N = msg.segments.size();

    mav_trajectory_generation::Vertex::Vector vertices;
    const int dimension = 3;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    mav_trajectory_generation::Vertex start(dimension), end(dimension);
    const mav_planning_msgs::PolynomialSegment4D &start_p = msg.segments[0];
    const mav_planning_msgs::PolynomialSegment4D &end_p = msg.segments[N-1];
    start.makeStartOrEnd(Eigen::Vector3d(start_p.x[0], start_p.y[0], start_p.z[0]), derivative_to_optimize);
    end.makeStartOrEnd(Eigen::Vector3d(end_p.x[0], end_p.y[0], end_p.z[0]), derivative_to_optimize);
    vertices.push_back(start);
    for (size_t i=1; i<N-1; i++)
    {
        const mav_planning_msgs::PolynomialSegment4D &segment = msg.segments[i];
        mav_trajectory_generation::Vertex vertex(dimension);
        vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(segment.x[0], segment.y[0], segment.z[0]));
        vertices.push_back(vertex);        
    } 
    vertices.push_back(end);
    mav_trajectory_generation::Trajectory trajectory;
    planTrajectory(vertices, &trajectory);
    publishTrajectory(trajectory);

}

// Callback to get current Pose of UAV
void TrajectoryGenerator::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

  // store current position in our planner
  tf::poseMsgToEigen(odom->pose.pose, current_pose_);

  // store current vleocity
  tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
}

// Method to set maximum speed.
void TrajectoryGenerator::setMaxSpeed(const double max_v) {
  max_v_ = max_v;
}

void TrajectoryGenerator::planTrajectory(const mav_trajectory_generation::Vertex::Vector& vertices, mav_trajectory_generation::Trajectory* trajectory) {
  ROS_INFO("Planning trajectory...");
  const int dimension = 3;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

  //Estimate segment times
  std::vector<double> segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);
  //Set up params
  mav_trajectory_generation::NonlinearOptimizationParameters params;
  params.algorithm = nlopt::LN_SBPLX;
  params.f_rel = 0.00005;
  params.time_alloc_method = mav_trajectory_generation::NonlinearOptimizationParameters::kSquaredTime;
  //Set up solver
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, params);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  //add constraints
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::JERK, max_j_);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::SNAP, max_s_);

  //solve
  opt.optimize();
  opt.getTrajectory(&(*trajectory));

}

bool TrajectoryGenerator::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
  ROS_INFO("Publishing trajectory to RViz...");
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::MarkerArray markers;
  double distance =
      0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string frame_id = "world";

  mav_trajectory_generation::drawMavTrajectory(trajectory,
                                               distance,
                                               frame_id,
                                               &markers);
  pub_markers_.publish(markers);

  // send trajectory to be executed on UAV
  mav_planning_msgs::PolynomialTrajectory4D msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
                                                                 &msg);
  msg.header.frame_id = "world";
  ROS_INFO("Publishing trajectory to /trajectory_generator/smooth_trajectory...");
  pub_trajectory_.publish(msg);

  return true;
}

