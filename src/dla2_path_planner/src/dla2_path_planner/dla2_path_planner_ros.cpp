/*
 * DLA2 Path Planner ROS - dla2_path_planner_ros.cpp
 *
 *  Author: Jesus Pestana <pestana@icg.tugraz.at>
 *  Created on: Dec 19, 2019
 *
 */

#include <dla2_path_planner/dla2_path_planner_ros.h>

DLA2PathPlanner::DLA2PathPlanner(ros::NodeHandle &n, ros::NodeHandle &pn, int argc, char** argv) :
    pnode_(pn),
    node_(n),
    traj_planning_successful(false)
{   
    // ROS topics
    current_position_sub = pnode_.subscribe("current_position", 10, &DLA2PathPlanner::currentPositionCallback, this);
    goal_position_sub = pnode_.subscribe("goal_position", 10, &DLA2PathPlanner::goalPositionCallback, this);
    trajectory_pub_raw = pnode_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("planned_trajectory_raw", 1);
    trajectory_pub = pnode_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("planned_trajectory", 1);

    current_position.x = 0.; current_position.y = 0.; current_position.z = 0.;
    goal_position.x = 1.; goal_position.y = 1.; goal_position.z = 1.;

    // Parse the arguments, returns true if successful, false otherwise
    if (argParse(argc, argv, &runTime, &plannerType, &objectiveType, &outputFile))
    {
        // Return with success
        ROS_INFO("DLA2PathPlanner::DLA2PathPlanner(...) argParse success!");
    } else {
        // Return with error - Modified argParse to make this equivalent to giving no arguments.
        ROS_INFO("DLA2PathPlanner::DLA2PathPlanner(...) argParse error!");
    }
}

DLA2PathPlanner::~DLA2PathPlanner() {

}

void DLA2PathPlanner::currentPositionCallback(const geometry_msgs::Point::ConstPtr& p_msg) {
    current_position = *p_msg;
    ROS_INFO_STREAM("New current position, x: " << current_position.x << "; y: " << current_position.y << "; z: " << current_position.z);
}

void DLA2PathPlanner::goalPositionCallback(const geometry_msgs::Point::ConstPtr& p_msg) {
    goal_position = *p_msg;
    ROS_INFO_STREAM("New goal position, x: " << goal_position.x << "; y: " << goal_position.y << "; z: " << goal_position.z);

    plan();

    if (traj_planning_successful) {
        convertOMPLPathToMsg();
        trajectory_pub_raw.publish(last_traj_msg);
        convertOMPLPathSimplifiedToMsg();
        trajectory_pub.publish(last_traj_msg);
        mav_planning_msgs::PolynomialTrajectory4D::Ptr p_traj_msg = 
            mav_planning_msgs::PolynomialTrajectory4D::Ptr( new mav_planning_msgs::PolynomialTrajectory4D( last_traj_msg ) );
    }
}

void DLA2PathPlanner::convertOMPLPathSimplifiedToMsg() {
    //Reference &msg -> just another name for last_traj_msg
    mav_planning_msgs::PolynomialTrajectory4D &msg = last_traj_msg;
    msg.segments.clear();

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world"; // "odom"

    std::vector<ompl::base::State *> &states = p_simplified_traj_ompl->getStates();
    size_t N = states.size();
    for (size_t i = 0; i<N; i++) {
        ompl::base::State *p_s = states[i];
        const double &x_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[0];
        const double &y_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[1];
        const double &z_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[2];
        // double z_s = 0.;
        double yaw_s = 0.;
        ROS_INFO_STREAM("Simplfied path: states["<< i <<"], x_s: " << x_s << "; y_s: " << y_s << "; z_s: " << z_s);

        mav_planning_msgs::PolynomialSegment4D segment;
        segment.header = msg.header;
        segment.num_coeffs = 0;
        segment.segment_time = ros::Duration(0.);
        
        segment.x.push_back(x_s);
        segment.y.push_back(y_s);
        segment.z.push_back(z_s);
        segment.yaw.push_back(yaw_s);
        msg.segments.push_back(segment);
    }
}
// TODO: Set param to input (p_last_traj_ompl || p_simplified_traj_ompl) into the below func
void DLA2PathPlanner::convertOMPLPathToMsg() {
    //Reference &msg -> just another name for last_traj_msg
    mav_planning_msgs::PolynomialTrajectory4D &msg = last_traj_msg;
    msg.segments.clear();

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world"; // "odom"

    std::vector<ompl::base::State *> &states = p_last_traj_ompl->getStates();
    size_t N = states.size();
    for (size_t i = 0; i<N; i++) {
        ompl::base::State *p_s = states[i];
        const double &x_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[0];
        const double &y_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[1];
        const double &z_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[2];
        // double z_s = 0.;
        double yaw_s = 0.;
        ROS_INFO_STREAM("states["<< i <<"], x_s: " << x_s << "; y_s: " << y_s << "; z_s: " << z_s);

        mav_planning_msgs::PolynomialSegment4D segment;
        segment.header = msg.header;
        segment.num_coeffs = 0;
        segment.segment_time = ros::Duration(0.);
        
        segment.x.push_back(x_s);
        segment.y.push_back(y_s);
        segment.z.push_back(z_s);
        segment.yaw.push_back(yaw_s);
        msg.segments.push_back(segment);
    }
}

bool DLA2PathPlanner::isValidPath() {
    std::vector<ompl::base::State *> &states = p_last_traj_ompl->getStates();
    size_t N = states.size();
    for (size_t i = 1; i<N; i++) {
        ompl::base::State *prev_s = states[i-1];
        ompl::base::State *current_s = states[i];
        const double &prev_x = prev_s->as<ob::RealVectorStateSpace::StateType>()->values[0];
        const double &prev_y = prev_s->as<ob::RealVectorStateSpace::StateType>()->values[1];
        const double &prev_z = prev_s->as<ob::RealVectorStateSpace::StateType>()->values[2];
        
        const double &current_x = current_s->as<ob::RealVectorStateSpace::StateType>()->values[0];
        const double &current_y = current_s->as<ob::RealVectorStateSpace::StateType>()->values[1];
        const double &current_z = current_s->as<ob::RealVectorStateSpace::StateType>()->values[2];
        
        octomap::point3d prev_point(prev_x, prev_y, prev_z);
        octomap::point3d current_point(current_x, current_y, current_z);
        octomap::point3d direction(current_x - prev_x, current_y - prev_y, current_z - prev_z);
        float segment_length = sqrt(pow(current_x - prev_x,2)+pow(current_y - prev_y,2)+pow(current_z - prev_z,2));
        octomap::point3d endpoint;
        bool isObstacle = tree->castRay(prev_point, direction, endpoint, true, segment_length);
        std::cout << "\nPrev point: " << prev_point << std::endl;
        std::cout << "Current point: " << current_point << std::endl;
        std::cout << "Direction vector: " << direction << std::endl;
        std::cout << "Segment length: " << segment_length << std::endl;
        std::cout << "Endpoint: " << endpoint << std::endl;
        if (isObstacle) {
            std::cout << "Collide with obstacle!" << std::endl;
            return false;
        };
    }
    std::cout << "Path is valid!!" <<std::endl;
    return true;
}

void DLA2PathPlanner::run_simplifier(const ob::SpaceInformationPtr &si, int runs) {
    ob::OptimizationObjectivePtr obj(new ob::MaximizeMinClearanceObjective(si));
    og::PathSimplifier simplifier(si, ob::GoalPtr(), obj);

    double avg_costs=0.0;
    ob::Cost original_cost = p_simplified_traj_ompl->cost(obj);
    for (int i=0; i<runs; i++) {
        simplifier.shortcutPath(*p_simplified_traj_ompl, 100, 100, 0.33, 0.025);
        std::cout << "Cost of new path is: " << p_simplified_traj_ompl->cost(obj).value() << std::endl;
        avg_costs += p_simplified_traj_ompl->cost(obj).value();
    }
    avg_costs /= runs;
    std::cout << "Avg cost: " << avg_costs << "; Original cost: " << original_cost.value() << std::endl;
}

void DLA2PathPlanner::run_perturber(const ob::SpaceInformationPtr &si, int runs) {
    ob::OptimizationObjectivePtr obj(new ob::MaximizeMinClearanceObjective(si));
    og::PathSimplifier simplifier(si, ob::GoalPtr(), obj);

    double avg_costs=0.0;
    ob::Cost original_cost = p_simplified_traj_ompl->cost(obj);
    for (int i=0; i<runs; i++) {
        // Pass in reference (or just another name of the type variable), REFERENCE != POINTER => pass in *Pointer
        simplifier.perturbPath(*p_simplified_traj_ompl, 2.0, 100, 100, 0.025);
        std::cout << "Cost of new path (perturber) is: " << p_simplified_traj_ompl->cost(obj).value() << std::endl;
        avg_costs += p_simplified_traj_ompl->cost(obj).value();
    }
    avg_costs /= runs;
    std::cout << "PERTURBER: Avg cost: " << avg_costs << "; Original cost: " << original_cost.value() << std::endl;
    
}

void DLA2PathPlanner::run_BSpline(const ob::SpaceInformationPtr &si, int pass) {
    ob::OptimizationObjectivePtr obj(new ob::MaximizeMinClearanceObjective(si));
    og::PathSimplifier simplifier(si, ob::GoalPtr(), obj);
    simplifier.smoothBSpline(*p_simplified_traj_ompl, pass);
}
void DLA2PathPlanner::plan()
{
    
    tree = new octomap::OcTree(0.05);

    // Read map.bt file into the ocTree obj
    std::string path = ros::package::getPath("dla2_path_planner");        
    std::cout << path << "/maps/power_plant.bt \n";
    tree->readBinary(path+"/maps/power_plant.bt");

    std::cout <<"read in tree, "<<tree->getNumLeafNodes()<<" leaves "<<std::endl;

    double x,y,z;
    tree->getMetricMin(x,y,z); // Output minimun value of the bounding space to x,y,z
    octomap::point3d min(x,y,z); // Create variable min of type 3dpoint
    //std::cout<<"Metric min: "<<x<<","<<y<<","<<z<<std::endl;
    tree->getMetricMax(x,y,z);
    octomap::point3d max(x,y,z);
    //std::cout<<"Metric max: "<<x<<","<<y<<","<<z<<std::endl;

    //Construct new distmap 
    bool unknownAsOccupied = true;
    unknownAsOccupied = false;
    float maxDist = 1.0;
    
    //- the first argument ist the max distance at which distance computations are clamped
    //- the second argument is the octomap
    //- arguments 3 and 4 can be used to restrict the distance map to a subarea
    //- argument 5 defines whether unknown space is treated as occupied or free
    //The constructor copies data but does not yet compute the distance map

    distmap = new DynamicEDTOctomap(maxDist, tree, min, max, unknownAsOccupied);    

    distmap->update();
    // Construct the robot state space in which we're planning. 
    // IMPORTANT: Add dimension in accordance with the map's dimensions
    auto space(std::make_shared<ob::RealVectorStateSpace>());
    space->addDimension(min.x(), max.x()); 
    space->addDimension(min.y(), max.y());
    space->addDimension(min.z(), max.z());

    std::cout << "[ ] space dimension: " << space->getDimension() << "\n";

    // Construct a space information instance for this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));


    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(std::make_shared<ValidityChecker>(si));

    si->setup();

    // Set our robot's starting state to be the bottom-left corner of
    // the environment, or (0,0).
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = current_position.x;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = current_position.y;
    start->as<ob::RealVectorStateSpace::StateType>()->values[2] = current_position.z;

    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_position.x;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_position.y;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal_position.z;

    // Create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // ** Specify OptimizationObjective Type
    pdef->setOptimizationObjective(getClearanceObjective(si));

    // Construct the optimal planner specified by our command line argument.
    // This helper function is simply a switch statement.

    // ** Specify planner Type 
    // ? Why this still points to type <ob::Planner>?: auto optimizingPlanner(allocatePlanner(si, plannerType));
    auto optimizingPlanner(std::make_shared<og::RRTstar>(si));
    // ** Set maximun range (step size) for RRTStar planner
    optimizingPlanner->setRange(15.0);
    
    // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    // **IMPORTANT: The solve() method belongs to ob::Planner (inherited class of <og::<plannerType>>)
    // TODO : Understand the meaning of the casting below
    ob::PlannerStatus solved = optimizingPlanner->ob::Planner::solve(2.0);

    if (solved)
    {
        pathPtr = pdef->getSolutionPath();
        p_last_traj_ompl =  std::static_pointer_cast<ompl::geometric::PathGeometric>(pathPtr);
        
        if (isValidPath() ) {
            
            // Output the length of the path found
            std::cout
                << optimizingPlanner->getName()
                << " found a solution of length "
                << pdef->getSolutionPath()->length()
                << " with an optimization objective value of "
                << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()) << std::endl;

            // ** Question: Can we pass a shared_pointer of simplified path to both of the below functions?
            og::PathGeometric path(dynamic_cast<const og::PathGeometric&>(*pathPtr));
            p_simplified_traj_ompl = std::make_shared<og::PathGeometric>(path);
            run_simplifier(si, 20);
            run_perturber(si, 20);
            run_BSpline(si, 1);
            // int runs = 20;
            

            // If a filename was specified, output the path as a matrix to
            // that file for visualization
            if (!outputFile.empty())
            {
                std::ofstream outFile(outputFile.c_str());
                std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->
                    printAsMatrix(outFile);
                outFile.close();
            }

            
            traj_planning_successful = true;

        } else {
            std::cout << "This path go through walls!! No solution!" << std::endl;
            traj_planning_successful = false;
        }
    } else {
        std::cout << "No solution found." << std::endl;
        traj_planning_successful = false;
    }
    delete tree;
}
