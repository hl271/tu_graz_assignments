# Drones' Lecture Assigment 2, 3 - Path Planner & Trajectory Generation
------------------------------------------

## I. Compilation instructions
------------------------

### 1. Install every dependency listed on the CMakeLists.txt and/or in the package.xml file. 
#### Important packages including:
**For assignment 2**:
- OMPL
- Octomap
- DynamicEDT3D

**For assignment 3**:
- mav_trajectory_generation

``sudo apt install ros-melodic-ompl*``

inside the src workspace folder: ``git clone https://github.com/ethz-asl/mav_comm.git``

**Integrating dynamic_edt_3d**

``sudo apt install ros-melodic-octovis* ros-melodic-octomap* ros-melodic-dynamic-edt-3d*``

### 2. Download this package inside the src folder of your ROS workspace.

### 3. Compile it using either the command 'catkin_make' or 'catkin build'.

## II. Testing instructions
--------------------
### Run ``roscore`` and ``rviz`` (in separate terminals)
### Run the following launch file (in separate terminals)
``` bash
roslaunch dla2_path_planner octomap_mapping_a2.launch

roslaunch dla2_path_planner trajectory_visualization.launch
```
### Run the following ROS nodes (in separate terminals)
```bash
rosrun dla2_path_planner dla2_path_planner_ros_node
# OR
rosrun dla2_path_planner dla2_path_planner_ros_node --runtime 5.0 --planner RRTStar -o WeightedLengthAndClearanceCombo -f planner_trajectory.txt --info 2

rosrun dla2_path_planner dla3_trajectory_generator_ros_node
```
### Configure rviz 
inside rviz add a "display" of typer "Marker" subscribed to the topic "/trajectory_visualization/trajectory_markers". Afterwards every new trajectory published by the planner should be visualized in rviz.

inside rviz add a "display" of type "OccupancyGrid" subscribed to the topic "/octomap_binarys". Afterwards the map for power_plant.bt should be visualized in rviz.

### Publish current and goal points to /path_planner

#### To change the initial/current position run:

2D (master branch):
``rostopic pub /path_planner/current_position mav_planning_msgs/Point2D "x: 0.1
y: 0.1" --once``

3D (when using 3D version):
``rostopic pub /path_planner/current_position geometry_msgs/Point "x: 0                                  
y: 0
z: 3" --once``

``rostopic pub /path_planner/current_position geometry_msgs/Point "x: -7                                  
y: -10
z: 2" --once``

#### To change the initial/goal position and plan a trajectory run:

2D (master branch):
``rostopic pub /path_planner/goal_position mav_planning_msgs/Point2D "x: 0.9
y: 0.9" --once``

**Test for different goal points**

3D (when using 3D version):
``rostopic pub /path_planner/goal_position geometry_msgs/Point "x: 10
y: -27
z: 10" --once``

``rostopic pub /path_planner/goal_position geometry_msgs/Point "x: 10
y: -20
z: 10" --once``

``rostopic pub /path_planner/goal_position geometry_msgs/Point "x: 27
y: -20
z: 5" --once``

### (Optional) To receive the planned trajectory on the terminal run:

``rostopic echo /path_planner/planned_trajectory``

