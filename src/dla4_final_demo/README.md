# Drones' Lecture Assigment 4 - Final Demo
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
roslaunch dla4_final_demo octomap_mapping_a2.launch

roslaunch dla4_final_demo trajectory_visualization.launch
```
### Run the following ROS nodes (in separate terminals)
```bash
rosrun dla4_final_demo path_planner_ros_node

rosrun dla4_final_demo trajectory_generator_ros_node
```
### Configure rviz 
inside rviz add a "display" of typer "Marker" subscribed to the topic "/trajectory_visualization/trajectory_markers". Afterwards every new trajectory published by the planner should be visualized in rviz.

inside rviz add a "display" of type "OccupancyGrid" subscribed to the topic "/octomap_binarys". Afterwards the map for power_plant.bt should be visualized in rviz.

### Publish current and goal points to /path_planner

#### To change the initial/current position run:

- WP0: {x: -1.75, y: 0.00, z: 1.50}
- WP1= {x: -0.03, y: 0.13, z: 0.35}
- WP2= {x: +2.25, y: 1.50, z: 1.50}
- WP3= {x: -0.03, y: 0.13, z: 1.10}
- WP4= WP0

``rostopic pub /path_planner/current_position geometry_msgs/Point "x: -1.75                                  
y: 0.00
z: 1.50" --once``

``rostopic pub /path_planner/current_position geometry_msgs/Point "x: -0.03                                  
y: 0.13
z: 0.35" --once``

``rostopic pub /path_planner/current_position geometry_msgs/Point "x: 2.25                                  
y: 1.50
z: 1.50" --once``

``rostopic pub /path_planner/current_position geometry_msgs/Point "x: -0.03                                  
y: 0.13
z: 1.10" --once``

#### To change the initial/goal position and plan a trajectory run:

``rostopic pub /path_planner/goal_position geometry_msgs/Point "x: -0.03                                  
y: 0.13
z: 0.35" --once``

``rostopic pub /path_planner/goal_position geometry_msgs/Point "x: 2.25                                  
y: 1.50
z: 1.50" --once``

``rostopic pub /path_planner/goal_position geometry_msgs/Point "x: -0.03                                  
y: 0.13
z: 1.10" --once``

``rostopic pub /path_planner/goal_position geometry_msgs/Point "x: -1.75                                  
y: 0.00
z: 1.50" --once``

### (Optional) To receive the planned trajectory on the terminal run:

``rostopic echo /path_planner/planned_trajectory``
