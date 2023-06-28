# TurtleBot3 Coverage Path Planning

## 1. Creating a workspace.
```
mkdir turtlebot3_ws
cd turtlebot3_ws
mkdir src
cd src
```
____
## 2. Cloning packages from therepository.
```
git clone https://github.com/vladislav-parkhachev/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
____
## 3. Assembling the workspace.
```
cd ..
catkin_make
```
____
## 4. Starting the system. Open a new terminal window and run the simulation in Gazebo.
```
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
____
## 5. Open a new terminal window and launch gmapping.
```
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_slam turtlebot3_gmapping.launch
```
____
## 6. Open a new terminal window and run move_base.
```
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```
____
## 7 Open a new terminal window and launch the nodes of the package depth_first_search_node.launch
```
source devel/setup.bash
roslaunch turtlebot3_coverage_path_planning depth_first_search_node.launch
```
____


