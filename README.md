# planner_fluid_visualization
This is a ROS package that visualizes robotic arm's end-effector as a perturbation of the 2D fluid. Where the x,y of the pose is mapped directly as offsets from screen center and z of pose controls the radius of the perturbation. Orientation of the end-effector modifies the color at the perturbation.

## Story
While going through Github's trending repositories list one Friday afternoon I found a [2D WebGL fluid simulation](https://github.com/PavelDoGreat/WebGL-Fluid-Simulation) that was interesting when clicking and moving through points on the screen with my mouse. So as any sane person would do, I asked myself how would this look if it were a ROS node. The answer is visualize the arm motion paths of robotic manipulations.

I then set out that night, focused on building the core ROS web node around the current fuild simulation codebase and also working on a demo backend. At first I implemented under MoveIt, which is a subsystem built for ROS that handles planning and control of both physical and simulated robots. Specifically, I am using the OMPL set of default planners configured for the UR3e robot from Universal Robotics.

Over the weekend I added support for Relaxed-IK which is developed to perform real-time smooth IK for robot control. I needed to write a simple linear interpolation function that create an end-effector trajectory that Relaxed-IK can more realistically follow. Then I added support for Lively-IK, which is an offshoot of Relaxed-IK that adds Perlin noise in order to have the robot appear more life-like.    

I wrapped all of this up as a demo application that can be launched. I think it is interesting to look at. Perhaps one could hook it up to a real robot's end-effector state as it performs some useful task in the future.

*Update* I reworked this repository to use my [ur3e_real_time_motion_playground](https://github.com/curthenrichs/ur3e_real_time_motion_playground) code-base. 

## Setup

This package requires the following other ROS packages to be installed.

- [curthenrichs/ur3e_real_time_motion_playground](https://github.com/curthenrichs/ur3e_real_time_motion_playground)
- [ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot)

## Running Demo
To run the demo I built to showcase this ROS Fluid Simulation node, enter the following into a terminal.

```
roslaunch planner_fluid_visualization demo.launch planner:=rik
```

All is working if RVIZ shows up with the robot moving around and when you navigate to `http:\\localhost:8080` the fluid simulation pulses grey perturbations then starts to move with the robot.

## Custom Paths
A simple utility was made in the motion playground to capture a path using RVIZ and MoveIT into the JSON form needed for this package. Simply enter the following into a terminal. Note, the path is always relative to the paths directory in the package and that the file should not include the `.json` extension.

```
roslaunch ur3e_real_time_motion_playground saver.launch path_file:=my_path_file
```

Then to save a pose press Enter. Move the end-effector in RVIZ and save again (repeating until satisfied). When done press `q` and then Enter.

Now just launch the demo with an additional argument `path_file:=my_path_file`.
