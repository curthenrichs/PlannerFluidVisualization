# planner_fluid_visualization
This is a ROS package that visualizes robotic arm's end-effector as a perturbation of the 2D fluid. Where the x,y of the pose is mapped directly as offsets from screen center and z of pose controls the radius of the perturbation. Color is changed by the orientation of the end-effector as it moves through its trajectory.

## Story
While going through Github's daily trending repositories list one Friday afternoon I found a [2D WebGL fluid simulation](https://github.com/PavelDoGreat/WebGL-Fluid-Simulation) that felt
interesting when clicking and moving through points on the screen with my mouse. So as any sane person would do, I asked myself what would this thing do if it were a ROS node. The answer is visualize the arm motion paths of robotic manipulations.

I then set out that night, focused on building the core ROS web node around the current fuild simulation codebase and also working on a demo backend. At first I implemented for MoveIt which is a subsystem built for ROS that handles planning and control of both physical and simulated robots. Specifically, I am using the OMPL set of default planners configured for the UR3e robot from Universal Robotics.

Over the weekend I added support for Relaxed-IK which is developed to perform real-time smooth IK for robot control. I needed to write a simple linear interpolation function that create an end-effector trajectory that Relaxed-IK can more realistically follow. Then I added support for Lively-IK, which is an offshoot of Relaxed-IK that adds Perlin noise in order to have the robot appear more life-like.    

I wrapped all of this up as a demo application that can be launched. I think it is interesting to look at. Perhaps one could hook it up to a real robot's end-effector state as it performs some useful task in the future.

## Setup

This package requires the following other ROS packages to be installed. In the case of Relaxed-IK install either Juila, Python, or both variant(s) as the performance doesn't really matter for the demo. For Lively-IK follow the installation instructions (should be Julia based).

- [MoveIt](http://wiki.ros.org/moveit)
- [uwgraphics/relaxed_ik](https://github.com/uwgraphics/relaxed_ik)
- [Wisc-HCI/lively_ik](https://github.com/Wisc-HCI/lively_ik)
- [ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot)

We now need to setup Relaxed-IK. In this package, there is a directory called rik_setup which contains the three files needed to use Relaxed-IK. Copy these to their appropriate locations (replace existing if necessary) then run through the setup procedure.

Lively-IK setup is much the same except copy the files from lik_setup instead.

## Running Demo
To run the demo I built to showcase this ROS Fluid Simulation node, enter the following into a terminal.

```
roslaunch planner_fluid_visualization demo.launch planner:=rik
```

All is working if RVIZ shows up with the robot moving around and when you navigate to `http:\\localhost:8080` the fluid simulation pulses grey perturbations then starts to move with the robot.

## Custom Paths
A simple utility was made to capture a path using RVIZ and MoveIT into the JSON form needed for this package. Simply enter the following into a terminal. Note, the path is always relative to the paths directory in the package and that the file should not include the `.json` extension.

```
roslaunch planner_fluid_visualization saver.launch path_file:=my_path_file
```

Then to save a pose press Enter. Move the end-effector in RVIZ and save again (repeating until satisfied). When done press `q` and then Enter.

Now just launch the demo with an additional argument `path_file:=my_path_file`.
