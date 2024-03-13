# 6-axis-Manipulator-Kinematics
The objective of this repo is to create a general environment which lets the user simulate in Matlab a general 6 DoF manipulator (for now UR5 and ABB IRb-7600 and custom 6-DoF manipulator) by plotting the links and the relative frames. If the urdf of the robot is available, it is possible to simulate using it. \
The repo includes the application of Denvait-Hartenberg conventions for frame placement, definition of D-H parameters, computation of Direct, Inverse, and Inverse Differential Kinematics, together with trajectory-generation functions based on different types of position/velocity profiles (cubic, quintic polynomials and LSPB).

[Here](https://github.com/d-aniele-carrar-o/ABB-IRb-7600-Direct-and-Inverse-Kinematics) you can find detailed explanation of the Direct and Inverse Kinematics for 6-axis manipulator ABB IRb-7600. \
[Here](https://github.com/d-aniele-carrar-o/ur5Robotics) you can find a project which uses UR5 with ROS for simulation and also setup for real robot utilization for a pick-and-place task with Computer Vision implemented.

The explanation of how the repo is organized will be added soon.
For now, it is possible to download the souce files and run testing_script.m in the 'scr' folder to see how the main features work. It is possible to edit the main parameters of kinematics, trajectory generation and simulation in the 'params' script.

This repo is still under active developement and aims to implement as much as possible for what concerns manipulator control and utilization.
