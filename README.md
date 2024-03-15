# 6-axis-Manipulator-Kinematics
The objective of this repo is to create a general environment which lets the user simulate in Matlab a general 6 DoF manipulator (for now UR5 and ABB IRb-7600 and custom 6-DoF manipulator) by plotting the links and the relative frames. If the urdf of the robot is available, it is possible to simulate using it. \
The repo includes the application of Denvait-Hartenberg conventions for frame placement, definition of D-H parameters, computation of Direct, Inverse, and Inverse Differential Kinematics, together with trajectory-generation functions based on different types of position/velocity profiles (cubic, quintic polynomials and LSPB).

[Here](https://github.com/d-aniele-carrar-o/ABB-IRb-7600-Direct-and-Inverse-Kinematics) you can find detailed explanation of the Direct and Inverse Kinematics for 6-axis manipulator ABB IRb-7600. \
[Here](https://github.com/d-aniele-carrar-o/ur5Robotics) you can find a project which uses UR5 with ROS for simulation and also setup for real robot utilization for a pick-and-place task with Computer Vision implemented.

How to use the code:
- clone the repository
- open the folder in Matlab
- go to "src" folder and run "params" script (click on "Add to path" if prompt message)
- compile c++ functions:
    - additional initial step for Windows users:
        - in the HOME tab, click on Add-Ons
        - search for MinGW and install it
    - run the "compile_cpp_code.m" script inside "cpp_src" folder
    - on MacOS you may need to install Xcode (or just Xcode Command Line Tools)
    - make sure that in the console you get "MEX completed successfully." for each ".cpp" file
- run "testing_script" and check that the 3D plot of the manipulator appears.

It is possible to edit the main parameters of kinematics, trajectory generation and simulation in the 'params' script.

In order to use the code for your custom manipulator, it is enough to add its D-H parameters table (following my convention) in "params" script and change the name of the variable "manipulator" to your newly added one. \
If you do not have the relative urdf, put the "real_robot" flag to false and the standard Matlab's plot3 function will be used to display the manipulator configuration and the trajectory simulation.

This repo is still under active developement and aims to implement as much as possible for what concerns manipulator control and utilization.
