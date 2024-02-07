# 6-axis-Manipulator-Kinematics
The objective of this repo is to create a general environment which lets the user simulate in Matlab a general 6 DoF manipulator (for now UR5 and ABB IRb-7600) by plotting the links and the relative frames. \
The repo includes the application of Denvait-Hartenberg conventions for frame placement, definition of D-H parameters computation of Direct, Inverse, and Inverse Differential Kinematics, together with trajectory-generation functions based on different types of time functions (like cubic, quintic polynomials and LSPB).

[Here](https://github.com/d-aniele-carrar-o/ABB-IRb-7600-Direct-and-Inverse-Kinematics) you can find detailed explanation of the Direct and Inverse Kinematics for 6-axis manipulator ABB IRb-7600.

The explanation of how the repo is organized will be added soon.
For now, it is possible to download the souce files and run test_script.m in the main folder to see how the main features work.

This repo is still under active developement and aims to implement as much as possible for what concerns trajectory generation and execution.
