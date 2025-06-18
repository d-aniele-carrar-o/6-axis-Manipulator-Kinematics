# Robot Base Calibration

This directory contains scripts for calibrating the transformation between the world reference frame and the robot base reference frames.

## Overview

When working with dual robot setups, it's important to know the precise transformation between the world reference frame and each robot's base reference frame. This is especially important when the robots need to collaborate or work in a shared workspace.

The calibration process uses known poses in the world frame and the corresponding joint configurations to compute the transformation matrices.

## Files

- `compute_robot_base_transforms.m`: Main function to compute the transformation matrices
- `direct_kinematics_wrapper.m`: Wrapper for the direct kinematics function
- `calibrate_robot_bases.m`: Simple script to demonstrate the calibration process
- `robot_base_calibration_example.m`: Detailed example script showing the complete calibration process

## How to Use

### Step 1: Collect Calibration Data

1. Define at least 3 points in the world reference frame where you can accurately position the robot end-effectors
2. Move each robot to these points and record:
   - The position and orientation of each point in the world frame
   - The joint configuration of each robot when its end-effector is at the point

### Step 2: Run the Calibration

```matlab
% Define your calibration data
poses_left = [
    % [x, y, z, rx, ry, rz] for point 1
    % [x, y, z, rx, ry, rz] for point 2
    % [x, y, z, rx, ry, rz] for point 3
];

poses_right = [
    % [x, y, z, rx, ry, rz] for point 1
    % [x, y, z, rx, ry, rz] for point 2
    % [x, y, z, rx, ry, rz] for point 3
];

q_left = [
    % Joint configuration for point 1
    % Joint configuration for point 2
    % Joint configuration for point 3
];

q_right = [
    % Joint configuration for point 1
    % Joint configuration for point 2
    % Joint configuration for point 3
];

% Known parameters
stand_height = 0.215;  % Height of the robot stands
y_distance = 1.1;      % Distance between robot stands along Y axis

% Compute transformations
[T_world_base_left, T_world_base_right] = compute_robot_base_transforms(poses_left, poses_right, q_left, q_right, stand_height, y_distance);

% Save the results
save('calibrated_robot_bases.mat', 'T_world_base_left', 'T_world_base_right');
```

### Step 3: Use the Calibrated Transformations

```matlab
% Load the calibrated transformations
load('calibrated_robot_bases.mat');

% Update the robot base transformations in your code
robot_base_transforms = {
    T_world_base_left,  % Left robot
    T_world_base_right  % Right robot
};
```

## Example

For a complete example of the calibration process, run:

```matlab
robot_base_calibration_example
```

This script demonstrates:
1. How to collect calibration data
2. How to compute the transformations
3. How to visualize the results
4. How to verify the calibration accuracy

## Notes

- The calibration accuracy depends on the accuracy of your measurements
- Using more calibration points can improve the accuracy
- The calibration process assumes that the robots are rigidly mounted and don't move relative to the world frame
- The stand height and Y distance are used as constraints to improve the calibration accuracy