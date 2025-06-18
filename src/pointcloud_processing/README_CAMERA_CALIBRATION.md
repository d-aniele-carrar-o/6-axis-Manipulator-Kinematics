# Camera Calibration with Physical Calibration Frames

This document explains how to use the camera calibration scripts to determine the transformation between the camera frame and the world frame (table center) using physical calibration frames.

## Overview

Three calibration methods are provided:

1. **Single Frame Calibration** (`calibrate_camera_frame.m`): Uses one physical calibration frame with known dimensions and location.
2. **Dual Frame Calibration** (`calibrate_camera_dual_frames.m`): Uses two physical calibration frames at different known locations for more robust calibration.
3. **Table Edge-Based Calibration** (`calibrate_camera.m`): Uses the table edges to determine the calibration (existing method).

## Physical Calibration Frame

### Frame Design
- The calibration frame should have three perpendicular axes (X, Y, Z), each 8cm long.
- Each axis should be colored differently for easy detection (e.g., red for X, green for Y, blue for Z).
- The frame should be rigid and precisely manufactured (3D printing is recommended).

### Frame Placement
- Place the frame at a known location on the table surface.
- For dual frame calibration, place two frames at different known locations.
- Measure the frame locations as accurately as possible relative to the world origin (table center).

## Usage Instructions

### Single Frame Calibration

```matlab
% Setup parameters
tableParams = struct();
tableParams.height = 0.72;  % Table height in meters
tableParams.width = 1.2;    % Table width in meters
tableParams.length = 1.8;   % Table length in meters

frameParams = struct();
frameParams.axisLength = 0.08;  % 8cm axes
frameParams.location = [0, 0, tableParams.height];  % Frame at center of table

% Run calibration
rgbImagePath = 'path/to/rgb_image.png';
depthImagePath = 'path/to/depth_image.png';
tform_cam_to_world = calibrate_camera_frame(rgbImagePath, depthImagePath, frameParams, tableParams, true);

% Save the calibration result
save('camera_calibration.mat', 'tform_cam_to_world');
```

### Dual Frame Calibration

```matlab
% Setup parameters
tableParams = struct();
tableParams.height = 0.72;  % Table height in meters
tableParams.width = 1.2;    % Table width in meters
tableParams.length = 1.8;   % Table length in meters

dualFrameParams = struct();
dualFrameParams.axisLength = 0.08;  % 8cm axes
dualFrameParams.frame1Location = [-0.3, -0.3, tableParams.height];  % First frame location
dualFrameParams.frame2Location = [0.3, 0.3, tableParams.height];    % Second frame location
dualFrameParams.frame1Color = [1, 0, 0];  % Red for first frame
dualFrameParams.frame2Color = [0, 0, 1];  % Blue for second frame

% Run calibration
rgbImagePath = 'path/to/rgb_image.png';
depthImagePath = 'path/to/depth_image.png';
tform_cam_to_world = calibrate_camera_dual_frames(rgbImagePath, depthImagePath, dualFrameParams, tableParams, true);

% Save the calibration result
save('camera_calibration.mat', 'tform_cam_to_world');
```

### Using the Calibration Results

```matlab
% Load a saved calibration
load('camera_calibration.mat');

% Transform a point from camera coordinates to world coordinates
point_cam = [0.1, 0.2, 0.5];  % Point in camera coordinates
point_world = transformPointsForward(tform_cam_to_world, point_cam);

% Transform a point cloud from camera coordinates to world coordinates
ptCloud_world = pctransform(ptCloud_cam, tform_cam_to_world);
```

## Tips for Accurate Calibration

1. **Camera Setup**:
   - Position the camera to have a clear view of the entire table surface.
   - Ensure the camera is securely mounted to prevent movement during and after calibration.

2. **Lighting**:
   - Use consistent, even lighting to ensure good color detection.
   - Avoid direct reflections on the calibration frame.

3. **Frame Visibility**:
   - Ensure the calibration frame(s) are clearly visible in the camera view.
   - The frame should be large enough to be detected but not so large that it dominates the scene.

4. **Measurement Accuracy**:
   - Measure the table dimensions and frame locations as accurately as possible.
   - Use a measuring tape or ruler with millimeter precision.

5. **Multiple Calibrations**:
   - Perform multiple calibrations and average the results for better accuracy.
   - Check the consistency of the calibration results across multiple runs.

## Troubleshooting

1. **Frame Not Detected**:
   - Check that the frame is clearly visible in the camera view.
   - Ensure the lighting is adequate for color detection.
   - Try adjusting the color thresholds in the calibration script.

2. **Poor Calibration Results**:
   - Verify the accuracy of the measured frame location(s).
   - Check that the frame axes are truly perpendicular.
   - Ensure the camera is securely mounted and doesn't move during calibration.

3. **Inconsistent Results**:
   - Try the dual frame calibration method for more robust results.
   - Perform multiple calibrations and check for consistency.
   - Ensure the table surface is flat and stable.

## Advanced Usage

For more advanced calibration needs, consider:

1. **Custom Frame Design**:
   - Design a custom calibration frame with specific dimensions or features.
   - Modify the calibration scripts to accommodate your custom frame.

2. **Integration with Existing Systems**:
   - Integrate the calibration results with your existing robotic or vision system.
   - Use the transformation matrix to align camera data with world coordinates.

3. **Automatic Recalibration**:
   - Implement automatic recalibration procedures for long-term operation.
   - Use fixed markers in the environment for continuous calibration verification.