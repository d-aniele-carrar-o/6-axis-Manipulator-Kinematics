# Intel RealSense D415 Camera Calibration Guide

This guide provides detailed instructions for calibrating an Intel RealSense D415 camera mounted approximately 1 meter above a table surface using a checkerboard pattern.

## Recommended Checkerboard Parameters

### Optimal Specifications
- **Total Size**: 30-40 cm wide
- **Square Size**: 3.5 cm (35 mm) per square
- **Pattern Dimensions**: 8×10 squares (7×9 inner corners)
- **Material**: Matte paper mounted on rigid board (foam core or cardboard)

### Why These Specifications?
- The D415 has a field of view of approximately 69° × 42° at 1 meter distance
- 3.5 cm squares provide good visibility from 1 meter height
- Matte paper reduces reflections that can interfere with detection
- Rigid backing ensures the pattern remains perfectly flat

## Preparation Steps

1. **Print the Checkerboard**
   - Use a laser printer for best contrast
   - Print at actual size (no scaling)
   - Verify square size with a ruler after printing

2. **Mount the Pattern**
   - Attach to rigid backing using spray adhesive
   - Ensure the pattern is completely flat
   - Trim edges if necessary

3. **Measure Precisely**
   - Measure the exact size of several squares with a caliper
   - Calculate the average square size in meters
   - Record this value for the calibration script

4. **Camera Setup**
   - Mount the camera securely to prevent movement
   - Position approximately 1 meter above the table
   - Ensure the camera is roughly perpendicular to the table

5. **Camera Settings**
   - Set to High Accuracy depth preset
   - Disable auto-exposure during calibration
   - Set fixed white balance
   - Configure depth units to millimeters

## Calibration Procedure

1. **Place the Checkerboard**
   - Position at the center of the table
   - Ensure it's flat and fully visible in the camera view
   - Record the exact position relative to your world coordinate system

2. **Capture Images**
   - Take both RGB and depth images
   - Ensure good, even lighting without shadows or glare
   - Capture multiple sets with the checkerboard in different positions

3. **Run Calibration Script**
   ```matlab
   % Calibration script for Intel RealSense D415
   clear; clc; close all;

   % World parameters
   worldParams = struct();
   worldParams.tableHeight = 0.72;  % Adjust to your table height

   % Checkerboard parameters optimized for RealSense at 1m height
   checkerboardParams = struct();
   checkerboardParams.squareSize = 0.035;  % 3.5cm squares
   checkerboardParams.patternSize = [7, 9];  % 8×10 squares (7×9 inner corners)
   checkerboardParams.worldLocation = [0, 0, worldParams.tableHeight];  % Center of table

   % Paths to your images
   rgbImagePath = 'path/to/rgb_image.png';
   depthImagePath = 'path/to/depth_image.png';

   % Run calibration
   tform_cam_to_world = calibrate_camera_checkerboard(rgbImagePath, depthImagePath, 
                                                    checkerboardParams, worldParams, true);

   % Save the calibration result
   save('realsense_d415_calibration.mat', 'tform_cam_to_world');
   ```

4. **Multiple Calibrations**
   - Perform 3-5 calibrations with the checkerboard in different positions
   - Average the resulting transformations for better accuracy
   ```matlab
   % Load multiple calibrations
   load('calibration1.mat');
   tform1 = tform_cam_to_world;
   load('calibration2.mat');
   tform2 = tform_cam_to_world;
   load('calibration3.mat');
   tform3 = tform_cam_to_world;

   % Average rotations using quaternions
   q1 = rotm2quat(tform1.R);
   q2 = rotm2quat(tform2.R);
   q3 = rotm2quat(tform3.R);

   % Ensure quaternions are in the same hemisphere
   if dot(q1, q2) < 0, q2 = -q2; end
   if dot(q1, q3) < 0, q3 = -q3; end

   q_avg = mean([q1; q2; q3], 1);
   q_avg = q_avg / norm(q_avg);  % Normalize

   % Average translations
   t_avg = mean([tform1.T; tform2.T; tform3.T], 1);

   % Create final transformation
   tform_cam_to_world = rigidtform3d(quat2rotm(q_avg), t_avg);
   save('realsense_d415_calibration_final.mat', 'tform_cam_to_world');
   ```

## Verification and Testing

1. **Visual Verification**
   - Transform the point cloud to world coordinates
   - Verify that the table appears flat and at the correct height
   - Check that vertical objects are properly aligned

2. **Quantitative Verification**
   - Place objects at known positions on the table
   - Measure the error between expected and actual positions
   - Typical accuracy should be within 2-5 mm at 1 meter distance

3. **Troubleshooting**
   - If corners aren't detected, try adjusting lighting
   - If depth alignment is poor, check camera's depth-RGB alignment
   - If results are inconsistent, ensure the camera is firmly mounted

## RealSense D415 Specific Notes

- **Depth Quality**: The D415 provides better accuracy for well-textured surfaces
- **Depth Range**: Optimal accuracy is between 0.5-2 meters
- **Depth Filtering**: Use minimal post-processing for calibration
- **SDK Settings**: Use the RealSense SDK to disable auto-exposure during calibration
- **Temperature**: Allow the camera to warm up for 15-20 minutes before calibration

## Advanced Tips

1. **Multi-Position Calibration**
   - For highest accuracy, use multiple checkerboard positions
   - Place the checkerboard at different locations and orientations
   - Combine all detected corners into a single calibration

2. **Intrinsic Calibration**
   - Consider calibrating intrinsic parameters if highest accuracy is needed
   - Use the RealSense built-in calibration tool or OpenCV's calibration functions
   - Store intrinsic parameters and use them in the extrinsic calibration

3. **Regular Recalibration**
   - Recalibrate if the camera is moved
   - Consider periodic recalibration (monthly) for critical applications
   - Monitor calibration quality over time

## Resources

- [Intel RealSense D415 Documentation](https://www.intelrealsense.com/depth-camera-d415/)
- [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html)
- [MATLAB Camera Calibration Documentation](https://www.mathworks.com/help/vision/ug/camera-calibration.html)