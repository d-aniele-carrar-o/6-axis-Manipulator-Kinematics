#!/usr/bin/env python3
"""
Simplified Camera Calibration using Checkerboard at Table Center

This approach:
1. Places checkerboard at world origin (table center)
2. Uses depth information for Z offset
3. Extracts X,Y axes from checkerboard orientation
4. Camera X = World Y, Camera Y = World X, Camera Z = -World Z
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import json

# Configuration
CHECKERBOARD_WIDTH = 9   # Inner corners
CHECKERBOARD_HEIGHT = 7
SQUARE_SIZE = 0.025      # 2.5cm squares
TABLE_HEIGHT = 0.76      # Table height from ground

def setup_camera():
    """Setup RealSense camera"""
    pipeline = rs.pipeline()
    config = rs.config()
    
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 6)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6)
    
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    
    return pipeline, align, profile

def get_camera_intrinsics(profile):
    """Get camera intrinsics"""
    color_stream = profile.get_stream(rs.stream.color)
    intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
    
    camera_matrix = np.array([
        [intrinsics.fx, 0, intrinsics.ppx],
        [0, intrinsics.fy, intrinsics.ppy],
        [0, 0, 1]
    ], dtype=np.float32)
    
    return camera_matrix, intrinsics

def capture_checkerboard(pipeline, align):
    """Capture single image with checkerboard"""
    print("Position checkerboard at table center. Press SPACE to capture.")
    
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            continue
            
        color_image = np.asanyarray(color_frame.get_data())
        cv2.imshow('Calibration - Press SPACE', color_image)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            cv2.destroyAllWindows()
            return color_image, depth_frame
        elif key == 27:  # ESC
            cv2.destroyAllWindows()
            return None, None

def detect_checkerboard_corners(color_image):
    """Detect and refine checkerboard corners"""
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCornersSB(gray, (CHECKERBOARD_WIDTH, CHECKERBOARD_HEIGHT), flags=cv2.CALIB_CB_EXHAUSTIVE)
    
    if not ret:
        raise ValueError("Checkerboard not detected")
    
    # Refine corners
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    
    return corners.reshape(-1, 2)

def extract_checkerboard_axes(corners):
    """Extract X and Y axes from checkerboard corners"""
    # Reshape corners to grid
    corners_grid = corners.reshape(CHECKERBOARD_HEIGHT, CHECKERBOARD_WIDTH, 2)
    
    # Get center point
    center_row = CHECKERBOARD_HEIGHT // 2
    center_col = CHECKERBOARD_WIDTH // 2
    center_point = corners_grid[center_row, center_col]
    
    # X-axis direction (horizontal in checkerboard)
    if center_col < CHECKERBOARD_WIDTH - 1:
        x_point = corners_grid[center_row, center_col + 1]
    else:
        x_point = corners_grid[center_row, center_col - 1]
        center_point, x_point = x_point, center_point
    
    # Y-axis direction (vertical in checkerboard)  
    if center_row < CHECKERBOARD_HEIGHT - 1:
        y_point = corners_grid[center_row + 1, center_col]
    else:
        y_point = corners_grid[center_row - 1, center_col]
        center_point, y_point = y_point, center_point
    
    return center_point, x_point, y_point

def get_depth_at_corners(depth_frame, corners):
    """Get average depth at checkerboard corners"""
    depths = []
    
    for corner in corners:
        x, y = int(corner[0]), int(corner[1])
        # Sample 3x3 area around corner
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                depth = depth_frame.get_distance(x + dx, y + dy)
                if depth > 0:
                    depths.append(depth)
    
    return np.mean(depths) if depths else 0

def compute_transformation(center_point, x_point, y_point, avg_depth, camera_matrix):
    """Compute camera-to-world transformation"""
    # Convert image points to normalized camera coordinates
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    
    def img_to_cam(point, depth):
        x_cam = (point[0] - cx) * depth / fx
        y_cam = (point[1] - cy) * depth / fy
        z_cam = depth
        return np.array([x_cam, y_cam, z_cam])
    
    # Get 3D points in camera coordinates
    center_3d = img_to_cam(center_point, avg_depth)
    x_3d = img_to_cam(x_point, avg_depth)
    y_3d = img_to_cam(y_point, avg_depth)
    
    print(f"DEBUG: Center 3D in camera coords: {center_3d}")
    print(f"DEBUG: X-axis 3D in camera coords: {x_3d}")
    print(f"DEBUG: Y-axis 3D in camera coords: {y_3d}")
    
    # Compute axes in camera coordinates
    x_axis_cam = normalize(x_3d - center_3d)  # Checkerboard X direction
    y_axis_cam = normalize(y_3d - center_3d)  # Checkerboard Y direction
    z_axis_cam = normalize(np.cross(x_axis_cam, y_axis_cam))  # Normal to checkerboard
    
    print(f"DEBUG: X-axis direction: {x_axis_cam}")
    print(f"DEBUG: Y-axis direction: {y_axis_cam}")
    print(f"DEBUG: Z-axis direction: {z_axis_cam}")
    
    # World coordinate system mapping:
    # Camera X = World Y, Camera Y = World X, Camera Z = -World Z
    x_axis_world = y_axis_cam    # World X = Camera Y
    y_axis_world = x_axis_cam    # World Y = Camera X  
    z_axis_world = -z_axis_cam   # World Z = -Camera Z
    
    # Rotation matrix from world to camera
    R_world_to_cam = np.column_stack([x_axis_world, y_axis_world, z_axis_world])
    
    # Camera position in world coordinates
    # The checkerboard center is at world origin (0,0,0) on the table
    # Camera position = checkerboard center position + camera offset
    camera_height = avg_depth + TABLE_HEIGHT
    
    # Camera offset from world origin (checkerboard center)
    # Since checkerboard is at world origin, camera offset = -center_3d projected to world
    camera_offset_world = np.array([
        -center_3d[1],  # World X = -Camera Y
        -center_3d[0],  # World Y = -Camera X
        camera_height   # World Z = camera height above ground
    ])
    
    print(f"DEBUG: Camera offset in world coords: {camera_offset_world}")
    
    # Build transformation matrix (camera to world)
    T_cam_to_world = np.eye(4)
    T_cam_to_world[:3, :3] = R_world_to_cam.T  # Inverse rotation
    T_cam_to_world[:3, 3] = camera_offset_world
    
    return T_cam_to_world, center_3d, x_axis_cam, y_axis_cam

def normalize(v):
    """Normalize vector"""
    norm = np.linalg.norm(v)
    return v / norm if norm > 0 else v

def main():
    """Main calibration function"""
    print("Simple Camera Calibration")
    print("=" * 30)
    
    # Setup camera
    pipeline, align, profile = setup_camera()
    camera_matrix, intrinsics = get_camera_intrinsics(profile)
    
    try:
        # Capture image
        color_image, depth_frame = capture_checkerboard(pipeline, align)
        if color_image is None:
            print("Capture cancelled")
            return
        
        # Detect checkerboard
        corners = detect_checkerboard_corners(color_image)
        print(f"Detected {len(corners)} corners")
        
        # Extract axes
        center_point, x_point, y_point = extract_checkerboard_axes(corners)
        print(f"Center: {center_point}")
        print(f"X-axis point: {x_point}")
        print(f"Y-axis point: {y_point}")
        
        # Get depth
        avg_depth = get_depth_at_corners(depth_frame, corners)
        print(f"Average depth: {avg_depth:.3f}m")
        
        # Compute transformation
        T_cam_to_world, center_3d, x_axis_cam, y_axis_cam = compute_transformation(center_point, x_point, y_point, avg_depth, camera_matrix)
        
        # Save results
        calibration_data = {
            "transformation_matrix": T_cam_to_world.tolist(),
            "camera_matrix": camera_matrix.tolist(),
            "average_depth": float(avg_depth),
            "table_height": TABLE_HEIGHT,
            "camera_height": float(avg_depth + TABLE_HEIGHT),
            "checkerboard_center_3d": center_3d.tolist()
        }
        
        with open("simple_calibration.json", "w") as f:
            json.dump(calibration_data, f, indent=2)
        
        print("\nCalibration Results:")
        print(f"Camera height above ground: {avg_depth + TABLE_HEIGHT:.3f}m")
        print("Transformation matrix:")
        print(T_cam_to_world)
        print("\nSaved to simple_calibration.json")
        
        # Enhanced Visualization
        vis_image = color_image.copy()
        
        # Draw checkerboard center (green)
        center_int = center_point.astype(int)
        cv2.circle(vis_image, tuple(center_int), 12, (0, 255, 0), -1)
        cv2.putText(vis_image, "CENTER", (center_int[0]-30, center_int[1]-20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Draw X-axis (red arrow)
        x_int = x_point.astype(int)
        cv2.arrowedLine(vis_image, tuple(center_int), tuple(x_int), (0, 0, 255), 3, tipLength=0.3)
        cv2.putText(vis_image, "X", (x_int[0]+10, x_int[1]), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        # Draw Y-axis (blue arrow)
        y_int = y_point.astype(int)
        cv2.arrowedLine(vis_image, tuple(center_int), tuple(y_int), (255, 0, 0), 3, tipLength=0.3)
        cv2.putText(vis_image, "Y", (y_int[0]+10, y_int[1]), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        
        # Add info text
        cv2.putText(vis_image, f"Depth: {avg_depth:.3f}m", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(vis_image, f"Camera Height: {avg_depth + TABLE_HEIGHT:.3f}m", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imwrite("calibration_visualization.png", vis_image)
        print("Enhanced visualization saved to calibration_visualization.png")
        
    finally:
        pipeline.stop()

if __name__ == "__main__":
    main()
