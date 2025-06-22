import os
os.environ['OPENCV_OPENCL_RUNTIME'] = ''
os.environ['OPENCV_OPENCL_DEVICE'] = 'disabled'
os.environ['OPENCV_OPENCL_BINARY_CACHE_ENABLE'] = '0'
import cv2
import numpy as np

cv2.ocl.setUseOpenCL(False)

# Inner number of squares
CB_W, CB_H = 10-1, 8-1
CB_DIMS = (CB_W, CB_H)
SQUARE_SIZE = 0.026


def draw_on_image(img, title, corners):
    center = corners.mean(0)

    # Reshape corners for easy manipulation
    x_coords = corners[:, 0].reshape(CB_H, CB_W).T
    y_coords = corners[:, 1].reshape(CB_H, CB_W)
    
    # Draw coordinate axes
    center_pts_x = np.array(list(zip(x_coords.mean(0), y_coords.mean(1))))
    center_pts_y = np.array(list(zip(x_coords.mean(1), y_coords.mean(0))))

    # X-axis (red)
    min_x_point = center_pts_x[np.argmin(center_pts_x[:,0])]
    cv2.arrowedLine(img, (int(center[0]), int(center[1])), (int(min_x_point[0]), int(min_x_point[1])), (0, 0, 255), 8)
    cv2.putText(img, 'X', (int(min_x_point[0] + 10), int(min_x_point[1] + 10)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    # Y-axis (green)
    max_y_point = center_pts_y[np.argmax(center_pts_y[:,1])]
    cv2.arrowedLine(img, (int(center[0]), int(center[1])), (int(max_y_point[0]), int(max_y_point[1])), (0, 255, 0), 8)
    cv2.putText(img, 'Y', (int(max_y_point[0] + 10), int(max_y_point[1] + 10)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    cv2.imshow(title, img)

def extrinsic_calibration(corners):
    objp = np.zeros(((CB_W + 1) * (CB_H + 1), 3), np.float32)
    objp[:, :2] = np.mgrid[0:(CB_W + 1), 0:(CB_H + 1)].T.reshape(-1, 2) * SQUARE_SIZE

    # Adjust objp if your world origin is the center of the chessboard
    # For example, if the top-left corner of the pattern is (0,0), and you want
    # the center of the pattern to be (0,0,0) in your world frame:
    center_x_offset = CB_W / 2.0 * SQUARE_SIZE
    center_y_offset = CB_H / 2.0 * SQUARE_SIZE
    objp[:, 0] -= center_x_offset
    objp[:, 1] -= center_y_offset

    # Assuming `object_points` are the 3D coordinates of the chessboard corners in your world frame
    # and `image_points` are the 2D pixel coordinates of those corners in the camera image.
    # `camera_matrix` and `dist_coeffs` are obtained from intrinsic calibration.
    retval, rvec, tvec = cv2.solvePnP(objp, corners, camera_matrix, dist_coeffs)

def main():
    img_paths = [f"output/camera_calibration/{img}" for img in os.listdir("output/camera_calibration") if img.endswith(".png")]

    for img_path in img_paths:
        # Load the image
        img = cv2.imread(img_path)
        print(f"image shape: {img.shape}")

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # ret, corners = cv2.findChessboardCorners(img, CB_DIMS,
        #                                         flags=cv2.CALIB_CB_ADAPTIVE_THRESH +
        #                                             cv2.CALIB_CB_FAST_CHECK +
        #                                             cv2.CALIB_CB_NORMALIZE_IMAGE)
        
        ret, corners = cv2.findChessboardCornersSB(img, CB_DIMS, flags=cv2.CALIB_CB_EXHAUSTIVE)
        if ret:
            print("METHOD 1")
            fnl = cv2.drawChessboardCorners(img.copy(), CB_DIMS, corners, ret)

            corners = cv2.cornerSubPix(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), corners, (11, 11), (-1, -1), criteria)
            corners = corners.reshape(-1, 2)
            
            # Draw results on the image
            draw_on_image(fnl, "Method 1", corners)
        
        if not ret:
            print("No Checkerboard Found")
        else:
            cv2.waitKey(0)
        
        print()
        print("=" * 50)
        print()
        
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
