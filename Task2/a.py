import cv2
import numpy as np

# 1. Provide your Camera Calibration Data
# (These are fake placeholder values. You MUST replace these with your actual calibration results)
camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((4,1)) 

# 2. Define the actual physical size of your printed marker in meters
# Let's assume you printed the tag so it is exactly 10 cm (0.1 meters) wide.
marker_length = 0.1 

# Define the 3D coordinates of the 4 corners of the marker in its own local world.
# We assume the center of the marker is at (0,0,0).
obj_points = np.array([
    [-marker_length / 2,  marker_length / 2, 0], # Top-Left
    [ marker_length / 2,  marker_length / 2, 0], # Top-Right
    [ marker_length / 2, -marker_length / 2, 0], # Bottom-Right
    [-marker_length / 2, -marker_length / 2, 0]  # Bottom-Left
], dtype=np.float32)

# 3. Setup ArUco Detector (Using the 4x4 dictionary from your PDF!)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

# Load image or video frame
image = cv2.imread("img.png")
corners, ids, rejected = detector.detectMarkers(image)

if ids is not None:
    # 4. Loop through every detected marker
    for i in range(len(ids)):
        
        # solvePnP calculates the 6D pose (rotation vector and translation vector)
        success, rvec, tvec = cv2.solvePnP(
            obj_points,           # The 3D physical coordinates we defined above
            corners[i][0],        # The 2D pixel coordinates found in the image
            camera_matrix, 
            dist_coeffs
        )
        
        # tvec is your Translation (X, Y, Z distance from camera in meters)
        # rvec is your Rotation (Pitch, Yaw, Roll)
        
        print(f"Marker ID: {ids[i][0]}")
        print(f"Distance (Z): {tvec[2][0]:.2f} meters away")
        
        # 5. Draw the 3D XYZ axes directly onto the image so you can visualize the pose
        # Red = X, Green = Y, Blue = Z
        cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, marker_length / 2)

# Display the result
cv2.imshow("6D Pose Estimation", image)
cv2.waitKey(0)
cv2.destroyAllWindows()