import cv2
import numpy as np

# --- Placeholder Camera Matrix (Replace with your calibration data!) ---
camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((4,1)) 

# Physical size of the marker in meters (e.g., 0.05 for 5cm)
marker_length = 0.05 

obj_points = np.array([
    [-marker_length / 2,  marker_length / 2, 0],
    [ marker_length / 2,  marker_length / 2, 0],
    [ marker_length / 2, -marker_length / 2, 0],
    [-marker_length / 2, -marker_length / 2, 0]
], dtype=np.float32)

# Setup ArUco (Assuming the 4x4 dictionary from your PDF)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

# The specific IDs of the two markers you printed
MARKER_1_ID = 2
MARKER_2_ID = 5

cap = cv2.VideoCapture(0)

while True:
    ret, image = cap.read()
    if not ret:
        break

    corners, ids, rejected = detector.detectMarkers(image)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        
        tvec_1 = None
        tvec_2 = None
        
        # Loop through all detected markers and calculate their pose
        for i in range(len(ids)):
            success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], camera_matrix, dist_coeffs)
            
            # If the ID matches our first target, save its translation vector
            if ids[i][0] == MARKER_1_ID:
                tvec_1 = tvec
                cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, marker_length)
                
            # If the ID matches our second target, save its translation vector
            elif ids[i][0] == MARKER_2_ID:
                tvec_2 = tvec
                cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, marker_length)

        # If both markers are visible on screen at the same time, calculate the distance!
        if tvec_1 is not None and tvec_2 is not None:
            # np.linalg.norm calculates the Euclidean distance perfectly
            distance = np.linalg.norm(tvec_1 - tvec_2)
            
            # Display the distance on the screen
            text = f"Distance: {distance:.3f} meters"
            cv2.putText(image, text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Optional: Draw a line between the two markers in 2D space for visual flair
            # We get the center of each marker by averaging its 4 corners
            center_1 = np.mean(corners[np.where(ids == MARKER_1_ID)[0][0]][0], axis=0).astype(int)
            center_2 = np.mean(corners[np.where(ids == MARKER_2_ID)[0][0]][0], axis=0).astype(int)
            cv2.line(image, tuple(center_1), tuple(center_2), (255, 0, 0), 2)

    cv2.imshow("ArUco 3D Distance", image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()