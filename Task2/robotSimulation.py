import cv2
import numpy as np
import math

# ==========================================
# 1. ROBOT & CAMERA CONFIGURATION
# ==========================================

# --- Placeholder Camera Matrix ---
# real calibration data for accurate distance in meters!
camera_matrix = np.array([
    [1165.0, 0, 960.0], 
    [0, 1165.0, 540.0], 
    [0, 0, 1]
] 
dtype=np.float32)

# Physical size of your printed markers in meters (e.g., 0.05 = 5cm)
marker_length = 0.05 

# 3D points of the marker for pose estimation
obj_points = np.array([
    [-marker_length / 2,  marker_length / 2, 0],
    [ marker_length / 2,  marker_length / 2, 0],
    [ marker_length / 2, -marker_length / 2, 0],
    [-marker_length / 2, -marker_length / 2, 0]
], dtype=np.float32)

# Specific IDs from your PDF
BASE_MARKER_ID = 1    # The marker the Ghost Arm will stand on
TARGET_MARKER_ID = 2  # The object you want to pick up

# --- Braccio Arm Lengths (in meters) ---
L1 = 0.07  # Base height
L2 = 0.125 # Shoulder to Elbow
L3 = 0.125 # Elbow to Vertical Wrist
L4 = 0.06  # Wrist to Gripper tip

# ==========================================
# 2. FORWARD KINEMATICS (GHOST ARM MATH)
# ==========================================
def calculate_fk_3d_points(m1, m2, m3, m4):
    """Calculates the 3D (X, Y, Z) coordinates of each joint."""
    t1 = math.radians(m1)
    t2 = math.radians(m2)
    t3 = math.radians(m3)
    t4 = math.radians(m4)

    p0 = np.array([0, 0, 0])                                      # Origin
    p1 = np.array([0, 0, L1])                                     # Base Top
    p2 = p1 + np.array([                                          # Elbow
        L2 * math.cos(t2) * math.cos(t1),
        L2 * math.cos(t2) * math.sin(t1),
        L2 * math.sin(t2)
    ])
    p3 = p2 + np.array([                                          # Wrist
        L3 * math.cos(t2 + t3) * math.cos(t1),
        L3 * math.cos(t2 + t3) * math.sin(t1),
        L3 * math.sin(t2 + t3)
    ])
    p4 = p3 + np.array([                                          # Gripper Tip
        L4 * math.cos(t2 + t3 + t4) * math.cos(t1),
        L4 * math.cos(t2 + t3 + t4) * math.sin(t1),
        L4 * math.sin(t2 + t3 + t4)
    ])
    
    return np.array([p0, p1, p2, p3, p4], dtype=np.float32)

# ==========================================
# 3. ARUCO & WEBCAM SETUP
# ==========================================
# Load the 4x4 dictionary that matches your printed tags
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

# Initialize Webcam
cap = cv2.VideoCapture(0)

# ==========================================
# 4. MAIN LIVE LOOP
# ==========================================
while True:
    ret, image = cap.read()
    if not ret:
        print("Failed to grab frame. Check your webcam!")
        break

    # Detect markers
    corners, ids, rejected = detector.detectMarkers(image)

    # Variables to hold translation vectors for distance calculation
    tvec_base = None
    tvec_target = None
    center_base = None
    center_target = None

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        
        for i in range(len(ids)):
            marker_id = ids[i][0]
            success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], camera_matrix, dist_coeffs)
            
            # --- PROCESS BASE MARKER (ID 1) ---
            if marker_id == BASE_MARKER_ID:
                tvec_base = tvec
                # Get 2D center point for drawing the line later
                center_base = np.mean(corners[i][0], axis=0).astype(int)
                
                # Draw standard axes
                cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, marker_length)
                
                # Render the Ghost Arm! (Using safety position angles)
                m1, m2, m3, m4 = 90, 45, 180, 180 
                joint_points_3d = calculate_fk_3d_points(m1, m2, m3, m4)
                
                # Project 3D arm points onto the 2D image anchored to the marker
                joint_points_2d, _ = cv2.projectPoints(joint_points_3d, rvec, tvec, camera_matrix, dist_coeffs)
                points = np.int32(joint_points_2d).reshape(-1, 2)
                
                # Draw the wireframe bones and joints
                cv2.polylines(image, [points], isClosed=False, color=(0, 255, 255), thickness=4)
                for pt in points:
                    cv2.circle(image, tuple(pt), 6, (0, 0, 255), -1)
                    
            # --- PROCESS TARGET MARKER (ID 2) ---
            elif marker_id == TARGET_MARKER_ID:
                tvec_target = tvec
                center_target = np.mean(corners[i][0], axis=0).astype(int)
                cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, marker_length)

        # --- CALCULATE DISTANCE IF BOTH ARE VISIBLE ---
        if tvec_base is not None and tvec_target is not None:
            # Calculate 3D Euclidean distance
            distance = np.linalg.norm(tvec_base - tvec_target)
            
            # Display distance text
            text = f"Target Distance: {distance:.3f} m"
            cv2.putText(image, text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
            
            # Draw a targeting line between the two markers
            cv2.line(image, tuple(center_base), tuple(center_target), (255, 0, 0), 2)

    # Show the final AR feed
    cv2.imshow("Braccio AR Simulation", image)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
cap.release()
cv2.destroyAllWindows()