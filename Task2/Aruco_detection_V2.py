#!/usr/bin/env python

import time
import cv2 
import numpy as np 
from ArucoDetection_definitions import *
import braccio_control_python 
import keyboard

# System and camera configurations
PRIMARY_DICT = "DICT_4X4_50"
SECONDARY_DICT = "DICT_6X6_50"

# Camera Intrinsic Matrix and Distortion Coefficients
# Maps 2D pixel coordinates to 3D spatial coordinates
CAMERA_MATRIX = np.array([
    [1165.0, 0, 960.0], 
    [0, 1165.0, 540.0], 
    [0, 0, 1]
], dtype=np.float32)

DIST_COEFFS = np.zeros((4, 1), dtype=np.float32)

# Physical size of the target ArUco marker in millimeters
TARGET_MARKER_SIZE_MM = 50.0 

AVAILABLE_DICTS = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
}

def detect_markers_in_frame(video_frame, dict_type, params):
    """
    Scans the provided frame for ArUco markers based on the specified dictionary.
    Returns the bounding box coordinates and the corresponding marker IDs.
    """
    bounding_boxes, marker_ids, _ = cv2.aruco.detectMarkers(video_frame, dict_type, parameters=params)
    sorted_ids = [m_id[0] for m_id in marker_ids] if marker_ids is not None else None
    return bounding_boxes, sorted_ids

# Initial boundary points for the workspace projection
init_locs = [[10, 400], [400, 400], [400, 10], [10, 10]]
active_boundary_pts = list(init_locs)
active_center_pt = [[0, 0]]

hold_marker_position = True

def run_vision_system():
    """
    Main vision loop handling camera feed, workspace boundary detection, 
    target object pose estimation, and robotic pick-and-place triggering.
    """
    system_start_time = time.time()
    
    dict_primary = cv2.aruco.getPredefinedDictionary(AVAILABLE_DICTS[PRIMARY_DICT])
    params_primary = cv2.aruco.DetectorParameters()

    dict_secondary = cv2.aruco.getPredefinedDictionary(AVAILABLE_DICTS[SECONDARY_DICT])
    params_secondary = cv2.aruco.DetectorParameters()
    
    cam_url = 'http://192.168.0.17:8080/videofeed'
    video_stream = cv2.VideoCapture(0)
    
    boundary_pts = active_boundary_pts

    while True:
        now = time.time()
        loop_delay = 0 

        success, current_frame = video_stream.read()  
        if not success:
            continue
            
        detected_markers, d_ids = detect_markers_in_frame(current_frame, dict_primary, params_primary)
        pristine_frame = current_frame.copy()

        top_left_corners, extracted_ids = getMarkerCoordinates(detected_markers, d_ids, 0)

        # Retain the last known marker position if the marker is temporarily obscured
        if hold_marker_position:
            if extracted_ids is not None:
                for idx, m_id in enumerate(extracted_ids):
                    if m_id > 4: break  
                    active_boundary_pts[m_id - 1] = top_left_corners[idx]
            
            top_left_corners = active_boundary_pts            
            extracted_ids = [1, 2, 3, 4]      

        # Render visualizations on the main camera feed
        if (system_start_time + loop_delay * 1) < now and (system_start_time + loop_delay * 2) > now:   
            cv2.aruco.drawDetectedMarkers(current_frame, detected_markers) 
        if (system_start_time + loop_delay * 2) < now:    
            draw_corners(current_frame, top_left_corners)
        if (system_start_time + loop_delay * 3) < now:
            draw_numbers(current_frame, top_left_corners, extracted_ids)
        if (system_start_time + loop_delay * 4) < now:    
            show_spec(current_frame, top_left_corners)
       
        frame_with_overlay, is_square_found = draw_field(current_frame, top_left_corners, extracted_ids)
        
        # Isolate the workspace area and detect the target object (foam)
        if (system_start_time + loop_delay * 6) < now:
            if is_square_found:
                boundary_pts = top_left_corners
                
            warped_img = four_point_transform(pristine_frame, np.array(boundary_pts))
            h, w, _ = warped_img.shape
            
            foam_markers, foam_ids = detect_markers_in_frame(warped_img, dict_secondary, params_secondary)
            foam_corner, foam_c_id = getMarkerCoordinates(foam_markers, foam_ids, 0)
            center_pt = getMarkerCenter_foam(foam_markers)
           
            if hold_marker_position and foam_c_id is not None:
                active_center_pt[0] = center_pt[0]
                
            center_pt[0] = active_center_pt[0]              
            
            # Estimate the 3D pose of the target object using the intrinsic camera matrix
            if foam_ids is not None and len(foam_markers) > 0:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    foam_markers, 
                    TARGET_MARKER_SIZE_MM, 
                    CAMERA_MATRIX, 
                    DIST_COEFFS
                )
                
                # Render the 3D spatial axes on the target object to visualize orientation
                for i in range(len(foam_ids)):
                    cv2.drawFrameAxes(warped_img, CAMERA_MATRIX, DIST_COEFFS, rvecs[i], tvecs[i], TARGET_MARKER_SIZE_MM / 2)

            draw_corners(warped_img, center_pt)
            cv2.line(warped_img, (center_pt[0][0], 0), (center_pt[0][0], h), (0, 0, 255), 2)
            cv2.line(warped_img, (0, center_pt[0][1]), (w, center_pt[0][1]), (0, 0, 255), 2)

            draw_numbers(warped_img, foam_corner, foam_c_id)
            cv2.imshow('Workspace View (Warped)', warped_img)

        cv2.imshow('Main Camera Feed', frame_with_overlay)
        
        # Halt sequence and park robot
        if cv2.waitKey(1) & 0xFF == ord('q'):
            braccio_control_python.arm.write(b'P,90,45,180,180,90,73,20\n')
            break
            
        # Execute the automated pick-and-place sequence
        if keyboard.is_pressed('p'):
            cam_x, cam_y = center_pt[0][0], center_pt[0][1]
            img_h, img_w, _ = warped_img.shape

            # Physical workspace dimensions in millimeters
            REAL_W, REAL_H = 750, 450
            
            # Translate image coordinates to physical workspace coordinates
            x_mm = (cam_x / img_w) * REAL_W
            y_mm = (cam_y / img_h) * REAL_H

            # Align coordinates with the robotic arm's base origin
            robot_x = x_mm - (REAL_W / 2)
            robot_y = y_mm

            print(f"Computed Workspace Coordinates: X:{robot_x:.2f}, Y:{robot_y:.2f}")
            
            # Apply geometric compensation for camera offset and perspective
            final_x, final_y = braccio_control_python.camera_compensation(robot_x, robot_y)
            print(f"Calibrated Robot Coordinates: X:{final_x}, Y:{final_y}")

            # Transmit structured G-code equivalent string to Arduino
            serial_cmd = f"G:{final_x},{final_y},20,200,0,20\n"
            braccio_control_python.arm.write(serial_cmd.encode())
            print("Motion command transmitted. Initiating pick and place.")

    video_stream.release()
    cv2.destroyAllWindows()
    return center_pt 

if __name__ == '__main__':
    braccio_control_python.home_robot()
    target_center = run_vision_system()