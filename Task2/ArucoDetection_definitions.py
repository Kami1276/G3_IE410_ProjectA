import cv2
import numpy as np

def getMarkerCoordinates(detected_markers, ids, point_idx=0): 
    coords_list = []
    for m in detected_markers:
        coords_list.append([int(m[0][point_idx][0]), int(m[0][point_idx][1])])
    return coords_list, ids

def getMarkerCenter_foam(marker_data):
    pt_top_left, _ = getMarkerCoordinates(marker_data, 1, point_idx=0) 
    pt_top_right, _ = getMarkerCoordinates(marker_data, 1, point_idx=1)    
    pt_btm_left, _ = getMarkerCoordinates(marker_data, 1, point_idx=2) 
    pt_btm_right, _ = getMarkerCoordinates(marker_data, 1, point_idx=3) 
    
    if pt_top_left:
        c_x = (pt_top_left[0][0] + pt_top_right[0][0] + pt_btm_left[0][0] + pt_btm_right[0][0]) / 4.0
        c_y = (pt_top_left[0][1] + pt_top_right[0][1] + pt_btm_left[0][1] + pt_btm_right[0][1]) / 4.0
        return [[int(c_x), int(c_y)]]
    
    return [[0, 0]]

def draw_corners(image_frame, corner_pts):
    for pt in corner_pts:    
        cv2.circle(image_frame, (pt[0], pt[1]), 10, (0, 255, 0), thickness=-1)       

def draw_numbers(image_frame, corner_pts, ids_list):
    font_style = cv2.FONT_HERSHEY_SIMPLEX
    for idx, pt in enumerate(corner_pts):    
        cv2.putText(image_frame, str(ids_list[idx]), (pt[0] + 10, pt[1] + 10), font_style, 2, (0, 0, 0), 4)
        
def show_spec(image_frame, corner_pts):
    display_text = f"{len(corner_pts)} markers found."
    cv2.putText(image_frame, display_text, (15, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 250), 1)
        
def draw_field(image_frame, corner_pts, ids_list):   
    if len(corner_pts) == 4:
        ordered_markers = [0, 0, 0, 0] 
        for s_id in [1, 2, 3, 4]:
            idx = ids_list.index(s_id)
            ordered_markers[s_id - 1] = corner_pts[idx]
            
        poly_pts = np.array(ordered_markers)      
        overlay_img = image_frame.copy()
        cv2.fillPoly(overlay_img, pts=[poly_pts], color=(255, 215, 0))
        
        blended_img = cv2.addWeighted(overlay_img, 0.4, image_frame, 0.6, 0)
        return blended_img, True
        
    return image_frame, False

def order_points(pts_array):
    ordered_rect = np.zeros((4, 2), dtype="float32")
    pt_sums = pts_array.sum(axis=1)
    ordered_rect[0] = pts_array[np.argmin(pt_sums)]
    ordered_rect[2] = pts_array[np.argmax(pt_sums)]
    
    pt_diffs = np.diff(pts_array, axis=1)
    ordered_rect[1] = pts_array[np.argmin(pt_diffs)]
    ordered_rect[3] = pts_array[np.argmax(pt_diffs)]
    return ordered_rect

def four_point_transform(img_source, pts_array):
    rect_bounds = order_points(pts_array)
    (top_l, top_r, btm_r, btm_l) = rect_bounds
    
    w_top = np.sqrt(((top_r[0] - top_l[0]) ** 2) + ((top_r[1] - top_l[1]) ** 2))
    w_btm = np.sqrt(((btm_r[0] - btm_l[0]) ** 2) + ((btm_r[1] - btm_l[1]) ** 2))
    max_w = max(int(w_top), int(w_btm))
    
    h_left = np.sqrt(((top_l[0] - btm_l[0]) ** 2) + ((top_l[1] - btm_l[1]) ** 2))
    h_right = np.sqrt(((top_r[0] - btm_r[0]) ** 2) + ((top_r[1] - btm_r[1]) ** 2))
    max_h = max(int(h_left), int(h_right))
    
    destination_pts = np.array([
        [0, 0],
        [max_w - 1, 0],
        [max_w - 1, max_h - 1],
        [0, max_h - 1]], dtype="float32")
        
    transform_matrix = cv2.getPerspectiveTransform(rect_bounds, destination_pts)
    return cv2.warpPerspective(img_source, transform_matrix, (max_w, max_h))