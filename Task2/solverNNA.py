import math
import numpy as np

# Robot physical geometry
LINK_0 = 71.5
LINK_1 = 125
LINK_2 = 125
LINK_3 = 60 + 132

def move_to_position_cart(x, y, z):
    radius_multiplier = 1.02  # 2 percent scale
    z_adj = z + 15  
    
    horizontal_dist = math.sqrt(x**2 + y**2)
    radial_dist = math.sqrt(horizontal_dist**2 + (z_adj - LINK_0)**2) * radius_multiplier
    
    if y == 0:
        base_angle = 180 if x <= 0 else 0
    else:
        base_angle = 90 - math.degrees(math.atan(x / y))  
    
    # Kinematics calculations
    angle_alpha = math.acos((radial_dist - LINK_2) / (LINK_1 + LINK_3))
    shoulder_angle = math.degrees(angle_alpha)
    
    angle_beta = math.asin((math.sin(angle_alpha) * LINK_3 - math.sin(angle_alpha) * LINK_1) / LINK_2)  
    elbow_angle = (90 - math.degrees(angle_alpha)) + math.degrees(angle_beta)
    wrist_angle = (90 - math.degrees(angle_alpha)) - math.degrees(angle_beta)
    
    if wrist_angle <= 0: 
        angle_alpha = math.acos((radial_dist - LINK_2) / (LINK_1 + LINK_3))
        shoulder_angle = math.degrees(angle_alpha + math.asin((LINK_3 - LINK_1) / radial_dist))
        elbow_angle = (90 - math.degrees(angle_alpha))
        wrist_angle = (90 - math.degrees(angle_alpha))
    
    if z_adj != LINK_0:
        shoulder_angle += math.degrees(math.atan((z_adj - LINK_0) / radial_dist))
    
    # Hardware misalignment offsets
    elbow_angle += 5  
    wrist_angle += 5  
    
    return [round(base_angle), round(shoulder_angle), round(elbow_angle), round(wrist_angle)]

def get_previous_teta2():
    with open("prev_teta.txt", "r") as file:
        data = file.read().split(";")[:-1]
    return [int(i) for i in data]

def backlash_compensation_base(theta_base):
    theta_base = round(theta_base)
    compensated_base = theta_base
    
    CW_OFFSET = 8  
    CCW_OFFSETS = np.linspace(0, 14, 135)
    
    past_angles = get_previous_teta2()  
    prev_base = past_angles[0]
    
    angle_diff = theta_base - prev_base
    
    if angle_diff > 1:
        if theta_base <= 45:
            compensated_base = theta_base
        else:
            idx = int(round(theta_base - 46))
            compensated_base = round(theta_base + CCW_OFFSETS[idx])
            
    if angle_diff < -1:
        compensated_base = theta_base - CW_OFFSET

    return compensated_base