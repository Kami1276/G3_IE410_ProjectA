import serial
import time
import solverNNA
import numpy as np

# Updated configurations for joint limits [min, max, default, index]
JOINT_LIMITS = {
    'base': [0, 0, 180, 0],
    'shoulder': [150, 15, 165, 1],
    'elbow': [0, 0, 180, 2],
    'wrist': [0, 0, 180, 3],
    'wristRot': [90, 0, 180, 4],
    'gripper': [73, 73, 0, 5]
}

arm = serial.Serial("COM8", 115200, timeout=5)
print("[SYS] Establishing connection with arm...") 
time.sleep(2)
arm.write(b'H0,90,20,90,90,73,20\n')  
time.sleep(2)

def transmit_to_arduino(angle_array):
    angle_array[0] = 180 - angle_array[0]  
    angle_array[3] = 180 - angle_array[3]  
    
    # Using modern f-strings and joining
    data_payload = ','.join(map(str, angle_array))
    command = f"P{data_payload},200\n"
    
    print(f"Transmitting -> {command.strip()}")
    arm.write(command.encode())          

def home_robot(speed=20):
    default_pose = [
        JOINT_LIMITS['base'][0], JOINT_LIMITS['shoulder'][0],
        JOINT_LIMITS['elbow'][0], JOINT_LIMITS['wrist'][0],
        JOINT_LIMITS['wristRot'][0], JOINT_LIMITS['gripper'][0]
    ]
    transmit_to_arduino(default_pose)

def set_arm_pose(t_base=JOINT_LIMITS['base'][0], t_shoulder=JOINT_LIMITS['shoulder'][0],
                 t_elbow=JOINT_LIMITS['elbow'][0], t_wrist=JOINT_LIMITS['wrist'][0],
                 t_wRot=JOINT_LIMITS['wristRot'][0], state="closed"):
    
    t_gripper = JOINT_LIMITS['gripper'][1] if state == "closed" else JOINT_LIMITS['gripper'][2]
    comp_base = solverNNA.backlash_compensation_base(t_base)  
        
    pose_array = [comp_base, t_shoulder, t_elbow, t_wrist, t_wRot, t_gripper]
    transmit_to_arduino(pose_array)
    
    # Save uncompensated values
    raw_angles = [t_base, t_shoulder, t_elbow, t_wrist, t_wRot, t_gripper]
    with open("prev_teta.txt", "w") as f:
        f.write(";".join(map(str, raw_angles)) + ";")
    
def move_to_xyz(x, y, z, claw_state="closed"):
    calculated_thetas = solverNNA.move_to_position_cart(x, y, z)
    set_arm_pose(*calculated_thetas, state=claw_state)
        
def read_previous_angles():
    with open("prev_teta.txt", "r") as f:
        data = f.read().split(";")[:-1] 
    return [int(val) for val in data]

def set_gripper_state(state):
    past_angles = read_previous_angles()
    set_arm_pose(*past_angles[:5], state=state)

def camera_compensation(raw_x, raw_y):
    OBJ_HEIGHT = 80  
    CAM_POS = [480, 150, 880]  
    OFFSET_VAL = 300
    
    adj_x = (OFFSET_VAL - raw_x) + (CAM_POS[0] - OFFSET_VAL)
    
    comp_x = adj_x - (OBJ_HEIGHT / (CAM_POS[2] / adj_x))
    
    if raw_y < CAM_POS[1]:
        comp_y = raw_y - (OBJ_HEIGHT / (CAM_POS[2] / raw_y))
    else:
        comp_y = raw_y + (OBJ_HEIGHT / (CAM_POS[2] / raw_y))
        
    final_x = OFFSET_VAL - (comp_x - (CAM_POS[0] - OFFSET_VAL))
    return int(final_x), int(comp_y)