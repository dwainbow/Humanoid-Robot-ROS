import numpy as np


def get_matrix(theta, d, a, alpha):
    matrix = np.array([
    [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
    [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
    [0, np.sin(alpha), np.cos(alpha), d],
    [0, 0, 0, 1]  
    ])
    
    return matrix

def get_yaw_pitch_roll(matrix):
    R =matrix[:3, :3]
    yaw = np.arctan2(R[1,0], R[0,0])
    pitch = np.arctan2(-R[2,0], np.sqrt(R[2,1]**2 + R[2,2]**2))
    roll = np.arctan2(R[2,1], R[2,2])
    
    return yaw, pitch, roll #in radians 


#this is an example for a leg with 4 joints

d_2 = 10 #arbitrary for now  
d_3 = 10
d_4 = 10
d_5 = 10

x_1 = 0 #arbitrary for now
x_2 = 10

#Joint 1 (0-1)
r_1 = d_2 
alpha_1 = (np.pi/2)
theta_1 = 0 
d_1 = 0 
#Joint 2 (1-2)
r_2 = d_3
alpha_2 = alpha_1 - (np.pi/2)
theta_2 = theta_1 + (np.pi/2)
d_2 = 0 
#Joint 3 (2-3)
r_3 = d_4
alpha_3 = alpha_2
theta_3 = theta_2
d_3 = x_1 
#Joint 4 (3-4)
r_4 = d_5
alpha_4 = alpha_3
theta_4 = theta_3
d_4 = x_2 

T_01 = get_matrix(theta_1, d_1, r_1, alpha_1)
T_12 = get_matrix(theta_2, d_2, r_2, alpha_2)
T_23 = get_matrix(theta_3, d_3, r_3, alpha_3)
T_34 = get_matrix(theta_4, d_4, r_4, alpha_4)

#Solve for Joint (0-4)
T_04 = T_01 * T_12 * T_23 * T_34

x,y,z = T_04[:3,3] #this is the position of the end effector
yaw, pitch, roll = get_yaw_pitch_roll(T_04) #this is the orientation of the end effector
print(f"X: {x}; Y: {y}; Z: {z}")
print(f"Yaw: {yaw}; Pitch: {pitch}; Roll: {roll}")

