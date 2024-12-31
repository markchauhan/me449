import numpy as np
from modern_robotics import MatrixExp6, VecTose3, TransToRp, so3ToVec

# Chassis Configuration and Geometry
l = 0.47 / 2
w = 0.3 / 2
r = 0.0475
F = r / 4 * np.array([[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
                      [1, 1, 1, 1],
                      [-1, 1, -1, 1]])
F6 = np.vstack((np.zeros(4, ), np.zeros(4, ), F, np.zeros(4, )))

def clamp_controls(u, max_wheel, max_joint):
    u = np.array(u)
    
    # Clamp joint controls (indices 4 to 8)
    u[4:9] = np.clip(u[4:9], -max_joint, max_joint)
    
    # Clamp wheel controls (indices 0 to 3)
    u[0:4] = np.clip(u[0:4], -max_wheel, max_wheel)
    return u

def NextState(config, control, timestep, max_wheel_speed, max_joint_speed):
    """
    Simulates the next configuration of the robot using first-order Euler Integration.
    """
    global F
    
    control = clamp_controls(control, max_wheel_speed, max_joint_speed)

    # Update joint and wheel angles
    new_wheel_angles = np.array(config[8:12]) + np.array(control[0:4]) * timestep
    new_joint_angles = np.array(config[3:8]) + np.array(control[4:9]) * timestep

    # Body Twist
    Vb = F @ control[0:4].T
    Vb = np.array([0, 0, *Vb, 0])

    Tsb = np.array([[np.cos(config[0]), -np.sin(config[0]), 0, config[1]], 
                    [np.sin(config[0]), np.cos(config[0]), 0, config[2]], 
                    [0, 0, 1, 0.0963], 
                    [0, 0, 0, 1]])
    
    #frame
    current_pose = MatrixExp6(VecTose3(Vb) * timestep)
    next_pose = Tsb @ current_pose
    [Rot, pos] = TransToRp(next_pose)
    phiVec = so3ToVec(Rot)

    new_config = np.array([np.arcsin(phiVec[2]), pos[0], pos[1], *new_joint_angles, *new_wheel_angles])
    return new_config






