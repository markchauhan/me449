from modern_robotics import *
import numpy as np



def TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k):
    """
        Generates a trajectory for the end-effector to move from 
        
        Tse_Initial: Initail End-Effector Configuration
        Tsc_initial: Initial Cube Configuration
        Tsc_Final: Final Cube Configuration
        Tce_grap: End effector configuration at grasp
        Tce_standoff: End effector configuration at standoff
        k time resolution factor

        Output
            numpy array of trajectory with gripper state
    """
    # Define length of time segments
    length_time = [8, 3, 1, 3, 5, 3, 1, 3]


    #Find the number of trajectory points
    N = [int(times * k/ 0.01) for times in length_time]
    output = np.zeros((sum(N), 13))

    key_way = [
        Tsc_initial @ Tce_standoff,
        Tsc_initial @ Tce_grasp,
        Tsc_initial @ Tce_grasp, 
        Tsc_initial @ Tce_grasp, 
        Tsc_final @ Tce_standoff,
        Tsc_final @ Tce_grasp,
        Tsc_final @ Tce_grasp,
        Tsc_final @ Tce_standoff
    ]

    counter = 0
    current_frame = Tse_initial
    grip_state = 0
    for i, result_frame in enumerate(key_way):
        traj = CartesianTrajectory(current_frame, result_frame, length_time[i], N[i], 3)
        if i ==2 or i == 6:
            grip_state = 1- grip_state #When needing to reverse the cube it will do the opp
        for t in traj:
            rot, pos = TransToRp(t)
            row = np.append(np.append(np.reshape(rot, 9), pos), grip_state)
            output[counter] = row
            counter += 1
        
        current_frame = result_frame
    
    return output




































   
   
   
   
   
   
   
   
   
   
   
   
   
   
   









   



   
   






































