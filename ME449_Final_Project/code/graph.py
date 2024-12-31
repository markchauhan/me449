import matplotlib.pyplot as plt
import os
import numpy as np

from milestone1 import *
from milestone2 import *
from milestone3 import *

#plot error

#13-element trajectory to matrix
def transformation_matrix(row):
    return np.array([
        [row[0], row[1], row[2], row[9]],
        [row[3], row[4], row[5], row[10]],
        [row[6], row[7], row[8], row[11]],
        [0, 0, 0, 1]
    ])   

def ErrorPlot(X_err, mode, directory_path):
    """
    Generates and saves plots for angular and Cartesian errors 
    
    """
    time = np.arange(0, len(X_err) * 0.01, 0.01)  # Time vector assuming dt = 0.01
    
    # Plot Angular Errors 
    fig1 = plt.figure(figsize=(8, 6))
    for i, label in enumerate(['$\\theta_x$', '$\\theta_y$', '$\\theta_z$']):
        plt.plot(time, X_err[:, i], label=label)
    plt.title('Angular Error vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Error (rad)')
    plt.legend()
    plt.grid()
    plt.savefig(os.path.join(directory_path, f'{mode}_angular_error_plot.png'))
    plt.show()

    # Plot Cartesian Errors 
    fig2 = plt.figure(figsize=(8, 6))
    for i, label in enumerate(['$x$', '$y$', '$z$']):
        plt.plot(time, X_err[:, i + 3], label=label)
    plt.title('Cartesian Error vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Cartesian Error (m)')
    plt.legend()
    plt.grid()
    plt.savefig(os.path.join(directory_path, f'{mode}_cartesian_error_plot.png'))
    plt.show()

def sim_data(sim_mode, output_path, max_wheel, max_joint, Kp, Ki, Tsei, Tsci, Tscf, k, dt):
   """Simulation of robot trajectorky
        sim_mode type of simulation
        output_path path to save the results
        max_wheel maximum wheel velocity
        max_join maximum joint velocity
        Kp proprotional gain matrix
        Ki proportion
        """
   
         #Robot geometry
   Blist = np.array([
           [0, 0, 1, 0, 0.033, 0],
           [0, -1, 0, -0.5076, 0, 0],
           [0, -1, 0, -0.3526, 0, 0],
            [0, -1, 0, -0.2176, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ]).T    
   Mlist = np.array([
           [1, 0, 0, 0.033],
           [0, 1, 0, 0],
           [0, 0, 1, 0.6546],
           [0, 0, 0, 1]
       ])  
   Tb0 = np.array([
           [1, 0, 0, 0.1662],
           [0, 1, 0, 0],
           [0, 0, 1, 0.0026],
           [0, 0, 0, 1]
       ])
   x = np.array([np.pi/4, -0.5, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

   align_axis = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])

    #angle of gripper
   theta = np.pi/4 - np.pi/16 
   align_axis_2 = np.array([[np.cos(theta), 0, np.sin(theta)], [0,1,0], [-np.sin(theta), 0, np.cos(theta)]])
   Tce_standoff = RpToTrans(align_axis @ align_axis_2, np.array([0.01, 0, 0.2]))
   Tce_grasp = RpToTrans(align_axis @ align_axis_2, np.array([0.01, 0, -0.005]))
   
   


   #Trajectory
   print(f"generating trajectory for {sim_mode} config")
   traj = TrajectoryGenerator(Tsei, Tsci, Tscf, Tce_grasp, Tce_standoff, k)

   N = traj.shape[0]-1
   output = np.zeros((int(round(N / k)), 13))
   X_error = np.zeros((int(round(N / k)), 6))
   X_error_integral = np.zeros(6) 
   
   for step in range(N):
    #Forward Kinematics
    T0e = FKinBody(Mlist, Blist, x[3:8])
    Tsb = np.array([
        [np.cos(x[0]), -np.sin(x[0]), 0, x[1]],
            [np.sin(x[0]), np.cos(x[0]), 0, x[2]],
            [0, 0, 1, 0.0963],
            [0, 0, 0, 1]
         ])

     # Compute current and future configurations
    current_pose = Tsb @ Tb0 @ T0e
    goal_pose = transformation_matrix(traj[step])
    next_pose = transformation_matrix(traj[step+1])
    # Debugging: Check the transformation matrices
    calc_vel, error = FeedbackControl(
             current_pose, goal_pose, next_pose, Kp, Ki, dt, X_error_integral
         )

    X_error_integral += error * dt / k

     #Jacbians and the velocity
    J_chassis = Adjoint(TransInv(T0e) @ TransInv(Tb0)) @ F6
    J_arm = JacobianBody(Blist, x[3:8])
    J = np.hstack((J_chassis, J_arm))
    xdot = np.linalg.pinv(J) @ calc_vel
    
    
    
     #Update the state
    x = np.append(NextState(x, xdot, dt/k, max_wheel, max_joint), traj[step][12])
    
    if step % k == 0:
     output[int(step / k)] = x
     X_error[int(step / k)] = error

    # Results
   np.savetxt(os.path.join(output_path, f'{sim_mode}_output.csv'), output, delimiter=',')
   np.savetxt(os.path.join(output_path, f'{sim_mode}_error.csv'), X_error, delimiter=',')
   
    #Plot
   print(f"plotting trajectory error for {sim_mode} config")
   ErrorPlot(X_error, sim_mode, output_path)
   print(f"Produced {sim_mode} output")


   
    
