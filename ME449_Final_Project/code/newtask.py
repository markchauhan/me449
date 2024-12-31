# best.py
# Script to execute the best configuration.
from modern_robotics import *
import numpy as np
import os
import matplotlib.pyplot as plt
from milestone1 import *
from milestone2 import *
from milestone3 import *
from graph import *

resolution = 1
timestep = 0.01
sim_mode = "newtask"
path = "./results/newtask"
os.makedirs(path, exist_ok=True)

Tsei = np.array([
    [0, 0, 1, 0],
    [0, 1, 0, 0],
    [-1, 0, 0, 0.5],
    [0, 0, 0, 1]
])
#qi = (1.5,-.5,.025)
#qf = (0.2,-1.2,.025)
#new initial position of cube

x_i, y_i, z_i = 1.5, -0.5, 0.025
x_f, y_f, z_f = 0.2, -1.2, 0.025
theta_i = np.pi / 4
theta_f = - np.pi / 2

R_i = np.array([
    [np.cos(theta_i), -np.sin(theta_i), 0],
    [np.sin(theta_i), np.cos(theta_f), 0],
    [0, 0, 1]])
    
R_f = np.array([
    [np.cos(theta_f), -np.sin(theta_f), 0],
    [np.sin(theta_f), np.cos(theta_f), 0],
    [0, 0, 1]])
    
Tsci = np.array([
    [R_i[0, 0], R_i[0, 1], R_i[0, 2], x_i],
    [R_i[1, 0], R_i[1, 1], R_i[1, 2], y_i],
    [R_i[2, 0], R_i[2, 1], R_i[2, 2], z_i],
    [0, 0, 0, 1]
])

Tscf = np.array([
    [R_f[0, 0], R_f[0, 1], R_f[0, 2], x_f],
    [R_f[1, 0], R_f[1, 1], R_f[1, 2], y_f],
    [R_f[2, 0], R_f[2, 1], R_f[2, 2], z_f],
    [0, 0, 0, 1]
])

Kp = np.eye(6) * 0.9
Ki = np.eye(6) * .5
max_wheel = 10
max_joint = 10
k = 1
dt = 0.01
path = '/Users/markchauhan/Documents/ME449_FP/results/newtask'
sim_data('newtask',path, max_wheel, max_joint,Kp,Ki,Tsei, Tsci, Tscf, k, dt)
