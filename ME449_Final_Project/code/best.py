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
sim_mode = "best"
path = "./results/best"
os.makedirs(path, exist_ok=True)

Tsei = np.array([
    [0, 0, 1, 0],
    [0, 1, 0, 0],
    [-1, 0, 0, 0.5],
    [0, 0, 0, 1]
])

Tsci = np.array([
    [1, 0, 0, 1],
    [0, 1, 0, 0],
    [0, 0, 1, 0.025],
    [0, 0, 0, 1]
])

Tscf = np.array([
    [0, 1, 0, 0],
    [-1, 0, 0, -1],
    [0, 0, 1, 0.025],
    [0, 0, 0, 1]
])

Kp = np.eye(6) * 0.4
Ki = np.eye(6) * 0.0
max_wheel = 10
max_joint = 10
k = 1
dt = 0.01
file_dir = '/Users/markchauhan/Documents/ME449_FP/results/best'
sim_data('best',file_dir, max_wheel, max_joint,Kp,Ki,Tsei, Tsci, Tscf, k, dt)
