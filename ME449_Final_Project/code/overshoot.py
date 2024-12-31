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
sim_mode = "overshoot"
path = "./results/overshoot"
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

Kp = np.eye(6) * 0.9
Ki = np.eye(6) * .5
max_wheel = 10
max_joint = 10
k = 1
dt = 0.01
path = '/Users/markchauhan/Documents/ME449_FP/results/overshoot'
sim_data('overshoot',path, max_wheel, max_joint,Kp,Ki,Tsei, Tsci, Tscf, k, dt)
