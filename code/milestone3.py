from modern_robotics import *
import numpy as np

def FeedbackControl(X, Xd, Xdnext, Kp, Ki, dt, X_err_int):
    """
    Computes the end-effector twist through feedback control
    
    Inputs:
    - X: Current End-Effector Configuration
    - Xd: Current Reference end-effector End-Effector Configuration
    - Xdnext: Next Reference 
    - Kp: Proportional Gain
    - Ki: Integral Gain
    - dt: time step
    - X_err_int: error
    
    Returns:
    - V: Commanded end-effector twist 
    - X_err: Current end-effector error twist 
    - X_err_int: Updated integral error vector
    """

    Vd = se3ToVec(1 /dt * MatrixLog6(TransInv(Xd) @ Xdnext))
    Xerr = se3ToVec(MatrixLog6(TransInv(X) @ Xd))
    V = Adjoint(TransInv(X) @ Xd) @ Vd + Kp @ Xerr + Ki @ (X_err_int + Xerr * dt)
    return V, Xerr











