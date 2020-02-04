#!/usr/bin/env python3

"""
    These are all functions that support computing the Legrangian Dynamics.
"""

import time
import sympy as sym
from sympy.abc import t
import scipy.linalg

def se3ToVec(v_se3):
    """
    Used to convert an se3 twist to a 6-Vector twist.

    Input:

        v_se3: (sympy Matrix) a twist represented in se3

    Output:

        v_six: (sympy Matrix) a twist represented as a 6-element column vector
    """

    v_six = sym.Matrix([v_se3[0, 3],
                        v_se3[1, 3],
                        v_se3[2, 3],
                        v_se3[3, 1],
                        v_se3[0, 2],
                        v_se3[1, 0]])

    return v_six

def TransInv(g):
    """
    Inverts a Transformation Matrix.

    Input:

        g: (sympy Matrix) a transformation matrix

    Output:

        g_inv: (sympy Matrix) the inverse transformation of g
    """
    g_inv = sym.zeros(4, 4)

  # extract the rotation matrix
    R = sym.zeros(3, 3)
    R[0, 0] = g[0, 0]
    R[0, 1] = g[0, 1]
    R[0, 2] = g[0, 2]
    R[1, 0] = g[1, 0]
    R[1, 1] = g[1, 1]
    R[1, 2] = g[1, 2]
    R[2, 0] = g[2, 0]
    R[2, 1] = g[2, 1]
    R[2, 2] = g[2, 2]

  # extract the translation vector
    x = g[0, 3]
    y = g[1, 3]
    z = g[2, 3]
    p = sym.Matrix([x, y, z])

  # transpose R
    R_trans = R.T
    g_inv[0, 0] = R_trans[0, 0]
    g_inv[0, 1] = R_trans[0, 1]
    g_inv[0, 2] = R_trans[0, 2]
    g_inv[1, 0] = R_trans[1, 0]
    g_inv[1, 1] = R_trans[1, 1]
    g_inv[1, 2] = R_trans[1, 2]
    g_inv[2, 0] = R_trans[2, 0]
    g_inv[2, 1] = R_trans[2, 1]
    g_inv[2, 2] = R_trans[2, 2]
    R_var = -R.T * p
    g_inv[0, 3] = R_var[0]
    g_inv[1, 3] = R_var[1]
    g_inv[2, 3] = R_var[2]
    g_inv[3, 3] = 1

    return g_inv

def integrate(f, x0, dt):
    """
    This function uses the RK4 integration method to take an initial condition
    and a timestep, as well as a dynamical system f(x) that outputs a vector
    of the new system state.

    Input:
        f: (function) set the array to integrate
        x0: (array) the current state of the system
        dt: (double) the timestep to integrate across

    Output:
        xnew: (array) The position & velocity of after the timestep

    Code Provided by Tommy Burretta.
    """
    k1=dt*f(x0)
    k2=dt*f(x0+k1/2.)
    k3=dt*f(x0+k2/2.)
    k4=dt*f(x0+k3)
    xnew=x0+(1/6.)*(k1+2.*k2+2.*k3+k4)
    return xnew
