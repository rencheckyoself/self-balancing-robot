""" This file is used to compute the equations of motion for a self balancing robot """

import time
import numpy as np
import sympy as sym
from sympy.abc import t
import matplotlib.pyplot as plt
import scipy.linalg


def se3ToVec(v_se3):
    """
    Used to convert an se3 twist to a 6-Vector
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
    Inverts a Transformation Matrix
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


def main():
    """
    main function to perform the calculation
    """

    ###########################
    # Rigid Body Properties
    ###########################

    # Robot Wheel Properties
    wheel_radius = 1
    m_wheel = 1

    # Thin disk approximation
    I_wheel = (m_wheel * .5 * wheel_radius**2)

    # Create the Spatial Intertia Matrix
    # Need to fill in the Ixx and Iyy positions!!
    G_wheel = sym.Matrix(np.diagflat([m_wheel, m_wheel, m_wheel, 1, 1, I_wheel]))

    # Robot Body Properties
    # The body is represented by a rectangle
    length = 3
    width = 1
    depth = 4
    m_body = 4

    # Thin plate approximation for inertia
    I_body = (1/12)*m_body*(4*(length**2) + width**2)

    # Need to fill in the Ixx and Iyy positions!!
    G_body = sym.Matrix(np.diagflat([m_body, m_body, m_body, 1, 1, I_body]))

    ###########################
    # Config Variable Set Up

    # Set Symbols
    g = 9.81
    lam1, lam2 = sym.symbols('l_1 l_2')

    # Set Functions
    x_b = sym.Function('xb')(t)
    y_b = sym.Function('yb')(t)
    th_b = sym.Function('thb')(t)
    p_r = sym.Function('pr')(t)
    p_l = sym.Function('pl')(t)
    th_t = sym.Function('tht')(t)

    # Set Config Matrix
    q = sym.Matrix([x_b, y_b, th_b, p_r, p_l])
    qd = q.diff(t)
    qdd = qd.diff(t)

    # Set Dummy Vars
    Xb, Xbd, Xbdd = sym.symbols('x_b x_bd x_bdd')
    Yb, Ybd, Ybdd = sym.symbols('y_b y_bd y_bdd')
    Tb, Tbd, Tbdd = sym.symbols('t_b t_bd t_bdd')
    Pr, Prd, Prdd = sym.symbols('p_r p_rd p_rdd')
    Pl, Pld, Pldd = sym.symbols('p_l p_ld p_ldd')
    Tt, Ttd, Ttdd = sym.symbols('t_t t_td t_tdd')

    subber_q = {q[0]:Xb, qd[0]:Xbd, qdd[0]:Xbdd,
                q[1]:Yb, qd[1]:Ybd, qdd[1]:Ybdd,
                q[2]:Tb, qd[2]:Tbd, qdd[2]:Tbdd,
                q[3]:Pr, qd[3]:Prd, qdd[3]:Prdd,
                q[4]:Pl, qd[4]:Pld, qdd[4]:Pldd,
                q[5]:Tt, qd[5]:Ttd, qdd[5]:Ttdd}

    ###########################
    # Create External Forces
    #
    # Proportional Controled Torque applied to the angle of the wheel
    set_pt = sym.pi/2
    kp = 0
    F_tha = kp*(set_pt - th_b)

    F_mat = sym.Matrix([[0],
                        [F_tha],
                        [0]])

    #################################
    #### Create Frame Transforms ####
      # Frame W - World frame at (0,0,0)
      # Frame b - base link of the robot. At the mid point between the two wheel axels
      # Frame r - at the center of the right wheel
      # Frame l - at the center of the left wheel
      # Frame t - at the center of mass of the robot body.

    # Translation Transformations ############################

    xt = (length/2) * sym.cos(th_t)
    yt = 0
    zt = (length/2) * sym.sin(th_t)

    g_transb = sym.Matrix([[1, 0, 0, x_b],
                           [0, 1, 0, y_b],
                           [0, 0, 1, wheel_radius],
                           [0, 0, 0, 1]])


    g_transr = sym.Matrix([[1, 0, 0, 0],
                           [0, 1, 0, -length/2],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

    g_transl = sym.Matrix([[1, 0, 0, 0],
                           [0, 1, 0, length/2],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

    g_transt = sym.Matrix([[1, 0, 0, xt],
                           [0, 1, 0, yt],
                           [0, 0, 1, zt],
                           [0, 0, 0, 1]])

    # Rotation Transformations ############################

    g_rotb = sym.Matrix([[sym.cos(th_b), -sym.sin(th_b), 0, 0],
                         [sym.sin(th_b), sym.cos(th_b), 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

    g_rotr = sym.Matrix([[sym.cos(p_r), 0, sym.sin(p_r), 0],
                         [0, 1, 0, 0],
                         [-sym.sin(p_r), 0, sym.cos(p_r), 0],
                         [0, 0, 0, 1]])

    g_rotl = sym.Matrix([[sym.cos(p_l), 0, sym.sin(p_l), 0],
                         [0, 1, 0, 0],
                         [-sym.sin(p_l), 0, sym.cos(p_l), 0],
                         [0, 0, 0, 1]])

    rot_ang = -((sym.pi/2) - th_t)
    g_rott = sym.Matrix([[sym.cos(rot_ang), 0, sym.sin(rot_ang), 0],
                         [0, 1, 0, 0],
                         [-sym.sin(rot_ang), 0, sym.cos(rot_ang), 0],
                         [0, 0, 0, 1]])


################################################################################


    # Final Transformations
    g_wa = g_transa * g_rota

    g_wb = g_transa * g_rotb * g_transb

    g_wt = g_wb * g_transb

    ###########################
    ### Assemble Legrangian ###

    #### Get KEs ####

    # KE = 1/2 *  V.T * G * V

    # Set Velocity Vectors

    Va = TransInv(g_wa) * g_wa.diff(t)
    Va = se3ToVec(Va)

    Vb = TransInv(g_wb) * g_wb.diff(t)
    Vb = se3ToVec(Vb)

    KEa = sym.simplify(.5 * Va.T * G_wheel * Va)

    KEb = sym.simplify(.5 * Vb.T * G_body * Vb)

    KE_tot = sym.simplify(KEa[0] + KEb[0])

    #### Get PEs ####
    # PE from the C.O.M. of the body

    y_b = (length/2) * sym.sin(th_b) + wheel_radius
    PEb = m_body * g * y_b

    PE_tot = PEb

    # Get Legrangian
    L = KE_tot - PE_tot

    # Set Constraints
    #
    # Model No-Slip Condition for the wheel

    phi = q[0] + wheel_radius * q[1]
    phi_d = qd[0] + wheel_radius * qd[1]
    phi_dd = phi_d.diff(t)

    phi = sym.Matrix([phi])
    grad_phi = phi.jacobian(q).T

    # phi_dd = sym.Eq(phi_dd,0)

    # display(phi_dd)

    #### Get EL Equations ####
    dLdq = sym.Matrix([L]).jacobian(q).T
    dLdqdot = sym.Matrix([L]).jacobian(qd).T
    dLdqdot_dt = dLdqdot.diff(t)

    EL_eq = sym.Eq(dLdqdot_dt-dLdq, F_mat + lam*grad_phi)

    EL_eq = EL_eq.subs(subber_q)
    phi_dd = phi_dd.subs(subber_q)

    display(EL_eq)
    return
