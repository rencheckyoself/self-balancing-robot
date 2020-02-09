""" This file is used to compute the equations of motion for a self balancing robot """

import time
import numpy as np
import sympy as sym
from sympy import latex
from sympy.abc import t
import matplotlib.pyplot as plt
import scipy.linalg

import pickle

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

###########################
# Rigid Body Properties
###########################

# Robot Wheel Properties
wheel_radius, wheel_base, m_wheel = sym.symbols('r_w w_{base} m_w')
Ixxw, Iyyw, Izzw = sym.symbols('I_{xxw} I_{yyw} I_{zzw}')

# Robot Body Properties
# The body is represented by a rectangle
COMz = sym.symbols('h_{com}')
m_body = sym.symbols('m_b')
Ixxb, Iyyb, Izzb = sym.symbols('I_{xxb} I_{yyb} I_{zzb}')

# Create the Spatial Intertia Matrix
G_wheel = sym.Matrix(np.diagflat([m_wheel, m_wheel, m_wheel, Ixxw, Iyyw, Izzw]))

# G_wheel[3, 4] = Ixyw
# G_wheel[4, 3] = Ixyw
#
# G_wheel[3, 5] = Ixzw
# G_wheel[5, 3] = Ixzw
#
# G_wheel[4, 5] = Iyzw
# G_wheel[5, 4] = Iyzw

G_body = sym.Matrix(np.diagflat([m_body, m_body, m_body, Ixxb, Iyyb, Izzb]))

# G_body[3, 4] = Ixyb
# G_body[4, 3] = Ixyb
#
# G_body[3, 5] = Ixzb
# G_body[5, 3] = Ixzb
#
# G_body[4, 5] = Iyzb
# G_body[5, 4] = Iyzb

###########################
# Config Variable Set Up
g = sym.symbols('g')
lam1, lam2 = sym.symbols('l_1 l_2')

# Set Functions
x_b = sym.Function('xb')(t)
y_b = sym.Function('yb')(t)
th_b = sym.Function('thb')(t)
p_r = sym.Function('pr')(t)
p_l = sym.Function('pl')(t)
th_t = sym.Function('tht')(t)

# Set Config Matrix
q = sym.Matrix([x_b, y_b, th_b, p_r, p_l, th_t])
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


############################################################################
# DEFINE THE EXTERNAL FORCES
############################################################################
F_tht = sym.symbols('F_w')


F_mat = sym.Matrix([[0],
                    [0],
                    [0],
                    [F_tht],
                    [F_tht],
                    [0]])

############################################################################
# TRANSFORMATION SETUP
  # Frame w - World frame at (0,0,0)
  # Frame b - base link, at the mid point between the two wheel axels
  # Frame r - at the center of the right wheel
  # Frame l - at the center of the left wheel
  # Frame t - at the center of mass of the robot body.
############################################################################

# Translational Transformations ############################################


g_transb = sym.Matrix([[1, 0, 0, x_b],
                       [0, 1, 0, y_b],
                       [0, 0, 1, wheel_radius],
                       [0, 0, 0, 1]])


g_transr = sym.Matrix([[1, 0, 0, 0],
                       [0, 1, 0, -wheel_base/2],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])

g_transl = sym.Matrix([[1, 0, 0, 0],
                       [0, 1, 0, wheel_base/2],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])

# COM assumed to be directly above b frame in the center of the body
xt = COMz * sym.cos(th_t)
yt = 0
zt = COMz * sym.sin(th_t)

g_transt = sym.Matrix([[1, 0, 0, xt],
                       [0, 1, 0, yt],
                       [0, 0, 1, zt],
                       [0, 0, 0, 1]])

# Rotational Transformations ###############################################


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


# Robot Transformations - all bodies relative to the base frame ############

g_br = g_transr * g_rotr

g_bl = g_transl * g_rotl

g_bt = g_transt * g_rott

# Final Transformations - all bodies relative to the world frame ###########


g_wb = g_transb * g_rotb

g_wr = g_wb * g_br

g_wl = g_wb * g_bl

g_wt = g_wb * g_bt


print("Setup Complete -- Assembling Legrangian...")

############################################################################
# ASSEMBLE THE LEGRANGIAN
############################################################################

# Calculate the Kinetic Energy of each body ################################
# KE = 1/2 *  V.T * G * V

# Set Velocity Vectors [vx, vy, vz, wx, wy, wz]
Vt = TransInv(g_wt) * g_wt.diff(t)
Vt = se3ToVec(Vt)

Vr = TransInv(g_wr) * g_wr.diff(t)
Vr = se3ToVec(Vr)

Vl = TransInv(g_wl) * g_wl.diff(t)
Vl = se3ToVec(Vl)

KEt = sym.simplify(.5 * Vt.T * G_body * Vt)
KEr = sym.simplify(.5 * Vr.T * G_wheel * Vr)
KEl = sym.simplify(.5 * Vl.T * G_wheel * Vl)

KE_tot = sym.simplify(KEt[0] + KEr[0] + KEl[0])

# Calculate the Potential Energy of each body ##############################
# Only component is the body of the robot

PEb = m_body * g * g_wt[2, 3]

PE_tot = sym.simplify(PEb)

# Get Legrangian
L = sym.simplify(KE_tot - PE_tot)

print("Legrangian Assembled -- Setting Up E-L Equations...")

############################################################################
# DEFINE THE CONSTRAINTS
    # For derivation, see Robotic Manipulation, Murray, Li, & Sastry pg. 272
############################################################################

lam_mat = sym.Matrix([lam1, lam2])
grad_phi = sym.Matrix([[1, 0, 0, wheel_radius*sym.cos(th_b), wheel_radius*sym.cos(th_b), 0],
                       [0, 1, 0, wheel_radius*sym.sin(th_b), wheel_radius*sym.sin(th_b), 0]])

phi_d = grad_phi * qd
phi_dd = sym.simplify(phi_d.diff(t))

phi_dd = sym.Eq(phi_dd, sym.Matrix([[0], [0]]))

############################################################################
# CALCULATE EULER-LEGRANGE & HAMILTONIAN EQUATIONS
############################################################################
dLdq = sym.Matrix([L]).jacobian(q).T
dLdqdot = sym.Matrix([L]).jacobian(qd).T
dLdqdot_dt = dLdqdot.diff(t)

ham = dLdqdot.T * qd
ham = sym.simplify(ham[0] - L)

ham_subs = ham.subs(subber_q)

# print("Hamiltotian Equation")
# print(latex(ham_subs))

pickle_ham = open("Ham.pickle", "wb")
pickle.dump(ham_subs, pickle_ham, protocol=2)
pickle_ham.close()

EL_eq = sym.Eq(dLdqdot_dt-dLdq, F_mat + grad_phi.T * lam_mat)

EL_eq = sym.simplify(EL_eq.subs(subber_q))
phi_dd = sym.simplify(phi_dd.subs(subber_q))

print("Found Euler-Legrange Equations -- Solving for EOM...")

sols = sym.solve([EL_eq, phi_dd], [Xbdd, Ybdd, Tbdd, Prdd, Pldd, Ttdd, lam1, lam2])

print("Solved Equations of Motion")

pickle_sols = open("Sols.pickle", "wb")
pickle.dump(sols, pickle_sols, protocol=2)
pickle_sols.close()

# print("x_b solution")
# print(latex(sym.simplify(sols[Xbdd])))
#
# print("y_b solution")
# print(latex(sym.simplify(sols[Ybdd])))
#
# print("th_b solution")
# print(latex(sym.simplify(sols[Tbdd])))
#
# print("P_r solution")
# print(latex(sym.simplify(sols[Prdd])))
#
# print("P_l solution")
# print(latex(sym.simplify(sols[Pldd])))
#
# print("th_t solution")
# print(latex(sym.simplify(sols[Ttdd])))

EOM_xb = sym.Eq(Xbdd, sols[Xbdd])
EOM_yb = sym.Eq(Ybdd, sols[Ybdd])
EOM_tb = sym.Eq(Tbdd, sols[Tbdd])
EOM_pr = sym.Eq(Prdd, sols[Prdd])
EOM_pl = sym.Eq(Pldd, sols[Pldd])
EOM_tt = sym.Eq(Ttdd, sols[Ttdd])

eom_arr = [EOM_xb, EOM_yb, EOM_tb, EOM_pr, EOM_pl, EOM_tt]

pickle_EOMs = open("EOMs.pickle", "wb")
pickle.dump(eom_arr, pickle_EOMs, protocol=2)
pickle_EOMs.close()
