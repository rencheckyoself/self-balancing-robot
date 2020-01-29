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

def dynamics(cur_loc):

    Xbdd_calc = EOM_lxb(cur_loc[0], cur_loc[1], cur_loc[2], cur_loc[3], cur_loc[4], cur_loc[5], cur_loc[6], cur_loc[7], cur_loc[8], cur_loc[9], cur_loc[10], cur_loc[11])
    Ybdd_calc = EOM_lyb(cur_loc[0], cur_loc[1], cur_loc[2], cur_loc[3], cur_loc[4], cur_loc[5], cur_loc[6], cur_loc[7], cur_loc[8], cur_loc[9], cur_loc[10], cur_loc[11])
    Tbdd_calc = EOM_ltb(cur_loc[0], cur_loc[1], cur_loc[2], cur_loc[3], cur_loc[4], cur_loc[5], cur_loc[6], cur_loc[7], cur_loc[8], cur_loc[9], cur_loc[10], cur_loc[11])
    Prdd_calc = EOM_lpr(cur_loc[0], cur_loc[1], cur_loc[2], cur_loc[3], cur_loc[4], cur_loc[5], cur_loc[6], cur_loc[7], cur_loc[8], cur_loc[9], cur_loc[10], cur_loc[11])
    Pldd_calc = EOM_lpl(cur_loc[0], cur_loc[1], cur_loc[2], cur_loc[3], cur_loc[4], cur_loc[5], cur_loc[6], cur_loc[7], cur_loc[8], cur_loc[9], cur_loc[10], cur_loc[11])
    Ttdd_calc = EOM_ltt(cur_loc[0], cur_loc[1], cur_loc[2], cur_loc[3], cur_loc[4], cur_loc[5], cur_loc[6], cur_loc[7], cur_loc[8], cur_loc[9], cur_loc[10], cur_loc[11])

    # (xb, xb_dot, yb, yb_dot, tb, tb_dot, pr, pr_dot, pl, pl_dot, tt, tt_dot)
    pos = np.array([cur_loc[1],
                  Xbdd_calc,
                  cur_loc[3],
                  Ybdd_calc,
                  cur_loc[5],
                  Tbdd_calc,
                  cur_loc[7],
                  Prdd_calc,
                  cur_loc[9],
                  Pldd_calc,
                  cur_loc[11],
                  Ttdd_calc])
    return pos

####################
# Simulation helpers
def integrate(f,x0,dt):
    """
    This function takes in an initial condition x0 and a timestep dt,
    as well as a dynamical system f(x) that outputs a vector of the
    same dimension as x0. It outputs a vector x at the future time step.

    Code Provided by Tommy Burretta
    """
    k1=dt*f(x0)
    k2=dt*f(x0+k1/2.)
    k3=dt*f(x0+k2/2.)
    k4=dt*f(x0+k3)
    xnew=x0+(1/6.)*(k1+2.*k2+2.*k3+k4)
    return xnew

def simulate(f,x0,tspan,dt):
    """
    This function takes in an initial condition x0, a timestep dt,
    a time span tspan consisting of a list [min_time, max_time],
    as well as a dynamical system f(x) that outputs a vector of the
    same dimension as x0. Additionally, this includes a flag (default false)
    that allows one to supply an Euler intergation scheme instead of
    the given scheme. It outputs a full trajectory simulated
    over the time span of dimensions (xvec_size, time_vec_size).

    Code Provided by Tommy Burretta
    """

    N = int((max(tspan)-min(tspan))/dt)
    x = np.copy(x0)
    tvec = np.linspace(min(tspan),max(tspan),N)
    xtraj = np.zeros((len(x0),N))

    for i in range(N):
        xtraj[:,i] = integrate(f,x,dt)

        x = np.copy(xtraj[:,i])
    return xtraj


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

###########################
# Create External Forces
#
# Proportional Controled Torque applied to the angle of the wheel
set_pt = sym.pi/2
kp = 0
F_tht = kp*(set_pt - th_b)

F_mat = sym.Matrix([[0],
                    [0],
                    [0],
                    [F_tht],
                    [F_tht],
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


# Mid-Transformations

g_br = g_transr * g_rotr

g_bl = g_transl * g_rotl

g_bt = g_transt * g_rott

# Final Transformations

g_wb = g_transb * g_rotb

g_wr = g_wb * g_br

g_wl = g_wb * g_bl

g_wt = g_wb * g_bt


print("Setup Complete")
###########################
### Assemble Legrangian ###

#### Get KEs ####

# KE = 1/2 *  V.T * G * V

# Set Velocity Vectors
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

#### Get PEs ####
# PE from the C.O.M. of the body

PEb = m_body * g * g_wt[2, 3]

PE_tot = PEb

# Get Legrangian
L = KE_tot - PE_tot

print("Legrangian Assembled")
# Set Constraints
#
# Model No-Slip Condition for the wheels
# See Documentation for derivation
# Robotic Manipulation, Murray, Li, & Sastry pg. 272

lam_mat = sym.Matrix([lam1, lam2])
grad_phi = sym.Matrix([[1, 0, 0, -wheel_radius*sym.cos(th_b), -wheel_radius*sym.cos(th_b), 0],
                       [0, 1, 0, -wheel_radius*sym.sin(th_b), -wheel_radius*sym.sin(th_b), 0]])

phi_d = grad_phi * qd
phi_dd = phi_d.diff(t)

phi_dd = sym.Eq(phi_dd, sym.Matrix([[0], [0]]))

#### Get EL Equations ####
dLdq = sym.Matrix([L]).jacobian(q).T
dLdqdot = sym.Matrix([L]).jacobian(qd).T
dLdqdot_dt = dLdqdot.diff(t)

EL_eq = sym.Eq(dLdqdot_dt-dLdq, F_mat + grad_phi.T * lam_mat)

EL_eq = EL_eq.subs(subber_q)
phi_dd = phi_dd.subs(subber_q)

print("Found Euler-Legrange Equations")

sols = sym.solve([EL_eq, phi_dd], [Xbdd, Ybdd, Tbdd, Prdd, Pldd, Ttdd, lam1, lam2])

print("Solved Equations of Motion")

EOM_xb = sym.Eq(Xbdd, sols[Xbdd])
EOM_yb = sym.Eq(Ybdd, sols[Ybdd])
EOM_tb = sym.Eq(Tbdd, sols[Tbdd])
EOM_pr = sym.Eq(Prdd, sols[Prdd])
EOM_pl = sym.Eq(Pldd, sols[Pldd])
EOM_tt = sym.Eq(Ttdd, sols[Ttdd])

q_list = [Xb, Xbd, Yb, Ybd, Tb, Tbd, Pr, Prd, Pl, Pld, Tt, Ttd]
EOM_lxb = sym.lambdify(q_list, EOM_xb.rhs)
EOM_lyb = sym.lambdify(q_list, EOM_yb.rhs)
EOM_ltb = sym.lambdify(q_list, EOM_tb.rhs)
EOM_lpr = sym.lambdify(q_list, EOM_pr.rhs)
EOM_lpl = sym.lambdify(q_list, EOM_pl.rhs)
EOM_ltt = sym.lambdify(q_list, EOM_tt.rhs)

# Simulation
#### Initial Conditions ####

# (xb, xb_dot, yb, yb_dot, tb, tb_dot, pr, pr_dot, pl, pl_dot, tt, tt_dot)
init_q = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (np.pi/2 - .1), 0]

tspan = [0,10]
dt = .01
N = int((tspan[1] - tspan[0])/dt)

tvec = np.linspace(0,tspan[1],N)

print("Simulating...")
xvec = simulate(dynamics, init_q, tspan, dt)

### Plot to Verify Results
fig1 = plt.figure(dpi=110,facecolor='w')
plt.grid(True)
plt.xlabel('Time')
plt.ylabel('Position')
plt.plot(xvec[0], xvec[2])
plt.autoscale(True)
plt.title("Robot World Position")
plt.legend(['$x_a(t)$',r'$\theta_a(t)$'])

fig1 = plt.figure(dpi=110,facecolor='w')
plt.grid(True)
plt.xlabel('Time')
plt.ylabel('Position')
plt.plot(tvec, xvec[10])
plt.autoscale(True)
plt.title("Body Angle Over Time")
plt.legend([r'$\theta_b(t)$'])

fig1 = plt.figure(dpi=110,facecolor='w')
plt.grid(True)
plt.xlabel('Time')
plt.ylabel('Position')
plt.plot(tvec, xvec[6], tvec, xvec[8])
plt.autoscale(True)
plt.title("Wheel Angles Over Time")
plt.legend([r'$\theta_b(t)$'])

plt.show()
