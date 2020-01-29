import numpy as np
import sympy as sym
from sympy.abc import t
import matplotlib.pyplot as plt
import scipy.linalg
import time

####################
# Simulation helpers
def integrate(f,x0,dt):
    """
    This function takes in an initial condition x0 and a timestep dt,
    as well as a dynamical system f(x) that outputs a vector of the
    same dimension as x0. It outputs a vector x at the future time step.
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
    """

    N = int((max(tspan)-min(tspan))/dt)
    x = np.copy(x0)
    tvec = np.linspace(min(tspan),max(tspan),N)
    xtraj = np.zeros((len(x0),N))

    for i in range(N):
        xtraj[:,i] = integrate(f,x,dt)

        x = np.copy(xtraj[:,i])
    return xtraj

def se3ToVec(g):

  V = sym.Matrix([g[0,3],
                  g[1,3],
                  g[2,3],
                  g[3,1],
                  g[0,2],
                  g[1,0]])

  return V

def TransInv(g):

  g_inv = sym.zeros(4,4)

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

  Xadd_calc = EOM_lxa(cur_loc[0], cur_loc[1], cur_loc[2], cur_loc[3], cur_loc[4], cur_loc[5])
  thadd_calc = EOM_lta(cur_loc[0], cur_loc[1], cur_loc[2], cur_loc[3], cur_loc[4], cur_loc[5])
  thbdd_calc = EOM_ltb(cur_loc[0], cur_loc[1], cur_loc[2], cur_loc[3], cur_loc[4], cur_loc[5])

  #(xa, xa_dot, tha, tha_dot, thb, thb_dot)
  pos = np.array([cur_loc[1],
                  Xadd_calc,
                  cur_loc[3],
                  thadd_calc,
                  cur_loc[5],
                  thbdd_calc])
  return pos


###########################
# Rigid Body Properties
#
# Wheel Properties

wheel_radius = 1
m_wheel = 1
I_wheel = (m_wheel * .5 * wheel_radius**2) # Thin disk approximation

G_wheel = sym.Matrix(np.diagflat([m_wheel, m_wheel, m_wheel, 1, 1, I_wheel]))

# Puck Properties
# The puck is represented by an equallateral triangle
length = 3
width = 1
m_body = 4

I_body = (1/12)*m_body*(4*(length**2) + width**2) # Thin plate approximation
G_body = sym.Matrix(np.diagflat([m_body, m_body, m_body, 1, 1, I_body]))

###########################
# Config Set Up

# Set Symbols
g = 9.81
lam = sym.symbols(r'\lambda')

# Set Functions
x_a = sym.Function('x_a')(t)
th_a = sym.Function(r'\theta_a')(t)
th_b = sym.Function(r'\theta_b')(t)

# Set Config Matrix
q = sym.Matrix([x_a, th_a, th_b])
qd = q.diff(t)
qdd = qd.diff(t)

# Set Dummy Vars
Xa, Xad, Xadd = sym.symbols('x_a \dot{x_a} \ddot{x_a}')
ta, tad, tadd = sym.symbols(r' \theta_a \dot{\theta_a} \ddot{\theta_a}')
tb, tbd, tbdd = sym.symbols(r' \theta_b \dot{\theta_b} \ddot{\theta_b}')

subber_q = {q[0]:Xa, qd[0]:Xad, qdd[0]:Xadd,
            q[1]:ta, qd[1]:tad, qdd[1]:tadd,
            q[2]:tb, qd[2]:tbd, qdd[2]:tbdd}

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
  # Frame W - World frame at (0,0)
  # Frame a - on the axel of the wheel
  # Frame b - at the center of mass of the body
  # Frame t - at the top center of the body

g_transa = sym.Matrix([[1, 0, 0, x_a],
                       [0, 1, 0, wheel_radius],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])


g_transb = sym.Matrix([[1, 0, 0, 0],
                       [0, 1, 0, length/2],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])

g_rota = sym.Matrix([[sym.cos(th_a), -sym.sin(th_a), 0, 0],
                     [sym.sin(th_a), sym.cos(th_a), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

rot_ang = -((sym.pi/2) - th_b)
g_rotb = sym.Matrix([[sym.cos(rot_ang), -sym.sin(rot_ang), 0, 0],
                     [sym.sin(rot_ang), sym.cos(rot_ang), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


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

sols = sym.solve([EL_eq, phi_dd],[Xadd, tadd, tbdd, lam])

#### Get Equations of Motion ####
EOM_xa = sym.Eq(Xadd, sols[Xadd])
EOM_ta = sym.Eq(tadd, sols[tadd])
EOM_tb = sym.Eq(tbdd, sols[tbdd])

EOM_lxa = sym.lambdify([Xa, Xad, ta, tad, tb, tbd], EOM_xa.rhs)
EOM_lta = sym.lambdify([Xa, Xad, ta, tad, tb, tbd], EOM_ta.rhs)
EOM_ltb = sym.lambdify([Xa, Xad, ta, tad, tb, tbd], EOM_tb.rhs)

#### Initial Conditions ####

#(xa, xa_dot, tha, tha_dot, thb, thb_dot)
init_q = [0, 0, 0, 0, (np.pi/2 - .1), 0]

tspan = [0,10]
dt = .01
N = int((tspan[1] - tspan[0])/dt)

tvec = np.linspace(0,tspan[1],N)
xvec = simulate(dynamics, init_q, tspan, dt)

### Plot to Verify Results
fig1 = plt.figure(dpi=110,facecolor='w')
plt.grid(True)
plt.xlabel('Time')
plt.ylabel('Position')
plt.plot(tvec, xvec[0], tvec, xvec[2])
plt.autoscale(True)
plt.title("Positon Over Time")
plt.legend(['$x_a(t)$',r'$\theta_a(t)$'])

fig1 = plt.figure(dpi=110,facecolor='w')
plt.grid(True)
plt.xlabel('Time')
plt.ylabel('Position')
plt.plot(tvec, xvec[4])
plt.autoscale(True)
plt.title("Positon Over Time")
plt.legend([r'$\theta_b(t)$'])

plt.show()
