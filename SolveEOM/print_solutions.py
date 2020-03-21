import pickle
import sympy as sym
from sympy import *

Xb, Xbd, Xbdd = sym.symbols('x_b x_bd x_bdd')
Yb, Ybd, Ybdd = sym.symbols('y_b y_bd y_bdd')
Tb, Tbd, Tbdd = sym.symbols('t_b t_bd t_bdd')
Pr, Prd, Prdd = sym.symbols('p_r p_rd p_rdd')
Pl, Pld, Pldd = sym.symbols('p_l p_ld p_ldd')
Tt, Ttd, Ttdd = sym.symbols('t_t t_td t_tdd')

test_file = open('Sols.pickle', 'rb')
sols = pickle.load(test_file)
test_file.close()

print("x_b solution")
print(latex(sym.simplify(sols[Xbdd])))

print("y_b solution")
print(latex(sym.simplify(sols[Ybdd])))

print("th_b solution")
print(latex(sym.simplify(sols[Tbdd])))

print("P_r solution")
print(latex(sym.simplify(sols[Prdd])))

print("P_l solution")
print(latex(sym.simplify(sols[Pldd])))

print("th_t solution")
print(latex(sym.simplify(sols[Ttdd])))
