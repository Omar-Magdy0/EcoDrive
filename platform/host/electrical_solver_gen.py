import sympy as sp

# Define Symbols
Vbus = sp.symbols('Vbus')
Vn = sp.symbols('Vn')
# Phase Node Voltages
Vp = sp.symbols('Vpa Vpb Vpc')
# Switch Conductances (Representing MOSFET + Diode state)
Ghs = sp.symbols('Gha Ghb Ghc')
Gls = sp.symbols('Gla Glb Glc')
# Load parameters
GL = sp.symbols('GL') # 1 / (R + L/dt)
Ih = sp.symbols('Iha Ihb Ihc') # History terms

equations = []

# KCL at each Phase Node (Vp_j)
for j in range(3):
    # Current leaving Vp_j:
    # To Vbus + To Ground + To Neutral
    eq = (Vp[j] - Vbus)*Ghs[j] + (Vp[j] - 0)*Gls[j] + (Vp[j] - Vn)*GL - Ih[j]
    equations.append(eq)

# KCL at Neutral Node (Vn)
# Sum of currents from Phase Nodes into Vn
eq_n = (Vn - Vp[0])*GL + (Vn - Vp[1])*GL + (Vn - Vp[2])*GL
equations.append(eq_n)

# Solve for the 4 unknown voltages
solutions = sp.solve(equations, [Vp[0], Vp[1], Vp[2], Vn])

# Simplify and print C++ for Vn (the "master" variable)
print("// Generated Vn Expression:")
print(sp.ccode(sp.simplify(solutions[Vn])))