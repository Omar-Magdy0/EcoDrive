import sympy as sp
from sympy.printing.c import ccode
from sympy.utilities.codegen import codegen
# --- SYMBOLS ---
state_i = sp.IndexedBase('state->i', shape=(3,))
state_theta = sp.symbols('state->theta')
state_omega = sp.symbols('state->omega')

p_Ld, p_Lq, p_Rs, p_pp, p_Ke, p_J, p_B = sp.symbols('p->Ld p->Lq p->Rs p->pp p->Ke p->J p->B')

in_rthev = sp.IndexedBase('in->rthev', shape=(3,))
in_vthev = sp.IndexedBase('in->vthev', shape=(3,))
out_T = sp.symbols('out->T')
 
dt = sp.symbols('dt')

# Unknowns for this timestep
next_i = sp.IndexedBase('next->i', shape=(3,))
next_v = sp.IndexedBase('next->v', shape=(3,))
next_vn = sp.symbols('next->vn')

#BEGIN FORMULATING SYSTEM EQUATION

# --- SALIENCY MATH ---
theta_e = state_theta * p_pp
omega_e = state_omega * p_pp
Lavg = (p_Ld + p_Lq)/2
Lvar = (p_Ld - p_Lq)/2

# --- THE EQUATION SYSTEM ---
# KVL: V_applied - V_n - E = L*di/dt + R*i
L = [None]*3
e = [None]*3
# --- THE SIMPLIFIED MATH LOGIC ---

# 1. Define Transient Conductance (G) and Equivalent Source (Veq)
# This mathematically "pre-solves" the L*di/dt terms
G = [None] * 3
Veq = [None] * 3
dL_dtheta = [None] * 3
#back-emf formula
for i in range(3):
    e[i] = p_Ke * omega_e * sp.cos(theta_e - (2 * sp.pi / 3) * i) 

for i in range(3):
    # Transient Impedance Z = L/dt + R
    L[i] = Lavg - Lvar * sp.cos(2 * theta_e - (2 * sp.pi / 3)*i)
    dL_dtheta[i] = 2 * Lvar * sp.sin(2 * theta_e - (2 * sp.pi / 3)*i)

    Z_transient = (L[i] / dt) + in_rthev[i] + p_Rs
    G[i] = 1 / Z_transient
    
    # Inductive Momentum + Applied Voltage - BackEMF
    Veq[i] = in_vthev[i] - e[i] + (L[i] / dt) * state_i[i]

# 2. Solve for Vn (Neutral Point)
# This comes from the law: Sum(G_i * (Veq_i - Vn)) = 0
vn_expr = (G[0]*Veq[0] + G[1]*Veq[1] + G[2]*Veq[2]) / (G[0] + G[1] + G[2])

# 3. Calculate next_i
# Now that Vn is defined, next_i is just Ohm's Law
phase_current = [
    G[0] * (Veq[0] - vn_expr),
    G[1] * (Veq[1] - vn_expr),
    G[2] * (Veq[2] - vn_expr), 
]

terminals_voltage = [
    in_vthev[0] - (phase_current[0] * in_rthev[0]),
    in_vthev[1] - (phase_current[1] * in_rthev[1]),
    in_vthev[2] - (phase_current[2] * in_rthev[2]),
    vn_expr
]


# 1. Alignment (Magnet) Torque
# Torque = Sum( (e_i / omega_m) * i_i )
t_alignment = 0
for i in range(3):
    # This represents (d_psi / d_theta) * i
    t_alignment += (p_Ke * p_pp * sp.cos(theta_e - (2 * sp.pi / 3) * i) * phase_current[i])

# 2. Reluctance Torque
t_reluctance = 0
for i in range(3):
    # T_rel = (p/2) * i^2 * dL/dtheta
    t_reluctance += (p_pp / 2) * (phase_current[i]**2) * dL_dtheta[i]

net_torque = [t_alignment + t_reluctance]
# Solve mechanical domain



exprs = phase_current + terminals_voltage + net_torque



# --- C++ GENERATION ---
print("// Generated Solver for EcoDrive (Transient Admittance Method)")

vars_to_solve = [next_i[0], next_i[1], next_i[2], next_v[0], next_v[1], next_v[2], next_vn, out_T]
# Update your exprs list to include them
substitutions, simplified_exprs = sp.cse(exprs)

# Open the file for writing
with open("pmsm_model.h", "w") as f:
    # 1. Write Header Guard and Structs
    f.write("#ifndef PMSM_MODEL_H\n#define PMSM_MODEL_H\n\n")
    f.write("#include <math.h>\n\n")
    
    f.write("typedef struct {\n    float i[3];\n    float v[3];\n   float vn;\n   float theta;\n    float omega;\n} MotorState;\n\n")
    f.write("typedef struct {\n    float vthev[3];\n    float rthev[3];\n} MotorInput;\n\n")
    f.write("typedef struct {\n    float Ld, Lq, Rs, pp, Ke, J, B;\n} MotorParams;\n\n")
    f.write("typedef struct {\n    float T;\n} MotorOutput;\n\n")

    # 2. Write Function Signature
    f.write("static inline void pmsm_step(const MotorState* state, const MotorInput* in, const MotorParams* p, float dt, MotorState* next, MotorOutput* out) {\n")

    # 3. Write Optimized Math (Intermediate variables)
    for var, expr in substitutions:
        f.write(f"    const float {ccode(var)} = {ccode(expr)};\n")

    f.write("\n")

    # 4. Write Final Assignments
    # Map the simplified expressions back to the struct members
    for i, var in enumerate(vars_to_solve):
        # Determine the assignment target name
        target = ccode(var)
        # Use simple string replacement to map SymPy names to C-struct pointers      
        f.write(f"    {target} = {ccode(simplified_exprs[i])};\n")

    f.write("}\n\n#endif // PMSM_MODEL_H\n")

print("Done! pmsm_model.h has been generated.")



# --- SYMBOLS ---
state_i = sp.IndexedBase('state->i', shape=(3,))
# state_v can be used if you decide to replace in_vbus/2 with a floating voltage
state_v = sp.IndexedBase('state->v', shape=(3,))

out_rthev = sp.IndexedBase('out->rthev', shape=(3,))
out_vthev = sp.IndexedBase('out->vthev', shape=(3,))

in_vbus = sp.symbols('in->vbus')
in_duty = sp.IndexedBase('in->duty', shape=(3,))
in_drive = sp.IndexedBase('in->drive', shape=(3,))

p_Rdson, p_RdsOff, p_FWmin = sp.symbols('p->Rdson p->RdsOff p->FWmin')

# --- LOGIC FORMULATION ---
rser = [None] * 3
vsource = [None] * 3

for i in range(3):
    # Diode conduction logic
    fw_top = state_i[i] < -p_FWmin     # Current flowing into the rail
    fw_bottom = state_i[i] > p_FWmin   # Current flowing from the rail
    
    # Thevenin Voltage: PWM | Top Diode | Bottom Diode | High-Z
    vsource[i] = sp.Piecewise(
        (in_vbus * in_duty[i], sp.Eq(in_drive[i], 1)),
        (in_vbus + 0.7, fw_top),
        (-0.7, fw_bottom),
        (in_vbus / 2, True) 
    )
    
    # Thevenin Resistance: Low (Active/Diode) | High (Off)
    rser[i] = sp.Piecewise(
        (p_Rdson, sp.Eq(in_drive[i], 1) | fw_top | fw_bottom),
        (p_RdsOff, True)
    )

# Bundle all outputs for CSE
all_exprs = vsource + rser
substitutions, simplified_exprs = sp.cse(all_exprs)

# --- FILE GENERATION ---
with open("inverter_model.h", "w") as f:
    f.write("#ifndef INVERTER_MODEL_H\n#define INVERTER_MODEL_H\n\n")
    f.write("#include <math.h>\n#include <stdbool.h>\n\n")
    
    # Structures consistent with your style
    f.write("typedef struct {\n    float i[3];\n    float v[3];\n} InverterState;\n\n")
    f.write("typedef struct {\n    float vbus;\n    float duty[3];\n    float drive[3];\n} InverterInput;\n\n")
    f.write("typedef struct {\n    float Rdson, RdsOff, FWmin;\n} InverterParams;\n\n")
    f.write("typedef struct {\n    float vthev[3];\n    float rthev[3];\n} InverterOutput;\n\n")

    # Function Signature
    f.write("static inline void inverter_step(const InverterState* state, const InverterInput* in, const InverterParams* p, InverterOutput* out) {\n")

    # Optimized Intermediate Variables (CSE)
    f.write("    // --- Optimized Intermediate Logic ---\n")
    for var, expr in substitutions:
        # Ensure floating point literals (0.7f) and clean C code
        clean_expr = ccode(expr).replace("0.7", "0.7f")
        f.write(f"    const float {ccode(var)} = {clean_expr};\n")

    f.write("\n    // --- Final Assignments ---\n")
    
    # Map back to out_vthev[0-2] and out_rthev[0-2]
    for i in range(3):
        f.write(f"    {ccode(out_vthev[i])} = {ccode(simplified_exprs[i]).replace('0.7', '0.7f')};\n")
    
    for i in range(3):
        f.write(f"    {ccode(out_rthev[i])} = {ccode(simplified_exprs[i+3]).replace('0.7', '0.7f')};\n")

    f.write("}\n\n#endif // INVERTER_MODEL_H\n")

print("Done! inverter_model.h has been generated.")





    

