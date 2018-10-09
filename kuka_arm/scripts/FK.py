from mpmath import *
from sympy import *

dtr = pi/180

### Your FK code here
# Create symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
# Create Modified DH parameters
s = {alpha0: 0,       a0: 0,      d1: 0.33+0.42,  q1: q1,
     alpha1: -90*dtr, a1: 0.35,   d2: 0,          q2: q2-90*dtr,
     alpha2: 0,       a2: 1.25,   d3: 0,          q3: q3,
     alpha3: -90*dtr, a3: -0.054, d4: 0.96+0.54,  q4: q4,
     alpha4: 90*dtr,  a4: 0,      d5: 0,          q5: q5,
     alpha5: -90*dtr, a5: 0,      d6: 0,          q6: q6,
     alpha6: 0,       a6: 0,      d7: 0.193+0.11, q7: 0}
# Define Modified DH Transformation matrix
def transMat(q, d, a, alpha):
    transform = Matrix([
        [cos(q), -sin(q), 0, a],
        [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
        [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alpha)*d],
        [0, 0, 0, 1]])
    return transform.subs(s)
T0_1 = transMat(q1, d1, a0, alpha0)
T0_2 = simplify(T0_1 * transMat(q2, d2, a1, alpha1))
T0_3 = simplify(T0_2 * transMat(q3, d3, a2, alpha2))
T0_4 = simplify(T0_3 * transMat(q4, d4, a3, alpha3))
T0_5 = simplify(T0_4 * transMat(q5, d5, a4, alpha4))
T0_6 = simplify(T0_5 * transMat(q6, d6, a5, alpha5))
T0_EE = simplify(T0_6 * transMat(q7, d7, a6, alpha6))

# Create individual transformation matrices
R_y = Matrix([[cos(-90*dtr),  0, sin(-90*dtr), 0],
              [0,           1,              0, 0],
              [-sin(-90*dtr), 0, cos(-90*dtr), 0],
              [0,         0,        0,         1]])
R_z = Matrix([[cos(180*dtr), -sin(180*dtr), 0, 0],
              [sin(180*dtr),  cos(180*dtr), 0, 0],  
              [           0,             0, 1, 0],
              [           0,             0, 0, 1]])
R_corr = simplify(R_z * R_y)
# Extract rotation matrices from the transformation matrices
T_final = simplify(T0_EE * R_corr)
base_link = Matrix([[0], [0], [0], [1]])
# EE_position = (simplify(T_final * base_link)).evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5:0, q6:0})

Rot = T_final.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5:0, q6:0})
print (Rot)