from sympy import symbols, Matrix, sin, cos

theta, d, a, alpha = symbols('theta d a alpha')

T = Matrix([
    [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
    [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
    [0, sin(alpha), cos(alpha), d],
    [0, 0, 0, 1]
])
print(T)