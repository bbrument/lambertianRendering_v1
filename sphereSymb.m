clear
syms x y z t R x_c y_c z_c u_x u_y u_z U_x U_y U_z real

eqn = R^2 - (U_x + t*u_x - x_c)^2 - (U_y + t*u_y - y_c)^2 - (U_z + t*u_z - z_c)^2;
simplify(eqn)
C = coeffs(eqn,t)

a = C(3); b = C(2); c = C(1);

delta = b^2 -4*a*c

sol = solve(eqn,t)


