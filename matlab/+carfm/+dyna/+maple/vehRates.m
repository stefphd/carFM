% Get rates  *** DO NOT EDIT ***
t1 = cos(lambda__P);
t4 = sin(lambda__P);
t7 = z - h;
V__z = mu * V__P * t1 - phi * t4 * V__P + phi * (t7 * omega__x - omega__z * (b - d)) + omega__y * (mu * t7 - b + d) + v__w + v__z;
Omega__x = -mu * omega__z + v__phi + omega__x;
Omega__y = phi * omega__z + v__mu + omega__y;
Omega__z = mu * omega__x - phi * omega__y + omega__z;
v__delta = delta__dot;
v__delta__dot = delta__ddot;
z__fldot = v__z__fl;
z__frdot = v__z__fr;
z__rldot = v__z__rl;
z__rrdot = v__z__rr;
