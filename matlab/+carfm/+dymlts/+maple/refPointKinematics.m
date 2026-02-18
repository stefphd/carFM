% Reference Point Kinematics  *** DO NOT EDIT ***
t1 = sec(phi);
t2 = cos(mu);
t4 = sin(phi);
t6 = sin(mu);
yaw__rate = (t2 * Omega__z + t4 * omega__y - Omega__x * t6) * t1;
t9 = cos(lambda__P);
V__xP = V__P * t9;
t10 = sin(lambda__P);
V__yP = V__P * t10;
t12 = V__P * (yaw__rate + v__lambda__P);
t14 = omega__y * v__w;
a__xP = -t10 * t12 + t9 * v__V__P + t14;
t18 = omega__x * v__w;
a__yP = t10 * v__V__P + t12 * t9 - t18;
a__tP = -t10 * v__w * omega__x + t9 * v__w * omega__y + v__V__P;
a__nP = -t10 * t14 - t18 * t9 + t12;
