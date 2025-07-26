% Reference Point Kinematics  *** DO NOT EDIT ***
t1 = sec(phi);
t2 = sin(phi);
t4 = sin(mu);
t6 = cos(mu);
yaw__rate = (t2 * omega__y - t4 * Omega__x + t6 * Omega__z) * t1;
t9 = cos(lambda__P);
V__xP = V__P * t9;
t10 = sin(lambda__P);
V__yP = V__P * t10;
t12 = V__P * (yaw__rate + v__lambda__P);
a__xP = -t10 * t12 + t9 * v__V__P + v__w * omega__y;
a__yP = t10 * v__V__P + t12 * t9 - omega__x * v__w;
a__tP = -t10 * v__w * omega__x + t9 * v__w * omega__y + v__V__P;
a__nP = -t10 * v__w * omega__y - t9 * v__w * omega__x + t12;
