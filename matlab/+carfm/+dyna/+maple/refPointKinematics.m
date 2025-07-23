% Reference Point Kinematics  *** DO NOT EDIT ***
yaw__rate = Omega__z;
t1 = cos(lambda__P);
V__xP = V__P * t1;
t2 = sin(lambda__P);
V__yP = V__P * t2;
t4 = V__P * (Omega__z + v__lambda__P);
a__xP = t1 * v__V__P - t2 * t4 + omega__y * v__w;
a__yP = t1 * t4 + t2 * v__V__P - omega__x * v__w;
a__tP = t1 * v__w * omega__y - t2 * v__w * omega__x + v__V__P;
a__nP = -t1 * v__w * omega__x - t2 * v__w * omega__y + t4;
