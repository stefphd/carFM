function x1 = estGuessRates(x,v,omega,v__w,car,xscale)
%ESTGUESSRATES Estimates V__z, Omega__x, Omega__y, Omega__z, delta__dot,
%z__fldot, z__frdot, z__rldot, z__rrdot using finite difference. 
%Used when guess is from GGMLTS: in that case because these variables are 
%computed under QSS assumption and may not consistent with dynamics (e.g. always 0)

%% Scaling
vscale = xscale; % same of xscale

%% Vars
% states
phi        = x(1) *xscale(1);
mu         = x(2) *xscale(2);
z          = x(3) *xscale(3);
V__P       = x(9) *xscale(9);
lambda__P  = x(10)*xscale(10);
% state derivatives
v__phi        = v(1) *vscale(1);
v__mu         = v(2) *vscale(2);
v__z          = v(3) *vscale(3);
v__delta      = v(4) *vscale(4);
v__z__fl      = v(5) *vscale(5);
v__z__fr      = v(6) *vscale(6);
v__z__rl      = v(7) *vscale(7);
v__z__rr      = v(8) *vscale(8);
% Ang vels
omega__x = omega(1);
omega__y = omega(2);
omega__z = omega(3);

%% Unpack car data
carfm.common.unpackCar;

%% Calc
% assigned vars: V__z, Omega__x, Omega__y, Omega__z, delta__dot, z__fldot, z__frdot, z__rldot, z__rrdot
carfm.dymlts.maple.vehRates;

%% Assign outputs
x1 = 1*x; % 1*x to copy obj (otherwise using reference, i.e. change in x1 reflects in change in x!)
x1(11) = V__z      /xscale(11);
x1(12) = delta__dot/xscale(12);
x1(13) = z__fldot  /xscale(13);
x1(14) = z__frdot  /xscale(14);
x1(15) = z__rldot  /xscale(15);
x1(16) = z__rrdot  /xscale(16);
x1(17) = Omega__z  /xscale(17);
x1(18) = Omega__x  /xscale(18);
x1(19) = Omega__y  /xscale(19);

end