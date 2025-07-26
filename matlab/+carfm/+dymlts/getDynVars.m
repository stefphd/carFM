function [X, V, U] = getDynVars(data)
%getDynVars Convert data to inputs UNSCALED (x, v, u) for dynamicEquations.

% Extract vars
frontLeftTyre =  [data.frontLeftTyre];
frontRightTyre =  [data.frontRightTyre];
rearLeftTyre = [data.rearLeftTyre];
rearRightTyre = [data.rearRightTyre];
% internal = [data.internal];
phi = [data.rollAngle];
mu = [data.pitchAngle];
z = [data.heave];
delta = [data.steerAngle];
V__P = [data.speed];
lambda__P = [data.driftAngle];
V__z = [data.linVelocities]; V__z = V__z(3,:);
Omega__x = [data.angVelocities]; Omega__x = Omega__x(1,:);
Omega__y = [data.angVelocities]; Omega__y = Omega__y(2,:);
Omega__z = [data.angVelocities]; Omega__z = Omega__z(3,:);
z__fl = -[data.frontLeftTravel];
z__fr = -[data.frontRightTravel];
z__rl = -[data.rearLeftTravel];
z__rr = -[data.rearRightTravel];
omega__fl = [frontLeftTyre.angVelocity];
omega__fr = [frontRightTyre.angVelocity];
omega__rl = [rearLeftTyre.angVelocity];
omega__rr = [rearRightTyre.angVelocity];
% alpha__fl = [internal.alpha]; alpha__fl = alpha__fl(1,:); 
% alpha__fr = [internal.alpha]; alpha__fr = alpha__fr(2,:); 
% alpha__rl = [internal.alpha]; alpha__rl = alpha__rl(3,:); 
% alpha__rr = [internal.alpha]; alpha__rr = alpha__rr(4,:); 
Tau__t = [data.totalTorque];
delta__dot = [data.steerRate];
delta__ddot = [data.steerAcc];
z__fldot = -[data.frontLeftTravelRate];
z__frdot = -[data.frontRightTravelRate];
z__rldot = -[data.rearLeftTravelRate];
z__rrdot = -[data.rearRightTravelRate];

% These velocity vars may be non-zero when acceleraing/braking
v__V__P = [data.tangentialAcc];
v__V__z = [data.linVelocityRates]; v__V__z = v__V__z(3,:);
v__omega__fl = [frontLeftTyre.angAcceleration];
v__omega__fr = [frontRightTyre.angAcceleration];
v__omega__rl = [rearLeftTyre.angAcceleration]; 
v__omega__rr = [rearRightTyre.angAcceleration]; 

% These velocity vars are ALWAYS zero in steady-state (even when accelerating/braking)
v__phi        = zeros(1,numel(data)); 
v__mu         = zeros(1,numel(data)); 
v__z          = zeros(1,numel(data));
v__delta      = zeros(1,numel(data)); 
v__z__fl      = zeros(1,numel(data)); 
v__z__fr      = zeros(1,numel(data));
v__z__rl      = zeros(1,numel(data));
v__z__rr      = zeros(1,numel(data)); 
v__lambda__P  = zeros(1,numel(data));
v__delta__dot = zeros(1,numel(data)); 
v__z__fldot   = zeros(1,numel(data));
v__z__frdot   = zeros(1,numel(data)); 
v__z__rldot   = zeros(1,numel(data));
v__z__rrdot   = zeros(1,numel(data));
v__Omega__z   = zeros(1,numel(data)); 
v__Omega__x   = zeros(1,numel(data)); 
v__Omega__y   = zeros(1,numel(data));
% v__alpha__fl  = zeros(1,numel(data));
% v__alpha__fr  = zeros(1,numel(data));
% v__alpha__rl  = zeros(1,numel(data));
% v__alpha__rr  = zeros(1,numel(data));

% Init
X = zeros(23,numel(data));
V = zeros(23,numel(data));
U = zeros(2,numel(data));

% Assign vars
X(1 , :) = phi       ;
X(2 , :) = mu        ;
X(3 , :) = z         ;
X(4 , :) = delta     ;
X(5 , :) = z__fl     ;
X(6 , :) = z__fr     ;
X(7 , :) = z__rl     ;
X(8 , :) = z__rr     ;
X(9 , :) = V__P      ;
X(10, :) = lambda__P ;
X(11, :) = V__z      ;
X(12, :) = delta__dot;
X(13, :) = z__fldot  ;
X(14, :) = z__frdot  ;
X(15, :) = z__rldot  ;
X(16, :) = z__rrdot  ;
X(17, :) = Omega__z  ;
X(18, :) = Omega__x  ;
X(19, :) = Omega__y  ;
X(20, :) = omega__fl ;
X(21, :) = omega__fr ;
X(22, :) = omega__rl ;
X(23, :) = omega__rr ;
% X(24, :) = alpha__fl ;
% X(25, :) = alpha__fr ;
% X(26, :) = alpha__rl ;
% X(27, :) = alpha__rr ;

U(1 , :) = delta__ddot;
U(2 , :) = Tau__t;

V(1 , :) = v__phi       ;
V(2 , :) = v__mu        ;
V(3 , :) = v__z         ;
V(4 , :) = v__delta     ;
V(5 , :) = v__z__fl     ;
V(6 , :) = v__z__fr     ;
V(7 , :) = v__z__rl     ;
V(8 , :) = v__z__rr     ;
V(9 , :) = v__V__P      ;
V(10, :) = v__lambda__P ;
V(11, :) = v__V__z      ;
V(12, :) = v__delta__dot;
V(13, :) = v__z__fldot  ;
V(14, :) = v__z__frdot  ;
V(15, :) = v__z__rldot  ;
V(16, :) = v__z__rrdot  ;
V(17, :) = v__Omega__z  ;
V(18, :) = v__Omega__x  ;
V(19, :) = v__Omega__y  ;
V(20, :) = v__omega__fl ;
V(21, :) = v__omega__fr ;
V(22, :) = v__omega__rl ;
V(23, :) = v__omega__rr ;
% V(24, :) = v__alpha__fl ;
% V(25, :) = v__alpha__fr ;
% V(26, :) = v__alpha__rl ;
% V(27, :) = v__alpha__rr ;

end