function [dynamicRes,output] = dynamicEquations(x, v, u, car, opts)
% DYNAMICEQUATIONS Dynamic equations
% 
% INPUT:
% x: scaled dynamic states (23-by-1 vector)
% v: time derivative of scaled dynamic states (23-by-1 vector)
% u: scaled controls (2-by-1 vector)
% car: car data
% opts: options
%
% OUTPUT:
% dynamicRes: scaled residual of dynamic equations
% output: dynamics outputs (struct)

% Check args
arguments
    x (23,1)
    v (23,1)
    u (2,1)
    car (1,1) struct
    opts (1,1) struct
end

%% Scaling
xscale = opts.xscale;
vscale = opts.xscale; % same of xscale
uscale = opts.uscale;
rscale = opts.rscale;

%% Model options
% iTyreType = opts.iTyreType;
% iSuspensionType = opts.iSuspensionType;
iEngineBrake = opts.iEngineBrake;
iRotaryInertia = opts.iRotaryInertia;
iAeroComps = opts.iAeroComps;
% suspTrimRigid = opts.suspTrimRigid;
% tyreDeformationRigid = opts.tyreDeformationRigid;

%% States and controls
% states
phi        = x(1) *xscale(1);
mu         = x(2) *xscale(2);
z          = x(3) *xscale(3);
delta      = x(4) *xscale(4);
z__fl      = x(5) *xscale(5);
z__fr      = x(6) *xscale(6);
z__rl      = x(7) *xscale(7);
z__rr      = x(8) *xscale(8);
V__P       = x(9) *xscale(9);
lambda__P  = x(10)*xscale(10);
V__z       = x(11)*xscale(11);
delta__dot = x(12)*xscale(12);
z__fldot   = x(13)*xscale(13);
z__frdot   = x(14)*xscale(14);
z__rldot   = x(15)*xscale(15);
z__rrdot   = x(16)*xscale(16);
Omega__z   = x(17)*xscale(17);
Omega__x   = x(18)*xscale(18);
Omega__y   = x(19)*xscale(19);
omega__fl  = x(20)*xscale(20);
omega__fr  = x(21)*xscale(21);
omega__rl  = x(22)*xscale(22);
omega__rr  = x(23)*xscale(23);
% alpha__fl  = x(24)*xscale(24);
% alpha__fr  = x(25)*xscale(25);
% alpha__rl  = x(26)*xscale(26);
% alpha__rr  = x(27)*xscale(27);

% state derivatives
v__phi        = v(1) *vscale(1);
v__mu         = v(2) *vscale(2);
v__z          = v(3) *vscale(3);
v__delta      = v(4) *vscale(4);
v__z__fl      = v(5) *vscale(5);
v__z__fr      = v(6) *vscale(6);
v__z__rl      = v(7) *vscale(7);
v__z__rr      = v(8) *vscale(8);
v__V__P       = v(9) *vscale(9);
v__lambda__P  = v(10)*vscale(10);
v__V__z       = v(11)*vscale(11);
v__delta__dot = v(12)*vscale(12);
v__z__fldot   = v(13)*vscale(13);
v__z__frdot   = v(14)*vscale(14);
v__z__rldot   = v(15)*vscale(15);
v__z__rrdot   = v(16)*vscale(16);
v__Omega__z   = v(17)*vscale(17);
v__Omega__x   = v(18)*vscale(18);
v__Omega__y   = v(19)*vscale(19);
v__omega__fl  = v(20)*vscale(20);
v__omega__fr  = v(21)*vscale(21);
v__omega__rl  = v(22)*vscale(22);
v__omega__rr  = v(23)*vscale(23);
% v__alpha__fl  = v(24)*vscale(24);
% v__alpha__fr  = v(25)*vscale(25);
% v__alpha__rl  = v(26)*vscale(26);
% v__alpha__rr  = v(27)*vscale(27);

% controls
delta__ddot = u(1)*uscale(1);
Tau__t = u(2)*uscale(2);

%% Unpack car data
carfm.common.unpackCar;

%% Other params
a__x = 0;
a__y = 0;
g__x = opts.gravityFactors(1) * g;
g__y = opts.gravityFactors(2) * g;
g__z = opts.gravityFactors(3) * g;
omega__x = opts.angVelocities(1);
omega__y = opts.angVelocities(2);
omega__xdot = opts.angAccelerations(1);
omega__ydot = opts.angAccelerations(2);
v__w = opts.zVelocity;
v__wdot = opts.zVelocityRate;

%% Reference point kinematics
carfm.dymlts.maple.refPointKinematics;

%% Saturates tyre loads for tyre eval
fsat_tyre = @(x,xl) xl*(x<xl) + x*(x>=xl);

%% Suspension kinematics
% travels are z__fl,z__fr,z__rl,z__rr (>0 in extension), while user defines
% suspension kinematics using travel positive in compression: need sign changes
% Front
[x__fl, y__fl, psi__fl, phi__fl, mu__fl, ...
    D__x__fl, D__y__fl, D__psi__fl, D__phi__fl, D__mu__fl] = frontSuspension.Kinematics(frontSuspension, -z__fl, delta);
[x__fr, y__fr, psi__fr, phi__fr, mu__fr, ...
    D__x__fr, D__y__fr, D__psi__fr, D__phi__fr, D__mu__fr] = frontSuspension.Kinematics(frontSuspension, -z__fr, delta);

% fix left/right signs, depending on opts.iSuspensionSide
y__fl = -y__fl*opts.iSuspensionSide;
psi__fl = -psi__fl*opts.iSuspensionSide;
phi__fl = -phi__fl*opts.iSuspensionSide;
D__y__fl = -D__y__fl*opts.iSuspensionSide    ; 
D__psi__fl = -D__psi__fl*opts.iSuspensionSide; 
D__phi__fl = -D__phi__fl*opts.iSuspensionSide; 
y__fr = y__fr*opts.iSuspensionSide;
psi__fr = psi__fr*opts.iSuspensionSide;
phi__fr = phi__fr*opts.iSuspensionSide;
D__y__fr = D__y__fr*opts.iSuspensionSide    ;
D__psi__fr = D__psi__fr*opts.iSuspensionSide;
D__phi__fr = D__phi__fr*opts.iSuspensionSide;

% change sign for travel>0 in extension
D__y__fl   = -D__y__fl;
D__psi__fl = -D__psi__fl;
D__phi__fl = -D__phi__fl;
D__y__fr   = -D__y__fr;
D__psi__fr = -D__psi__fr;
D__phi__fr = -D__phi__fr;
 
% Rear
[x__rl, y__rl, psi__rl, phi__rl, mu__rl, ...
    D__x__rl, D__y__rl, D__psi__rl, D__phi__rl, D__mu__rl] = rearSuspension.Kinematics(rearSuspension, -z__rl, 0); % zero delta for rear
[x__rr, y__rr, psi__rr, phi__rr, mu__rr, ...
    D__x__rr, D__y__rr, D__psi__rr, D__phi__rr, D__mu__rr] = rearSuspension.Kinematics(rearSuspension, -z__rr, 0); % zero delta for rear

% fix left/right signs, depending on opts.iSuspensionSide
y__rl = -y__rl*opts.iSuspensionSide;
psi__rl = -psi__rl*opts.iSuspensionSide;
phi__rl = -phi__rl*opts.iSuspensionSide;
D__y__rl = -D__y__rl*opts.iSuspensionSide    ;
D__psi__rl = -D__psi__rl*opts.iSuspensionSide;
D__phi__rl = -D__phi__rl*opts.iSuspensionSide;
y__rr = y__rr*opts.iSuspensionSide;
psi__rr = psi__rr*opts.iSuspensionSide;
phi__rr = phi__rr*opts.iSuspensionSide;
D__y__rr = D__y__rr*opts.iSuspensionSide    ;
D__psi__rr = D__psi__rr*opts.iSuspensionSide;
D__phi__rr = D__phi__rr*opts.iSuspensionSide;

% change sign for travel>0 in extension
D__y__rl   = -D__y__rl;
D__psi__rl = -D__psi__rl;
D__phi__rl = -D__phi__rl;
D__y__rr   = -D__y__rr;
D__psi__rr = -D__psi__rr;
D__phi__rr = -D__phi__rr;


%% Rear tyres
carfm.dymlts.maple.rearTyreKinematics; % This is generated from MAPLE

% Left
N__rl = kr__r*xi__rl + cr__r*xi__rldot;
N__rl_tyre = fsat_tyre(N__rl / car.mass / car.gravity, opts.Neps) * car.mass * car.gravity; 
[X__rl,Y__rl,T__rlx,T__rly,T__rlz,ls__rl,sa__rl] = rearTyre.Forces(rearTyre,N__rl_tyre, ...
    VSrl0,VNrl0*opts.iTyreSide,-VRrl0,omega__rl, ...
    -ca__rl*opts.iTyreSide,-phit__rl*opts.iTyreSide); 

% Right
N__rr = kr__r*xi__rr + cr__r*xi__rrdot;
N__rr_tyre = fsat_tyre(N__rr / car.mass / car.gravity, opts.Neps) * car.mass * car.gravity; 
[X__rr,Y__rr,T__rrx,T__rry,T__rrz,ls__rr,sa__rr] = rearTyre.Forces(rearTyre,N__rr_tyre, ...
    VSrr0,-VNrr0*opts.iTyreSide,-VRrr0,omega__rr, ...
    ca__rr*opts.iTyreSide,phit__rr*opts.iTyreSide); 

% Fix left/right signs, depending on opts.iTyreSide
Y__rl = -Y__rl*opts.iTyreSide;
T__rlz = -T__rlz*opts.iTyreSide;
T__rlx = -T__rlx*opts.iTyreSide;
Y__rr = Y__rr*opts.iTyreSide;
T__rrz = T__rrz*opts.iTyreSide;
T__rrx = T__rrx*opts.iTyreSide;

% ISO to adapted SAE
Y__rr = -Y__rr;
T__rry = -T__rry;
T__rrz = -T__rrz;
Y__rl = -Y__rl;
T__rly = -T__rly;
T__rlz = -T__rlz;

%% Front tyres
carfm.dymlts.maple.frontTyreKinematics; % This is generated from MAPLE

% Left
N__fl = kr__r*xi__fl + cr__r*xi__fldot;
N__fl_tyre = fsat_tyre(N__fl / car.mass / car.gravity, opts.Neps) * car.mass * car.gravity; 
[X__fl,Y__fl,T__flx,T__fly,T__flz,ls__fl,sa__fl] = frontTyre.Forces(frontTyre,N__fl_tyre, ...
    VSfl0,VNfl0*opts.iTyreSide,-VRfl0,omega__fl, ...
    -ca__fl*opts.iTyreSide,-phit__fl*opts.iTyreSide); 

% Right
N__fr = kr__r*xi__fr + cr__r*xi__frdot;
N__fr_tyre = fsat_tyre(N__fr / car.mass / car.gravity, opts.Neps) * car.mass * car.gravity; 
[X__fr,Y__fr,T__frx,T__fry,T__frz,ls__fr,sa__fr] = frontTyre.Forces(frontTyre,N__fr_tyre, ...
    VSfr0,-VNfr0*opts.iTyreSide,-VRfr0,omega__fr, ...
    ca__fr*opts.iTyreSide,phit__fr*opts.iTyreSide); 

% Fix left/right signs, depending on opts.iTyreSide
Y__fl = -Y__fl*opts.iTyreSide;
T__flz = -T__flz*opts.iTyreSide;
T__flx = -T__flx*opts.iTyreSide;
Y__fr = Y__fr*opts.iTyreSide;
T__frz = T__frz*opts.iTyreSide;
T__frx = T__frx*opts.iTyreSide;

% ISO to adapted SAE
Y__fl = -Y__fl;
T__fly = -T__fly;
T__flz = -T__flz;
Y__fr = -Y__fr;
T__fry = -T__fry;
T__frz = -T__frz;

%% Aerodynamic forces
[aeroFD, aeroFS, aeroFL, aeroMR, aeroMP, aeroMY] = aero.Forces(aero, V__P, ...
    a__tP, a__nP, g__z, ...
    mu, phi, z);
%from aero frame to SAE frame (-x, +y, -z)
F__Ax = -aeroFD;
F__Ay = aeroFS;
F__Az = -aeroFL;
M__Ax = -aeroMR;
M__Ay = aeroMP;
M__Az = -aeroMY;
%disable aero components
if not(iAeroComps(1)), F__Ax = 0; end
if not(iAeroComps(2)), F__Ay = 0; end
if not(iAeroComps(3)), F__Az = 0; end
if not(iAeroComps(4)), M__Ax = 0; end
if not(iAeroComps(5)), M__Ay = 0; end
if not(iAeroComps(6)), M__Az = 0; end

%% Suspensions
% front
[F__fl, F__fr] = frontSuspension.Force(frontSuspension, -z__fl, -z__fr, -z__fldot, -z__frdot);
% rear
[F__rl, F__rr] = rearSuspension.Force(rearSuspension, -z__rl, -z__rr, -z__rldot, -z__rrdot);

%% Gear ratio
omega__r = 0.5*(omega__rr + omega__rl); % use the average between rear wheel speeds
if numel(tau__g)>1
    % eq. (27) of https://doi.org/10.1080/00423114.2021.1910718
    tau__g0 = tau__g(1) + sum(diff(tau__g)/2.*(1+sin(atan(opts.gearSharpness*(omega__r0-omegar__s(1:end-1))))),1);
else % only one gear ratio (above equation fails)
    tau__g0 = tau__g;
end
% total ratio from engine to wheel
tau__t = tau__g0; % no other ratio considered, total is gearbox!

%% Engine
omegae = omega__r/abs(tau__t);
[Tau__p__max, Tau__b] = engine.Torques(engine,omegae);
if not(iEngineBrake) % exclude engine braking
    Tau__b = 0;
end

%% Saturations
% fsat definition:
% f(x<x1) = 0
% f(x>x2) = 1
% between x1 and x2 fsat increases from 0 to 1 using a 3rd order polynomial
% with BCs f(x1)=0,f(x2)=1,f'(x1)=f'(x2)=0
fsat = @(x, x1, x2) 1*(x>=x2) + 0*(x<=x1) + ( (2*x.^3)/(x1 - x2)^3 - (3*x.^2*(x1 + x2))/(x1 - x2)^3 + (x1.^2*(x1 - 3*x2))/(x1 - x2)^3 + (6*x*x1*x2)/(x1 - x2)^3 ).*(x>x1).*(x<x2);
nfl = N__fl / car.mass / car.gravity;
nfr = N__fr / car.mass / car.gravity;
nrl = N__rl / car.mass / car.gravity;
nrr = N__rr / car.mass / car.gravity;
Tau__b = Tau__b * fsat(0.5*(nrr+nrl), 5*opts.Neps, 10*opts.Neps);
% v__omega__r = v__omega__r * fsat(nr, 5*opts.Neps, 10*opts.Neps);
% v__omega__f = v__omega__f * fsat(nf, 5*opts.Neps, 10*opts.Neps);

%% Transmission
if not(iRotaryInertia) % exclude inertia of rotating bodies
    i__ta = 0;
    i__tg = 0;
    i__fy = 0;
    i__ry = 0;
end
% Torque
% Tau__t0: total torque w.r.t. engine brake, i.e. it is 0 in coast-down,
% i.e. prop to throttle/brake pedal
Taueps = opts.Taueps;
% Split Tau__t into total driving (i.e. engine) torque Tau__td (positive part) 
% and total braking torque Tau__tb (negative part)
% Note that Tau__td includes engine drag, i.e. eventually negative in coast-down
Tau__tb = ( Tau__t - sqrt(Tau__t^2 + Taueps^2))/2; % negative part
Tau__td = ( Tau__t + sqrt(Tau__t^2 + Taueps^2))/2 - Tau__b*eta__t/abs(tau__t); % positive part
% Split braking torque on front/rear depending on brake ratio, equally between left/right
Tau__fb = gamma__b*Tau__tb;
Tau__rb = Tau__tb - Tau__fb; % (1-gamma__b)*Tau__tb
Tau__flb = 0.5*Tau__fb;
Tau__frb = 0.5*Tau__fb;
Tau__rlb = 0.5*Tau__rb;
Tau__rrb = 0.5*Tau__rb;
% Call user-function transmission.Torques to split total driving torque
% into front/rear, left/right wheels
[Tau__fld, Tau__frd, Tau__rld, Tau__rrd] = transmission.Torques(transmission, ...
    omega__fl, omega__fr, omega__rl, omega__rr, Tau__td);
% Total = brake + drive
Tau__fl = Tau__flb + Tau__fld;
Tau__fr = Tau__frb + Tau__frd;
Tau__rl = Tau__rlb + Tau__rld;
Tau__rr = Tau__rrb + Tau__rrd;

%% External force and torque
% ext force applied to the ref point P
F__x = opts.extForce(1); F__y = opts.extForce(2); F__z = opts.extForce(3); % ext force
M__x = opts.extTorque(1); M__y = opts.extTorque(2); M__z = opts.extTorque(3); % ext torque

%% Dynamic equations
dynamicRes = 0*v; %init with same type of x
carfm.dymlts.maple.dynamicResiduals;
dynamicRes = dynamicRes ./ rscale(:);

%% organise outputs
if nargout > 1
    % post-processing
    CPrr = 0*x(1:3); CPrl = CPrr; % init
    CPfl = CPrr; CPfr = CPrr; % init
    Wrr = CPrr; Wrl = CPrr; % init
    Wfl = CPrr; Wfr = CPrr; % init
    CA = CPrr; G = CPrr; II = 0*x(1:6); % init
    carfm.dymlts.maple.postProcessing;
    % dynamic results
    % overall vehicle
    output.speed       = V__P;
    output.yawRate     = yaw__rate;
    output.tangentialAcc = a__tP;
    output.normalAcc  = a__nP;
    output.longitudinalAcc = a__xP;
    output.lateralAcc  = a__yP;
    output.longitudinalSpeed = V__xP;
    output.lateralSpeed = V__yP;
    output.driftAngle  = lambda__P;
    output.driftRate   = v__lambda__P;
    output.rollAngle   = phi;
    output.rollRate    = v__phi;
    output.pitchAngle  = mu;
    output.pitchRate   = v__mu;
    output.heave       = z;
    output.heaveRate   = v__z;
    output.steerAngle  = delta;
    output.steerRate   = delta__dot;
    output.steerAcc    = v__delta__dot;
    output.centreOfMass = G(:);
    output.linVelocities = [V__x; V__y; V__z];
    output.linVelocityRates = [V__xdot; V__ydot; V__zdot];
    output.angVelocities = [Omega__x; Omega__y; Omega__z];
    output.rollInertia = II(1);
    output.pitchInertia = II(2);
    output.yawInertia = II(3);
    output.crossYawRollInertia = -II(4); % - b/c sign convenction
    output.crossYawPitchInertia = -II(5); % - b/c sign convenction
    output.crossRollPitchInertia = -II(6); % - b/c sign convenction
    % suspensions
    output.frontLeftTravel = -z__fl;
    output.frontLeftTravelRate = -z__fldot;
    output.frontLeftForce = F__fl;
    output.frontRightTravel = -z__fr;
    output.frontRightTravelRate = -z__frdot;
    output.frontRightForce = F__fr;
    output.rearLeftTravel = -z__rl;
    output.rearLeftTravelRate = -z__rldot;
    output.rearLeftForce = F__rl;
    output.rearRightTravel = -z__rr;
    output.rearRightTravelRate = -z__rrdot;
    output.rearRightForce = F__rr;
    % transmissions etc
    output.frontLeftTorque = Tau__fl;
    output.frontRightTorque = Tau__fr;
    output.rearLeftTorque = Tau__rl;
    output.rearRightTorque = Tau__rr;
    output.totalTorque   = Tau__t;
    output.gearboxRatio  = tau__g0;
    output.totalRatio    = tau__t;
    output.gyroInertia   = i__tg;
    output.rotaryInertia = i__ta;
    output.rotaryTorque  = i__ta*(v__omega__rr+v__omega__rl)*0.5;
    % ICE
    output.engineTorque     = Tau__td*abs(tau__t)/eta__t;
    output.engineTorqueWheel= Tau__td;
    output.engineMaxTorque  = Tau__p__max;
    output.engineMaxTorqueWheel = Tau__p__max/abs(tau__t);
    output.engineBrakeTorque = Tau__b;
    output.engineBrakeTorqueWheel = Tau__b/abs(tau__t);
    output.engineSpeed      = omegae;
    output.enginePower      = output.engineTorque*omegae;
    output.engineMaxPower   = Tau__p__max*omegae;
    % aero
    output.aeroDragForce   = aeroFD;
    output.aeroSideForce   = aeroFS;
    output.aeroLiftForce   = aeroFL;
    output.aeroRollMoment  = aeroMR;
    output.aeroPitchMoment = aeroMP;
    output.aeroYawMoment   = aeroMY;
    output.aeroPoint       = CA(:);

    % front left Tyre
    output.frontLeftTyre.angVelocity = omega__fl;
    output.frontLeftTyre.angAcceleration = v__omega__fl;
    output.frontLeftTyre.verticalDeformation = xi__fl;
    output.frontLeftTyre.verticalDeformationRate = xi__fldot;
    output.frontLeftTyre.N        = N__fl;
    output.frontLeftTyre.sideSlip = sa__fl;
    output.frontLeftTyre.camber   = ca__fl;
    output.frontLeftTyre.longSlip = ls__fl;
    output.frontLeftTyre.radius = rr__fl;
    output.frontLeftTyre.Fx = X__fl;
    output.frontLeftTyre.Fy = Y__fl;
    output.frontLeftTyre.Mx = T__flx;
    output.frontLeftTyre.My = T__fly;
    output.frontLeftTyre.Mz = T__flz;
    output.frontLeftTyre.contactPoint = CPfl(:);
    output.frontLeftTyre.contactPointVelocity = [VSfl0; VNfl0];
    output.frontLeftTyre.wheelCenter = Wfl(:);
    output.frontLeftTyre.tyreModelArgs = {N__fl_tyre,VSfl0,-VNfl0*(-opts.iTyreSide),-VRfl0,omega__fl,phi__fl*(-opts.iTyreSide),phit__fl*(-opts.iTyreSide)};
    % front right Tyre
    output.frontRightTyre.angVelocity = omega__fr;
    output.frontRightTyre.angAcceleration = v__omega__fr;
    output.frontRightTyre.verticalDeformation = xi__fr;
    output.frontRightTyre.verticalDeformationRate = xi__frdot;
    output.frontRightTyre.N        = N__fr;
    output.frontRightTyre.sideSlip = sa__fr;
    output.frontRightTyre.camber   = ca__fr;
    output.frontRightTyre.longSlip = ls__fr;
    output.frontRightTyre.radius = rr__fr;
    output.frontRightTyre.Fx = X__fr;
    output.frontRightTyre.Fy = Y__fr;
    output.frontRightTyre.Mx = T__frx;
    output.frontRightTyre.My = T__fry;
    output.frontRightTyre.Mz = T__frz;
    output.frontRightTyre.contactPoint = CPfr(:);
    output.frontRightTyre.contactPointVelocity = [VSfr0; VNfr0];
    output.frontRightTyre.wheelCenter = Wfr(:);
    output.frontRightTyre.tyreModelArgs = {N__fr_tyre,VSfr0,-VNfr0*(+opts.iTyreSide),-VRfr0,omega__fr,phi__fr*(+opts.iTyreSide),phit__fr*(+opts.iTyreSide)};
    % rear left Tyre
    output.rearLeftTyre.angVelocity = omega__rl;
    output.rearLeftTyre.angAcceleration = v__omega__rl;
    output.rearLeftTyre.verticalDeformation = xi__rl;
    output.rearLeftTyre.verticalDeformationRate = xi__rldot;
    output.rearLeftTyre.N        = N__rl;
    output.rearLeftTyre.sideSlip = sa__rl;
    output.rearLeftTyre.camber   = ca__rl;
    output.rearLeftTyre.longSlip = ls__rl;
    output.rearLeftTyre.radius = rr__rl;
    output.rearLeftTyre.Fx = X__rl;
    output.rearLeftTyre.Fy = Y__rl;
    output.rearLeftTyre.Mx = T__rlx;
    output.rearLeftTyre.My = T__rly;
    output.rearLeftTyre.Mz = T__rlz;
    output.rearLeftTyre.contactPoint = CPrl(:);
    output.rearLeftTyre.contactPointVelocity = [VSrl0; VNrl0];
    output.rearLeftTyre.wheelCenter = Wrl(:);
    output.rearLeftTyre.tyreModelArgs = {N__rl_tyre,VSrl0,-VNrl0*(-opts.iTyreSide),-VRrl0,omega__rl,phi__rl*(-opts.iTyreSide),phit__rl*(-opts.iTyreSide)};
    % rear right Tyre
    output.rearRightTyre.angVelocity = omega__rr;
    output.rearRightTyre.angAcceleration = v__omega__rr;
    output.rearRightTyre.verticalDeformation = xi__rr;
    output.rearRightTyre.verticalDeformationRate = xi__rrdot;
    output.rearRightTyre.N        = N__rr;
    output.rearRightTyre.sideSlip = sa__rr;
    output.rearRightTyre.camber   = ca__rr;
    output.rearRightTyre.longSlip = ls__rr;
    output.rearRightTyre.radius = rr__rr;
    output.rearRightTyre.Fx = X__rr;
    output.rearRightTyre.Fy = Y__rr;
    output.rearRightTyre.Mx = T__rrx;
    output.rearRightTyre.My = T__rry;
    output.rearRightTyre.Mz = T__rrz;
    output.rearRightTyre.contactPoint = CPrr(:);
    output.rearRightTyre.contactPointVelocity = [VSrr0; VNrr0];
    output.rearRightTyre.wheelCenter = Wrr(:);
    output.rearRightTyre.tyreModelArgs = {N__rr_tyre,VSrr0,-VNrr0*(+opts.iTyreSide),-VRrr0,omega__rr,phi__rr*(+opts.iTyreSide),phit__rr*(+opts.iTyreSide)};
    % variables and residual
    output.x         = x;
    output.v         = v;
    output.u         = u;
    output.r         = dynamicRes;
    output.residual  = norm(dynamicRes/numel(dynamicRes));
    % internal variables - not documented and used by end users
    % these variables are typically used to get the dynamical variables (see mltfm.dymlts.getDynVars)
    output.internal.kappa = [nan; nan; nan; nan]; % reserved for future usage
    output.internal.alpha = [nan; nan; nan; nan]; % reserved for future usage
    % exit code & msg
    if isnumeric(x) % numerical eval of this function
        % assume SSA solved
        output.exitcode = 0;
        output.exitmsg = 'Dynamics solved';
        % check cases
        if ~(output.residual<=opts.rtol) % check SSA solved (check also for nan x)
            output.exitcode = -1;
            output.exitmsg = 'Unable to solve dynamics';
        elseif output.engineTorque > output.engineMaxTorque  % check engine torque
            output.exitcode = 1;
            output.exitmsg = 'Dynamics solved but required driving torque is greater than available engine torque';
        elseif output.frontLeftTyre.N < 0 % check front load
            output.exitcode = 2;
            output.exitmsg = 'Dynamics solved but front left vertical load is negative';
        elseif output.frontRightTyre.N < 0 % check front load
            output.exitcode = 3;
            output.exitmsg = 'Dynamics solved but front right vertical load is negative';
        elseif output.rearLeftTyre.N < 0 % check rear load
            output.exitcode = 4;
            output.exitmsg = 'Dynamics solved but rear left vertical load is negative';
        elseif output.rearRightTyre.N < 0 % check rear load
            output.exitcode = 5;
            output.exitmsg = 'Dynamics solved but rear right vertical load is negative';
        end
    else % symbolic eval of this function
        output.exitcode = nan;
        output.exitmsg = 'none';
    end
end

return
