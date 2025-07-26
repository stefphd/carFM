function [steadyStateRes,output] = steadyStateEquations(x, u_long, u_lat, car, opts)
% STEADYSTATEEQUATIONS Steady-State equations
% 
% INPUT:
% x: scaled steady-state unknowns (33-by-1 vector)
% u_long: speed, tangential acceleration, total torque (3-by-1 vector)
% u_lat: yaw_rate, steering angle, drift angle (3-by-1 vector)
% car: car data
% opts: options
%
% OUTPUT:
% steadyStateRes: scaled residual of steady-state equations
% output: steady-state outputs (struct)

% Check args
arguments
    x (33,1)
    u_long (3,1)
    u_lat (3,1)
    car (1,1) struct
    opts (1,1) struct
end

%% Scaling
xscale = opts.xscale;
rscale = opts.rscale;

%% Model options
gz_g = opts.gz_g;
iTyreType = opts.iTyreType;
iSuspensionType = opts.iSuspensionType;
iEngineBrake = opts.iEngineBrake;
iRotaryInertia = opts.iRotaryInertia;
iAeroComps = opts.iAeroComps;
suspTrimRigid = opts.suspTrimRigid;
tyreDeformationRigid = opts.tyreDeformationRigid;

%% Steady state unknowns
phi0       = xscale(1) * x(1);
mu0        = xscale(2) * x(2);
z0         = xscale(3) * x(3);
delta0     = xscale(4) * x(4);
z__fl0     = xscale(5) * x(5);
z__fr0     = xscale(6) * x(6);
z__rl0     = xscale(7) * x(7);
z__rr0     = xscale(8) * x(8);
lambda__P0 = xscale(9) * x(9);
kappa__fl0 = xscale(10) * x(10);
kappa__fr0 = xscale(11) * x(11);
kappa__rl0 = xscale(12) * x(12);
kappa__rr0 = xscale(13) * x(13);
X__fl0     = xscale(14) * x(14);
X__fr0     = xscale(15) * x(15);
X__rl0     = xscale(16) * x(16);
X__rr0     = xscale(17) * x(17);
Y__fl0     = xscale(18) * x(18);
Y__fr0     = xscale(19) * x(19);
Y__rl0     = xscale(20) * x(20);
Y__rr0     = xscale(21) * x(21);
N__fl0     = xscale(22) * x(22);
N__fr0     = xscale(23) * x(23);
N__rl0     = xscale(24) * x(24);
N__rr0     = xscale(25) * x(25);
F__fl0     = xscale(26) * x(26);
F__fr0     = xscale(27) * x(27);
F__rl0     = xscale(28) * x(28);
F__rr0     = xscale(29) * x(29);
Tau__t0    = xscale(30) * x(30);
V__P0      = xscale(31) * x(31);
yaw__rate0 = xscale(32) * x(32);
a__t0      = xscale(33) * x(33);

%% steady state inputs
% longitudinal
u__V__P0 = u_long(1);
u__a__t0 = u_long(2);
u__Tau__t0 = u_long(3);
% lateral
u__yaw__rate0 = u_lat(1);
u__delta0 = u_lat(2);
u__lambda__P0 = u_lat(3);

%% Unpack car data
carfm.common.unpackCar;

%% Other params
g__x = 0;
g__y = 0; 
g__z = gz_g * g;

%% Saturates tyre loads for tyre eval
fsat_tyre = @(x,xl) xl*(x<xl) + x*(x>=xl);
N__fl0_tyre = fsat_tyre(N__fl0 / car.mass / car.gravity, opts.Neps) * car.mass * car.gravity;
N__fr0_tyre = fsat_tyre(N__fr0 / car.mass / car.gravity, opts.Neps) * car.mass * car.gravity; 
N__rl0_tyre = fsat_tyre(N__rl0 / car.mass / car.gravity, opts.Neps) * car.mass * car.gravity; 
N__rr0_tyre = fsat_tyre(N__rr0 / car.mass / car.gravity, opts.Neps) * car.mass * car.gravity;

%% Suspension kinematics
% travels are z__fl,z__fr,z__rl,z__rr (>0 in extension), while user defines
% suspension kinematics using travel positive in compression: need sign changes
% Front
[x__fl, y__fl, psi__fl, phi__fl, mu__fl, ...
    D__x__fl, D__y__fl, D__psi__fl, D__phi__fl, D__mu__fl] = frontSuspension.Kinematics(frontSuspension, -z__fl0, delta0);
[x__fr, y__fr, psi__fr, phi__fr, mu__fr, ...
    D__x__fr, D__y__fr, D__psi__fr, D__phi__fr, D__mu__fr] = frontSuspension.Kinematics(frontSuspension, -z__fr0, delta0);

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
    D__x__rl, D__y__rl, D__psi__rl, D__phi__rl, D__mu__rl] = rearSuspension.Kinematics(rearSuspension, -z__rl0, 0); % zero delta for rear
[x__rr, y__rr, psi__rr, phi__rr, mu__rr, ...
    D__x__rr, D__y__rr, D__psi__rr, D__phi__rr, D__mu__rr] = rearSuspension.Kinematics(rearSuspension, -z__rr0, 0); % zero delta for rear

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
carfm.ssa.maple.rearTyreKinematics; % This is generated from MAPLE

% Left
[rlTyreFx,rlTyreFy,T__rlx,T__rly,T__rlz,ls__rl,sa__rl] = rearTyre.Forces(rearTyre,N__rl0_tyre, ...
    VSrl0,VNrl0*opts.iTyreSide,-VRrl0,omega__rl0, ...
    -ca__rl*opts.iTyreSide,-phit__rl*opts.iTyreSide); 

% Right
[rrTyreFx,rrTyreFy,T__rrx,T__rry,T__rrz,ls__rr,sa__rr] = rearTyre.Forces(rearTyre,N__rr0_tyre, ...
    VSrr0,-VNrr0*opts.iTyreSide,-VRrr0,omega__rr0, ...
    ca__rr*opts.iTyreSide,phit__rr*opts.iTyreSide); 

% Fix left/right signs, depending on opts.iTyreSide
rlTyreFy = -rlTyreFy*opts.iTyreSide;
T__rlz = -T__rlz*opts.iTyreSide;
T__rlx = -T__rlx*opts.iTyreSide;
rrTyreFy = rrTyreFy*opts.iTyreSide;
T__rrz = T__rrz*opts.iTyreSide;
T__rrx = T__rrx*opts.iTyreSide;

% ISO to adapted SAE
rrTyreFy = -rrTyreFy;
T__rry = -T__rry;
T__rrz = -T__rrz;
rlTyreFy = -rlTyreFy;
T__rly = -T__rly;
T__rlz = -T__rlz;

%% Front tyres
carfm.ssa.maple.frontTyreKinematics; % This is generated from MAPLE

% Left
[flTyreFx,flTyreFy,T__flx,T__fly,T__flz,ls__fl,sa__fl] = frontTyre.Forces(frontTyre,N__fl0_tyre, ...
    VSfl0,VNfl0*opts.iTyreSide,-VRfl0,omega__fl0, ...
    -ca__fl*opts.iTyreSide,-phit__fl*opts.iTyreSide); 

% Right
[frTyreFx,frTyreFy,T__frx,T__fry,T__frz,ls__fr,sa__fr] = frontTyre.Forces(frontTyre,N__fr0_tyre, ...
    VSfr0,-VNfr0*opts.iTyreSide,-VRfr0,omega__fr0, ...
    ca__fr*opts.iTyreSide,phit__fr*opts.iTyreSide); 

% Fix left/right signs, depending on opts.iTyreSide
flTyreFy = -flTyreFy*opts.iTyreSide;
T__flz = -T__flz*opts.iTyreSide;
T__flx = -T__flx*opts.iTyreSide;
frTyreFy = frTyreFy*opts.iTyreSide;
T__frz = T__frz*opts.iTyreSide;
T__frx = T__frx*opts.iTyreSide;

% ISO to adapted SAE
flTyreFy = -flTyreFy;
T__fly = -T__fly;
T__flz = -T__flz;
frTyreFy = -frTyreFy;
T__fry = -T__fry;
T__frz = -T__frz;

%% Aerodynamic forces
[aeroFD, aeroFS, aeroFL, aeroMR, aeroMP, aeroMY] = aero.Forces(aero, V__P0, ...
    a__t0, V__P0*yaw__rate0, g__z, ...
    mu0, phi0, z0);
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
[flSuspensionF, frSuspensionF] = frontSuspension.Force(frontSuspension, -z__fl0, -z__fr0, -0, -0);
% rear
[rlSuspensionF, rrSuspensionF] = rearSuspension.Force(rearSuspension, -z__rl0, -z__rr0, -0, -0);

%% Gear ratio
omega__r0 = 0.5*(omega__rr0 + omega__rl0); % use the average between rear wheel speeds
omega__dotr0 = 0.5*(omega__dotrr0 + omega__dotrl0); % same for ang acc
if numel(tau__g)>1
    % eq. (27) of https://doi.org/10.1080/00423114.2021.1910718
    tau__g0 = tau__g(1) + sum(diff(tau__g)/2.*(1+sin(atan(opts.gearSharpness*(omega__r0-omegar__s(1:end-1))))),1);
else % only one gear ratio (above equation fails)
    tau__g0 = tau__g;
end
% total ratio from engine to wheel
tau__t = tau__g0; % no other ratio considered, total is gearbox!

%% Engine
omegae = omega__r0/abs(tau__t);
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
nfl = N__fl0 / car.mass / car.gravity;
nfr = N__fr0 / car.mass / car.gravity;
nrl = N__rl0 / car.mass / car.gravity;
nrr = N__rr0 / car.mass / car.gravity;
Tau__b = Tau__b * fsat(0.5*(nrr+nrl), 5*opts.Neps, 10*opts.Neps);
omega__dotrr0 = omega__dotrr0 * fsat(nrr, 5*opts.Neps, 10*opts.Neps);
omega__dotrl0 = omega__dotrl0 * fsat(nrl, 5*opts.Neps, 10*opts.Neps);
omega__dotfr0 = omega__dotfr0 * fsat(nfr, 5*opts.Neps, 10*opts.Neps);
omega__dotfl0 = omega__dotfl0 * fsat(nfl, 5*opts.Neps, 10*opts.Neps);

%% Transmission
% NOTE: transmission inertia is removed in EOM
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
Tau__tb = ( Tau__t0 - sqrt(Tau__t0^2 + Taueps^2))/2; % negative part
Tau__td = ( Tau__t0 + sqrt(Tau__t0^2 + Taueps^2))/2 - Tau__b*eta__t/abs(tau__t); % positive part
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
    omega__fl0, omega__fr0, omega__rl0, omega__rr0, Tau__td);
% Total = brake + drive
Tau__fl0 = Tau__flb + Tau__fld;
Tau__fr0 = Tau__frb + Tau__frd;
Tau__rl0 = Tau__rlb + Tau__rld;
Tau__rr0 = Tau__rrb + Tau__rrd;

%% External force and torque
% ext force applied to the ref point P
F__x = opts.extForce(1); F__y = opts.extForce(2); F__z = opts.extForce(3); % ext force
M__x = opts.extTorque(1); M__y = opts.extTorque(2); M__z = opts.extTorque(3); % ext torque

%% Steady state equations
% Tyre equations
%tyre.iType = 1 REAL tyres,   tyre.iType = 0 IDEAL tyres (no deformation, no slip..)
tyreRes = 0*x(1:12); %init with same type of x
tyreRes(1) = iTyreType(1) * (N__fl0 - kr__f*xi__fl0) ...
               + (1-iTyreType(1)) * (xi__fl0 - tyreDeformationRigid(1));
tyreRes(2) = iTyreType(2) * (Y__fl0 - flTyreFy) ...
               + (1-iTyreType(2)) * sa__fl;
tyreRes(3) = iTyreType(3) * (X__fl0 - flTyreFx) ...  
               + (1-iTyreType(3)) * ls__fl;
tyreRes(4) = iTyreType(1) * (N__fr0 - kr__f*xi__fr0) ...
               + (1-iTyreType(1)) * (xi__fr0 - tyreDeformationRigid(2));
tyreRes(5) = iTyreType(2) * (Y__fr0 - frTyreFy) ...
               + (1-iTyreType(2)) * sa__fr;
tyreRes(6) = iTyreType(3) * (X__fr0 - frTyreFx) ...  
               + (1-iTyreType(3)) * ls__fr;
tyreRes(7) = iTyreType(1) * (N__rl0 - kr__r*xi__rl0) ...
               + (1-iTyreType(1)) * (xi__rl0 - tyreDeformationRigid(3));
tyreRes(8) = iTyreType(2) * (Y__rl0 - rlTyreFy) ...
               + (1-iTyreType(2)) * sa__rl;
tyreRes(9) = iTyreType(3) * (X__rl0 - rlTyreFx) ...  
               + (1-iTyreType(3)) * ls__rl;
tyreRes(10) = iTyreType(1) * (N__rr0 - kr__r*xi__rr0) ...
               + (1-iTyreType(1)) * (xi__rr0 - tyreDeformationRigid(4));
tyreRes(11) = iTyreType(2) * (Y__rr0 - rrTyreFy) ...
               + (1-iTyreType(2)) * sa__rr;
tyreRes(12) = iTyreType(3) * (X__rr0 - rrTyreFx) ...  
               + (1-iTyreType(3)) * ls__rr;
% Suspension equations
suspRes = 0*x(1:4); %init with same type of x
suspRes(1) = iSuspensionType * (F__fl0 - flSuspensionF) ...
                + (1-iSuspensionType) * (phi0 - suspTrimRigid(1)); 
suspRes(2) = iSuspensionType * (F__fr0 - frSuspensionF) ...
                + (1-iSuspensionType) * (mu0 - suspTrimRigid(2));  
suspRes(3) = iSuspensionType * (F__rl0 - rlSuspensionF) ...
                + (1-iSuspensionType) * (z0 - suspTrimRigid(3));  
suspRes(4) = iSuspensionType * (F__rr0 - rrSuspensionF) ...
                + (1-iSuspensionType) * ( (F__fl0-F__fr0) - ((F__fl0-F__fr0) + (F__rl0-F__rr0))*suspTrimRigid(4));     
% Equilibrium equations
steadyStateRes = 0*x(1:14); %init with same type of x
carfm.ssa.maple.steadyStateResiduals; % This is generated from MAPLE
% Cat steadyStateRes
steadyStateRes = [tyreRes; suspRes; steadyStateRes];
% Scale equations
steadyStateRes(1)  = iTyreType(1) * steadyStateRes(1) /rscale(1) + (1-iTyreType(1)) * steadyStateRes(1) /1;
steadyStateRes(2)  = iTyreType(2) * steadyStateRes(2) /rscale(2) + (1-iTyreType(2)) * steadyStateRes(2) /1;
steadyStateRes(3)  = iTyreType(3) * steadyStateRes(3) /rscale(3) + (1-iTyreType(3)) * steadyStateRes(3) /1;
steadyStateRes(4)  = iTyreType(1) * steadyStateRes(4) /rscale(4) + (1-iTyreType(1)) * steadyStateRes(4) /1;
steadyStateRes(5)  = iTyreType(2) * steadyStateRes(5) /rscale(5) + (1-iTyreType(2)) * steadyStateRes(5) /1;
steadyStateRes(6)  = iTyreType(3) * steadyStateRes(6) /rscale(6) + (1-iTyreType(3)) * steadyStateRes(6) /1;
steadyStateRes(7)  = iTyreType(1) * steadyStateRes(7) /rscale(1) + (1-iTyreType(1)) * steadyStateRes(7) /1;
steadyStateRes(8)  = iTyreType(2) * steadyStateRes(8) /rscale(2) + (1-iTyreType(2)) * steadyStateRes(8) /1;
steadyStateRes(9)  = iTyreType(3) * steadyStateRes(9) /rscale(3) + (1-iTyreType(3)) * steadyStateRes(9) /1;
steadyStateRes(10) = iTyreType(1) * steadyStateRes(10)/rscale(4) + (1-iTyreType(1)) * steadyStateRes(10)/1;
steadyStateRes(11) = iTyreType(2) * steadyStateRes(11)/rscale(5) + (1-iTyreType(2)) * steadyStateRes(11)/1;
steadyStateRes(12) = iTyreType(3) * steadyStateRes(12)/rscale(6) + (1-iTyreType(3)) * steadyStateRes(12)/1;
steadyStateRes(13) = iSuspensionType * steadyStateRes(13)/rscale(7) + (1-iSuspensionType) * steadyStateRes(13)/1;
steadyStateRes(14) = iSuspensionType * steadyStateRes(14)/rscale(8) + (1-iSuspensionType) * steadyStateRes(14)/1;
steadyStateRes(15) = iSuspensionType * steadyStateRes(15)/rscale(7) + (1-iSuspensionType) * steadyStateRes(15)/1;
steadyStateRes(16) = iSuspensionType * steadyStateRes(16)/rscale(8) + (1-iSuspensionType) * steadyStateRes(16)/1;
steadyStateRes(17) = steadyStateRes(17)/rscale(17);
steadyStateRes(18) = steadyStateRes(18)/rscale(18);
steadyStateRes(19) = steadyStateRes(19)/rscale(19);
steadyStateRes(20) = steadyStateRes(20)/rscale(20);
steadyStateRes(21) = steadyStateRes(21)/rscale(21);
steadyStateRes(22) = steadyStateRes(22)/rscale(22);
steadyStateRes(23) = steadyStateRes(23)/rscale(23);
steadyStateRes(24) = steadyStateRes(24)/rscale(24);
steadyStateRes(25) = steadyStateRes(25)/rscale(25);
steadyStateRes(26) = steadyStateRes(26)/rscale(26);
steadyStateRes(27) = steadyStateRes(27)/rscale(27);
steadyStateRes(28) = steadyStateRes(28)/rscale(28);
steadyStateRes(29) = steadyStateRes(29)/rscale(29);
steadyStateRes(30) = steadyStateRes(30)/rscale(30);
% Set long active inputs
activeLongInputRes = 0*x(1:3); %init with same type of x
activeLongInputRes(1) = (V__P0 - u__V__P0) / rscale(31);
activeLongInputRes(2) = (a__t0 - u__a__t0) / rscale(32);
activeLongInputRes(3) = (Tau__t0 - u__Tau__t0) / rscale(33);
activeLongInputRes = activeLongInputRes(find(opts.ssActiveLongInputs)); % take only active inputs
% Set lat active inputs
activeLatInputRes = 0*x(1:3); %init with same type of x
activeLatInputRes(1) = (yaw__rate0 - u__yaw__rate0) / rscale(34);
activeLatInputRes(2) = (delta0 - u__delta0) / rscale(35);
activeLatInputRes(3) = (lambda__P0 - u__lambda__P0) / rscale(36);
activeLatInputRes = activeLatInputRes(find(opts.ssActiveLatInputs)); % take only active inputs
% Cat active input res
steadyStateRes = [steadyStateRes; activeLongInputRes; activeLatInputRes]; 

%% organise outputs
if nargout > 1
    % post-processing
    CPrr = 0*x(1:3); CPrl = CPrr; % init
    CPfl = CPrr; CPfr = CPrr; % init
    Wrr = CPrr; Wrl = CPrr; % init
    Wfl = CPrr; Wfr = CPrr; % init
    CA = CPrr; G = CPrr; II = 0*x(1:6); % init
    carfm.ssa.maple.postProcessing;
    % ss results
    % overall vehicle
    output.speed       = V__P0;
    output.yawRate     = yaw__rate0;
    output.tangentialAcc = a__tP;
    output.normalAcc  = a__nP;
    output.longitudinalAcc = a__xP;
    output.lateralAcc  = a__yP;
    output.longitudinalSpeed = V__P0*cos(lambda__P0);
    output.lateralSpeed = V__P0*sin(lambda__P0);
    output.driftAngle  = lambda__P0;
    output.driftRate   = 0;
    output.rollAngle   = phi0;
    output.rollRate    = 0;
    output.pitchAngle  = mu0;
    output.pitchRate   = 0;
    output.heave       = z0;
    output.heaveRate   = 0;
    output.steerAngle  = delta0;
    output.steerRate   = 0;
    output.steerAcc    = 0;
    output.centreOfMass = G(:);
    output.linVelocities = [V__x0; V__y0; V__z0];
    output.linVelocityRates = [V__dotx0; V__doty0; V__dotz0];
    output.angVelocities = [Omega__x0; Omega__y0; Omega__z0];
    output.rollInertia = II(1);
    output.pitchInertia = II(2);
    output.yawInertia = II(3);
    output.crossYawRollInertia = -II(4); % - b/c sign convenction
    output.crossYawPitchInertia = -II(5); % - b/c sign convenction
    output.crossRollPitchInertia = -II(6); % - b/c sign convenction
    % suspensions
    output.frontLeftTravel = -z__fl0;
    output.frontLeftTravelRate = -0;
    output.frontLeftForce = F__fl0;
    output.frontRightTravel = -z__fr0;
    output.frontRightTravelRate = -0;
    output.frontRightForce = F__fr0;
    output.rearLeftTravel = -z__rl0;
    output.rearLeftTravelRate = -0;
    output.rearLeftForce = F__rl0;
    output.rearRightTravel = -z__rr0;
    output.rearRightTravelRate = -0;
    output.rearRightForce = F__rr0;
    % transmissions etc
    output.frontLeftTorque = Tau__fl0;
    output.frontRightTorque = Tau__fr0;
    output.rearLeftTorque = Tau__rl0;
    output.rearRightTorque = Tau__rr0;
    output.totalTorque   = Tau__t0;
    output.gearboxRatio  = tau__g0;
    output.totalRatio    = tau__t;
    output.gyroInertia   = i__tg;
    output.rotaryInertia = i__ta;
    output.rotaryTorque  = i__ta*omega__dotr0;
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
    output.frontLeftTyre.angVelocity = omega__fl0;
    output.frontLeftTyre.angAcceleration = omega__dotfl0;
    output.frontLeftTyre.verticalDeformation = xi__fl0;
    output.frontLeftTyre.verticalDeformationRate = 0;
    output.frontLeftTyre.N        = N__fl0;
    output.frontLeftTyre.sideSlip = sa__fl;
    output.frontLeftTyre.camber   = ca__fl;
    output.frontLeftTyre.longSlip = ls__fl;
    output.frontLeftTyre.radius = r0__fl;
    output.frontLeftTyre.Fx = X__fl0;
    output.frontLeftTyre.Fy = Y__fl0;
    output.frontLeftTyre.Mx = T__flx;
    output.frontLeftTyre.My = T__fly;
    output.frontLeftTyre.Mz = T__flz;
    output.frontLeftTyre.contactPoint = CPfl(:);
    output.frontLeftTyre.contactPointVelocity = [VSfl0; VNfl0];
    output.frontLeftTyre.wheelCenter = Wfl(:);
    output.frontLeftTyre.tyreModelArgs = {N__fl0_tyre,VSfl0,-VNfl0*(-opts.iTyreSide),-VRfl0,omega__fl0,phi__fl*(-opts.iTyreSide),phit__fl*(-opts.iTyreSide)};
    % front right Tyre
    output.frontRightTyre.angVelocity = omega__fr0;
    output.frontRightTyre.angAcceleration = omega__dotfr0;
    output.frontRightTyre.verticalDeformation = xi__fr0;
    output.frontRightTyre.verticalDeformationRate = 0;
    output.frontRightTyre.N        = N__fr0;
    output.frontRightTyre.sideSlip = sa__fr;
    output.frontRightTyre.camber   = ca__fr;
    output.frontRightTyre.longSlip = ls__fr;
    output.frontRightTyre.radius = r0__fr;
    output.frontRightTyre.Fx = X__fr0;
    output.frontRightTyre.Fy = Y__fr0;
    output.frontRightTyre.Mx = T__frx;
    output.frontRightTyre.My = T__fry;
    output.frontRightTyre.Mz = T__frz;
    output.frontRightTyre.contactPoint = CPfr(:);
    output.frontRightTyre.contactPointVelocity = [VSfr0; VNfr0];
    output.frontRightTyre.wheelCenter = Wfr(:);
    output.frontRightTyre.tyreModelArgs = {N__fr0_tyre,VSfr0,-VNfr0*(+opts.iTyreSide),-VRfr0,omega__fr0,phi__fr*(+opts.iTyreSide),phit__fr*(+opts.iTyreSide)};
    % rear left Tyre
    output.rearLeftTyre.angVelocity = omega__rl0;
    output.rearLeftTyre.angAcceleration = omega__dotrl0;
    output.rearLeftTyre.verticalDeformation = xi__rl0;
    output.rearLeftTyre.verticalDeformationRate = 0;
    output.rearLeftTyre.N        = N__rl0;
    output.rearLeftTyre.sideSlip = sa__rl;
    output.rearLeftTyre.camber   = ca__rl;
    output.rearLeftTyre.longSlip = ls__rl;
    output.rearLeftTyre.radius = r0__rl;
    output.rearLeftTyre.Fx = X__rl0;
    output.rearLeftTyre.Fy = Y__rl0;
    output.rearLeftTyre.Mx = T__rlx;
    output.rearLeftTyre.My = T__rly;
    output.rearLeftTyre.Mz = T__rlz;
    output.rearLeftTyre.contactPoint = CPrl(:);
    output.rearLeftTyre.contactPointVelocity = [VSrl0; VNrl0];
    output.rearLeftTyre.wheelCenter = Wrl(:);
    output.rearLeftTyre.tyreModelArgs = {N__rl0_tyre,VSrl0,-VNrl0*(-opts.iTyreSide),-VRrl0,omega__rl0,phi__rl*(-opts.iTyreSide),phit__rl*(-opts.iTyreSide)};
    % rear right Tyre
    output.rearRightTyre.angVelocity = omega__rr0;
    output.rearRightTyre.angAcceleration = omega__dotrr0;
    output.rearRightTyre.verticalDeformation = xi__rr0;
    output.rearRightTyre.verticalDeformationRate = 0;
    output.rearRightTyre.N        = N__rr0;
    output.rearRightTyre.sideSlip = sa__rr;
    output.rearRightTyre.camber   = ca__rr;
    output.rearRightTyre.longSlip = ls__rr;
    output.rearRightTyre.radius = r0__rr;
    output.rearRightTyre.Fx = X__rr0;
    output.rearRightTyre.Fy = Y__rr0;
    output.rearRightTyre.Mx = T__rrx;
    output.rearRightTyre.My = T__rry;
    output.rearRightTyre.Mz = T__rrz;
    output.rearRightTyre.contactPoint = CPrr(:);
    output.rearRightTyre.contactPointVelocity = [VSrr0; VNrr0];
    output.rearRightTyre.wheelCenter = Wrr(:);
    output.rearRightTyre.tyreModelArgs = {N__rr0_tyre,VSrr0,-VNrr0*(+opts.iTyreSide),-VRrr0,omega__rr0,phi__rr*(+opts.iTyreSide),phit__rr*(+opts.iTyreSide)};
    % variables and residual
    output.xss         = x;
    output.rss         = steadyStateRes;
    output.residual    = norm(steadyStateRes/numel(steadyStateRes));
    % internal variables - not documented and used by end users
    % these variables are typically used to get the dynamical variables (see mltfm.dymlts.getDynVars)
    output.internal.kappa = [kappa__fl0; kappa__fr0; kappa__rl0; kappa__rr0];
    output.internal.alpha = [alpha__fl0; alpha__fr0; alpha__rl0; alpha__rr0];
    % steadyState.ssEqnArgs   = {x, u_long, u_lat, opts};
    % exit code & msg
    if isnumeric(x) % numerical eval of this function
        % assume SSA solved
        output.exitcode = 0;
        output.exitmsg = 'SSA solved';
        % check cases
        if ~(output.residual<=opts.rtol) % check SSA solved (check also for nan x)
            output.exitcode = -1;
            output.exitmsg = 'Unable to solve SSA';
        elseif output.engineTorque > output.engineMaxTorque  % check engine torque
            output.exitcode = 1;
            output.exitmsg = 'SSA solved but required driving torque is greater than available engine torque';
        elseif output.frontLeftTyre.N < 0 % check front load
            output.exitcode = 2;
            output.exitmsg = 'SSA solved but front left vertical load is negative';
        elseif output.frontRightTyre.N < 0 % check front load
            output.exitcode = 3;
            output.exitmsg = 'SSA solved but front right vertical load is negative';
        elseif output.rearLeftTyre.N < 0 % check rear load
            output.exitcode = 4;
            output.exitmsg = 'SSA solved but rear left vertical load is negative';
        elseif output.rearRightTyre.N < 0 % check rear load
            output.exitcode = 5;
            output.exitmsg = 'SSA solved but rear right vertical load is negative';
        end
    else % symbolic eval of this function
        output.exitcode = nan;
        output.exitmsg = 'none';
    end
end


return
