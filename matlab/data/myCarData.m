%% all data are referred to the FULLY EXTENDED configuration 

clear, clc

matfile = 'myCarData.mat'; % MAT file

fprintf('Preparing <%s>\n', matfile)
% SI units, unless specified differently
gravity = 9.806;

%% Overall geometry
wheelbase = 2.9;
frontTrackWidth = 2.016;
rearTrackWidth = 2.016;
% tyre
frontTyreUnloadedRadius = 0.30;
rearTyreUnloadedRadius = 0.30;
% reference point
referencePointDistance = wheelbase/2; % longitudinal distance of the reference point for vehicle tracking from the rear wheel

%% Inertia properties
% main body = full car
mass = 1300; % overall mass
cgH = 0.330; % CG height from the rear contact point
cgB = 1.535; % CG longitudinal distance from the rear contact point
% inertia wrt the CG
rollInertia = 500;
pitchInertia = 1500;
yawInertia = 1700;
crossYawRollInertia = 0;
% front unsprung (only one side)
frontUnsprungMass = 15;
frontWheelSpinInertia = 1.0;
% rear unsprung (only one side)
rearUnsprungMass = 20;
rearWheelSpinInertia = 1.3;

%% Transmission
% <transmission.Torques> definition and usage
% [Tfl,Tfr,Trl,Trr] = transmission.Torques(transmission, omegafl, omegafr, omegarl, omegarr, Td)
%
% INPUT
% transmission: structure containing transmission constants
% omegaij:      wheel angular velocities (ij=fl,fr,rl,rr) (rad/s)
% Td:           total driving torque, including engine brake, i.e. eventually
%               negative in coast-down (Nm)
%
% OUTPUT
% Tij:          wheel torque (ij=fl,fr,rl,rr) (Nm)

gearboxRatios = 1; % from 1st to highest gear - no gear ratios considered
% switching angular velocity of rear tyre in rad/s from gear k-th to (k+1)-th.
% The last value is the top angular velocity in rad/s.
gearboxSwitchingSpeeds = 1000;
transmissionInertia = 0; % neglected
transmissionEfficiency = 1.00;
brakeRatio = 0.58; % defined as front braking torque / total braking torque
% dataset
transmission.gammad = 0; % drive ratio, defined as front torque/total torque (0=RWD)
transmission.kd = 0; % differential stiffness (0=open) (Nms/rad)
% user-function that gives wheel torques
transmission.Torques = @basicTransmission;

%% Engine
% <engine.Torques> definition and usage
% [propulsionTorque, brakingTorque] = engine.Torques(engine, omega)
%
% INPUT
% engine: structure containing engine constants
% omega: engine angular velocity (rad/s)
%
% OUTPUT
% propulsionTorque: propulsion engine torque at the crankshaft
% brakingTorque: braking engine torque at the crankshaft

% dataset
engine.Pmax = 415e3;
engine.Pbrake = 0e3;
% user-function that gives engine torques
engine.Torques = @engineBasicModel;

%% Suspension
% <suspension.Force> definition and usage
% [Fl, Fr] = suspension.Force(susp, zl, zr, zldot, zrdot)
%
% INPUT
% susp: structure containining suspension constants
% zl:  left suspension travel (m)
% zr:  right suspension travel
%   zi = 0 fully extended
%   zi > 0 in compression
% zldot: left suspension travel rate 
% zrdot: right suspension travel rate 
%
% OUTPUT
% Fl: left suspension force (>0 when compressed, as for suspension travel)
% Fr: right suspension force (>0 when compressed, as for suspension travel)
%
% NOTE: end-stroke pads are included in the force definition

% <suspension.Kinematics> definition and usage
% [x,y,psi,phi,mu,Dx,Dy,Dpsi,Dphi,Dmu] = suspension.Kinematics(susp, z, delta)
%
% INPUT
% susp: structure containining suspension constants
% z:    suspension travel (m)
%   z = 0 fully extended
%   z > 0 in compression
% delta: steer angle, 0 for rear suspension (>0 clockwise) (rad)
%
% OUTPUT
% x: wheel center x-displacement (>0 forward)
% y: wheel center y-displacement (>0 rightward)
% psi: wheel yaw (>0 clockwise)
% phi: wheel camber (>0 clockwise)
% mu: wheel pitch
% Dx: dx/dz
% Dy: dy/dz
% Dpsi: dpsi/dz
% Dphi: dphi/dz
% Dmu: dmu/dz

% Front suspension
% dataset
frontSuspension.F0 = 0;
frontSuspension.k = 100e3;
frontSuspension.c = 5e3;
frontSuspension.ka = 70e3; % antiroll bar
% user-function that gives suspension force
frontSuspension.Force = @suspensionLinearModel;
% user-function that gives suspension kinematics
frontSuspension.Kinematics = @suspensionKinematics;

% Rear suspension
rearSuspension.F0 = 0;
rearSuspension.k = 90e3;
rearSuspension.c = 4e3;
rearSuspension.ka = 60e3; % antiroll bar
% user-function that gives suspension force
rearSuspension.Force = @suspensionLinearModel;
% user-function that gives suspension kinematics
rearSuspension.Kinematics = @suspensionKinematics;

%% Aerodynamics
% <aero.Forces> definition and usage
% [FD, FS, FL, MR, MP, MY] = aero.Forces(aero, V, ax, ay, az, mu, phi, z)
%
% INPUT
% aero: structure containing aero constants
% V:    vehicle speed (m/s)
% ax:   longitudinal acceleration (>0 for acceleration) (m/s2)
% ay:   lateral acceleration (>0 for right turn, i.e. SAE) (m/s2)
% az:   total vertical acceleration, including gravity (m/s2)
% mu:   pitch angle (>0 when accelerating, i.e. SAE) (rad)
% phi:  roll angle (>0 for right turn, i.e. SAE) (rad)
% z:    heave (>0 for compression, i.e. SAE) (m)
%
% OUTPUT
% FD:   drag force (N)
% FS:   side force (N)
% FL:   lift force (N)
% MR:   roll moment (Nm)
% MP:   pitch moment (Nm)
% MY:   yaw moment (Nm)
%
% NOTE: aerodynamic reference frame is
% - x axis pointing backward
% - y axis pointing rightward
% - z axis pointing upward
aeroH = 0;
aeroB = 0;
% dataset
aero.cda = 0.65;
aero.claf = -0.15;
aero.clar = -0.35;
aero.w = wheelbase;
aero.rhoa = 1.2;  % air density
% user-function that gives aero forces and torques
aero.Forces = @basicAeroModel;

%% Tyre forces
% <tyre.Forces> definition and usage
%[tyreFx,tyreFy,tyreMx,tyreMy,tyreMz,tyreLS,tyreSA] = tyre.Forces(tyre, N, Vs, Vn, Vr, Omega, ca, phit)
%
% INPUT
% tyre:     strcuture containing tyre constants
% N:        load (N)
% Vs:       tangent speed  (m/s)
% Vn:       normal speed   (m/s)
% Vr:       rolling speed  (m/s)
% Omega:    angular speed  (rad/s)
% ca:       camber angle   (rad)
% phit:     turn slip      (rad/m)
%
% OUTPUT
% tyreFx:   longitudinal force
% tyreFy:   lateral      force
% tyreMx:   overturning  moment
% tyreMy:   rolling      moment
% tyreMz:   aligning     moment
% tyreLS:   longitudinal slip
% tyreSA:   slip         angle

% Front tyre
% % MagicFormula_v6.2, data are read from *.tir file and converted into *.mat
% % NOTE: tyre geometry such as <TyreUnloadedRadius> must be specified at 
% % the beginning and are not exctracted from the *.tir file
% frontTyre.Forces = @tyreMF62Model;
% frontTyre = import_tir_file('Siemens_car205_60R15.tir', frontTyre); % save TIR data into frontTyre
% frontTyreStiffness = frontTyre.VERTICAL_STIFFNESS; % vertical
% frontTyreDamping = frontTyre.VERTICAL_DAMPING; % vertical - Not used in steady-state
% basic
frontTyre.Forces = @tyreMFBasicModel3;
frontTyre.KPUMIN = -1;
frontTyre.KPUMAX = +1;
frontTyre.ALPMIN = -1;
frontTyre.ALPMAX = +1;
frontTyre.FNOMIN = 3500;
frontTyre.PCX1 = 1.6935;
frontTyre.PDX1 = 1.8757;
frontTyre.PDX2 = -0.127;
frontTyre.PEX1 = 0.07708;
frontTyre.PKX1 = 30.5;
frontTyre.PKX3 = 0.2766;
frontTyre.PCY1 = 1.733;
frontTyre.PDY1 = 1.8217;
frontTyre.PDY2 = -0.43884;
frontTyre.PEY1 = 0.29446;
frontTyre.PKY1 = -44.2;
frontTyre.PKY2 = 2.5977;
frontTyre.LMUX = 0.93;
frontTyre.LMUY = 0.84;
frontTyre.LCX = 1;
frontTyre.LKX = 1;
frontTyre.LEX = 1;
frontTyre.LCY = 1;
frontTyre.LKY = 1;
frontTyre.LEY = 1;
frontTyreStiffness = 200e3;
frontTyreDamping = 150;

% Rear tyre
rearTyre = frontTyre; % identical
rearTyreStiffness = frontTyreStiffness;
rearTyreDamping = frontTyreDamping;

%% Save data
save(matfile)
