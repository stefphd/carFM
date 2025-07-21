%% Test the bikeData models
clc, clear

% data
car = load('myCarData.mat');

% tyres
N     = 1000;   % tyre load
VS0   = 90;     % tangent speed (m/s)
VN0   = 0;      % normal speed (m/s)
VR0   = 89;     % rolling speed (m/s)
ca0   = 0;      % camber angle (rad)
phit0 = 0;      % turn slip (rad/m)
[tyreFxf,tyreFyf] = car.frontTyre.Forces(car.frontTyre,N,VS0,VN0,VR0,[],ca0,phit0);
[tyreFxr,tyreFyr] = car.rearTyre.Forces(car.rearTyre,N,VS0,VN0,VR0,[],ca0,phit0);

% aero
V = 50; %speed
ax = 0;
ay = 0;
[FD, FS, FL, MR, MP, MY] = car.aero.Forces(car.aero, V, ax, ay, 0, 0, 0, 0);

% suspensions
zf = 0.1;
zr = 0;
frontForce = car.frontSuspension.Force(car.frontSuspension, zf, zf, 0, 0);
rearForce = car.rearSuspension.Force(car.rearSuspension, zr, zr, 0, 0);
car.frontSuspension.Kinematics(car.frontSuspension, zf);
car.rearSuspension.Kinematics(car.rearSuspension, zr);

% engine
V = linspace(10, 100, 500)';
omegar = V/car.rearTyreUnloadedRadius;
gearboxRatio = car.gearboxRatios(1) + sum(diff(car.gearboxRatios)/2.*(1+sin(atan(5*(omegar-car.gearboxSwitchingSpeeds(1:end-1))))),2);
omegae = omegar./gearboxRatio;
[propulsionTorque, brakingTorque]= car.engine.Torques(car.engine, omegae);
propulsionForce = propulsionTorque./gearboxRatio/car.rearTyreUnloadedRadius;
brakingForce = brakingTorque./gearboxRatio*car.rearTyreUnloadedRadius;
propulsionPower = propulsionForce.*V;