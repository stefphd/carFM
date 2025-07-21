% Unpack the car data

% double variables
g       = car.gravity;
w       = car.wheelbase;
t__f    = car.frontTrackWidth / 2; % half-track used
t__r    = car.rearTrackWidth / 2; % half-track used
R__f    = car.frontTyreUnloadedRadius;
R__r    = car.rearTyreUnloadedRadius;
d       = car.referencePointDistance;
m       = car.mass;
b       = car.cgB;
h       = car.cgH;
I__xx   = car.rollInertia;
I__zz   = car.yawInertia;
I__yy   = car.pitchInertia;
I__xz   = car.crossYawRollInertia;
m__fu   = car.frontUnsprungMass;
m__ru   = car.rearUnsprungMass;
i__fy   = car.frontWheelSpinInertia;
i__ry   = car.rearWheelSpinInertia;
i__ta   = car.transmissionInertia;
i__tg   = car.transmissionInertia;
eta__t  = car.transmissionEfficiency;
b__A    = car.aeroB;
h__A    = car.aeroH;
kr__f   = car.frontTyreStiffness;
cr__f   = car.frontTyreDamping;
kr__r   = car.rearTyreStiffness;
cr__r   = car.rearTyreDamping;
tau__g  = car.gearboxRatios(:); % column vector
omegar__s = car.gearboxSwitchingSpeeds(:); % column vector
k__df = car.frontDifferentialStiffness;
k__dr = car.rearDifferentialStiffness;
gamma__d = car.driveRatio;
gamma__b = car.brakeRatio;

% struct variables
aero = car.aero;
engine = car.engine;
frontSuspension = car.frontSuspension;
rearSuspension  = car.rearSuspension;
frontTyre = car.frontTyre;
rearTyre  = car.rearTyre;