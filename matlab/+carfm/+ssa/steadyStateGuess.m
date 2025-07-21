function x0 = steadyStateGuess(u_long, u_lat, car, opts)
%STEADYSTATEGUESS Guess for steady-state analysis
% 
% INPUT:
% u_long: speed, tangential acceleration, torque distribution, total torque (4-by-1 vector)
% u_lat: yaw_rate, steering angle, steering torque (5-by-1 vector)
% bike: bike data 
% opts: options
%
% OUTPUT:
% x0: scaled steady-state unknown guess (26-by-1 vector)

% Check args
arguments
    u_long (3,1)
    u_lat (3,1)
    car (1,1) struct
    opts (1,1) struct
end

%scaling 
xscale = opts.xscale;

% equilibrium point
V0          = u_long(1);
yaw__rate0  = u_lat(1);
aLong0      = u_long(2);
ay          = V0*yaw__rate0;

% params
carfm.common.unpackCar;
gz = opts.gz_g * g;

% HARDCODED
z = 0.05;
zi = -0.05;
mu = 0;
phi = 0;
lambda = 1e-6; % error if 0 (?)
delta = 0;

kappai = 0;

Nfl = gz*m*b/w;
Nfr = gz*m*b/w;
Nrl = gz*m*(w-b)/w;
Nrr = gz*m*(w-b)/w;

Fyfl = ay/gz*Nfl;
Fyfr = ay/gz*Nfr;
Fyrl = ay/gz*Nrl;
Fyrr = ay/gz*Nrr;

Fxfl = 0;
Fxfr = 0;
Fxrl = 0;
Fxrr = 0;

Taut = 0;

% from simplified susp eoms
Ffl = Nfl;
Ffr = Nfr;
Frl = Nrl;
Frr = Nrr;

% create guess vector
x0 = [phi
      mu
      z
      delta
      zi
      zi
      zi
      zi
      lambda
      kappai
      kappai
      kappai
      kappai
      Fxfl
      Fxfr
      Fxrl
      Fxrr
      Fyfl
      Fyfr
      Fyrl
      Fyrr
      Nfl
      Nfr
      Nrl
      Nrr
      Ffl
      Ffr
      Frl
      Frr
      Taut
      V0
      yaw__rate0
      aLong0
      ] ./ xscale(:);

end

