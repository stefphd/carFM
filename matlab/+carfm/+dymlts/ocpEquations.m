function [f, c, l, data] = ocpEquations(t,x,xp,u,aux,opts)
%OCPEQUATIONS Dynamic and path constraints and Lagrange cost of the OCP.
% 
% INPUT
% t: indepdent variable
% x: states
% xp: state sparial derivative
% u: control 
% aux: auxdata
% opts: options
%
% OUTPUT
% f: f(t,x,xp,u)=0
% c: path constraints
% l: Lagrange (i.e. running) cost integrands
% data: other output data for post-processing

%% Get unscaled data
% independent variable
s = t*opts.sscale;
% states
% x = [ phi, mu, z, delta,
%       z__fl, z__fr, z__rl, z__rr, 
%       V__P, lambda__P, V__z, 
%       delta__dot, z__fldot, z__frdot, z__rldot, z__rrdot, 
%       Omega__z, Omega__x, Omega__y, 
%       omega__fl, omega__fr, omega__rl, omega__rr
%       n, chi,
%       delta__ddot, Tau__t]
n = x(24,:)*opts.xscale(24); % lateral position
chi = x(25,:)*opts.xscale(25); % relative yaw orientation
V = x(9,:)*opts.xscale(9); % total speed
lambda = x(10,:)*opts.xscale(10); % sideslip
% controls
% u = [delta__dddot, Tau__tdot]

%% Eval track
Omegaz = aux.track.Omegaz(s); % geodesic
rwr = aux.track.rwr(s); % right half-width
rwl = aux.track.rwl(s); % left half-width
% This is to simplify the generated expressions in CASADI in the case of 2D track
if aux.track.isTrack3D
    % 3D track
    Omegax = aux.track.Omegax(s); % normal
    Omegay = aux.track.Omegay(s); % torsion
    mu = aux.track.mu(s); % road pitch
    phi = aux.track.phi(s); % road banking
else
    % 2D track
    Omegax = 0*t;
    Omegay = 0*t;
    mu = 0*t;
    phi = 0*t;
end
% This is to convert CasADi DM vars to standard double when input args are
% numeric (i.e. numeric eval of this function)
if isnumeric(x)
    rwr = full(rwr);
    rwl = full(rwl);
    mu = full(mu);
    phi = full(phi);
    Omegax = full(Omegax);
    Omegay = full(Omegay);
    Omegaz = full(Omegaz);
end

%% Road
Vx = V.*cos(lambda); 
Vy = V.*sin(lambda);
sdot = (Vx.*cos(chi)-Vy.*sin(chi)) ./ (1 - n.*Omegaz);
zetap = V./sdot;
% Set g__x,g__y,g__z - valid also for 2D
gravityFactors = 0*x(1:3,:);
gravityFactors(1,:) =-sin(mu).*cos(chi) + cos(mu).*sin(phi).*sin(chi);
gravityFactors(2,:) = sin(mu).*sin(chi) + cos(mu).*sin(phi).*cos(chi);
gravityFactors(3,:) = cos(mu).*cos(phi);
% Set omega__x,omega__y - valid also for 2D
angVelocities = 0*x(1:2,:);
angAccelerations = 0*x(1:2,:);
angVelocities(1,:) = (Omegax.*cos(chi)+Omegay.*sin(chi)).*sdot;
angVelocities(2,:) = (Omegay.*cos(chi)-Omegax.*sin(chi)).*sdot;
angAccelerations(1,:) = 0*t; % assumed negligible
angAccelerations(2,:) = 0*t; % assumed negligible
% Set v__w
zVelocity = 0*t; % assumed negligible
zVelocityRate = 0*t; % assumed negligible

%% Model equations
% Car model
% x1 = [ phi, mu, z, delta,
%       z__fl, z__fr, z__rl, z__rr, 
%       V__P, lambda__P, V__z, 
%       delta__dot, z__fldot, z__frdot, z__rldot, z__rrdot, 
%       Omega__z, Omega__x, Omega__y, 
%       omega__fl, omega__fr, omega__rl, omega__rr
% ]
x1 = x(1:23,:); % car states
xp1 = xp(1:23,:); % car state spatial derivative
v1 = xp1.*sdot/opts.sscale; % time derivative dx/dt=dx/ds*ds/dt
% u1 = [delta__ddot, Tau__t]
u1 = x(26:27,:); % car controls
f1 = 0*x1; % init
for k = numel(t) : -1 : 1 % Eval all t, from the end to init dyout at the first call
    % Set road opts
    opts.gravityFactors = gravityFactors(:,k);
    opts.angVelocities = angVelocities(:,k);
    opts.angAccelerations = angAccelerations(:,k);
    opts.zVelocity = zVelocity(k);
    opts.zVelocityRate = zVelocityRate(k);
    % Call to carfm.dymlts.dynamicEquations
    [f1(:,k), dyout(k)] = carfm.dymlts.dynamicEquations(x1(:,k), v1(:,k), u1(:,k), aux.car, opts);
end
f1 = f1./sdot*opts.sscale;

% Tracking equations
ndot = Vx.*sin(chi)+Vy.*cos(chi); % dn/dt
chidot = [dyout.yawRate] - Omegaz.*sdot; % dchi/dt
f2 = [  xp(24,:) - ndot./sdot*opts.sscale/opts.xscale(24);
        xp(25,:) - chidot./sdot*opts.sscale/opts.xscale(25)];

% Control itegration chains
f3 = xp(26:27,:)-u.*opts.upscale(:)./opts.uscale(:)*opts.sscale;

% CAT differential constraints
f = [f1;f2;f3] ./ opts.fscale(:);

% Path constraints
frontLeftTyre = [dyout.frontLeftTyre];
frontRightTyre = [dyout.frontRightTyre];
rearLeftTyre = [dyout.rearLeftTyre];
rearRightTyre = [dyout.rearRightTyre];
N__fl = [frontLeftTyre.N];
N__fr = [frontRightTyre.N];
N__rl = [rearLeftTyre.N];
N__rr = [rearRightTyre.N];
T__e = [dyout.engineTorque];
T__emax = [dyout.engineMaxTorque];
c = [n./rwr
     n./rwl 
     N__fl/aux.car.mass/aux.car.gravity    % N__fl > opts.Neps
     N__fr/aux.car.mass/aux.car.gravity    % N__fr > opts.Neps
     N__rl/aux.car.mass/aux.car.gravity    % N__rl > opts.Neps
     N__rr/aux.car.mass/aux.car.gravity    % N__rr > opts.Neps
     T__e./T__emax                        % engineTorque/engineMaxTorque < 1
    ] ./ opts.cscale(:);

% Lagrange cost
pnlt = opts.wdeltadot    *[dyout.steerRate         ].^2 + ...
       opts.wdeltaddot   *[dyout.steerAcc          ].^2 + ...
       opts.wlambda      *[dyout.driftAngle        ].^2 + ...
       opts.wlambdadot   *[dyout.driftRate         ].^2 + ...
       opts.wkappa       *([rearLeftTyre.longSlip].^2 + [rearRightTyre.longSlip].^2) + ...
       opts.wuDelta      *u(1,:).^2 + ...
       opts.wuTaut       *u(2,:).^2;
tp = 1 ./ sdot;
l = [tp
     pnlt] * opts.sscale / opts.lscale;

%% Post-processing
if nargout > 3
    data.s = s;
    data.V = V;
    data.n = n;
    data.chi = chi + [dyout.driftAngle];
    data.at = [dyout.tangentialAcc];
    data.an = [dyout.normalAcc];
    data.sdot = full(sdot);
    data.Vdot = full(v1(9,:))*opts.xscale(9);
    data.ndot = full(ndot);
    data.chidot = full(chidot) + [dyout.driftRate];
    xc = full(aux.track.x(s));
    yc = full(aux.track.y(s));
    zc = 0*t;
    theta = full(aux.track.theta(s));
    % xl = full(aux.track.xl(s));
    % yl = full(aux.track.yl(s));
    % zl = 0*t;
    % xr = full(aux.track.xr(s));
    % yr = full(aux.track.yr(s));
    % zr = 0*t;
    % rw = full(aux.track.rw(s));
    if aux.track.isTrack3D
        zc = full(aux.track.z(s));
        % zl = full(aux.track.zl(s));
        % zr = full(aux.track.zr(s));
    end
    % if isnumeric(zetap)
    data.zeta = cumtrapz(data.s, full(zetap));
    % end
    [data.x, data.y, data.z] = carfm.common.getVehCoords(xc, yc, zc, theta, mu, phi, ...
        data.n);
    [data.psi, data.sigma, data.beta] = carfm.common.getVehAngles(theta, mu, phi, ...
                    data.chi);
    [data.vx, data.vy, data.vz] = carfm.common.getVehVels(data.psi, data.sigma, data.V);
    data.omegax = full((Omegax.*cos(data.chi)+Omegay.*sin(data.chi)).*sdot);
    data.omegay = full((Omegay.*cos(data.chi)-Omegax.*sin(data.chi)).*sdot);
    data.omegaz = full([dyout.yawRate] + [dyout.driftRate]);
	data.Gammax = data.omegax./data.V;
	data.Gammay = data.omegay./data.V;
	data.Gammaz = data.omegaz./data.V;
    % data.rha = full(rha); % NOT IMPLEMENTED
    % if isnumeric(tp)
    data.t = cumtrapz(data.s, full(tp));
    % end
    % data.tp = full(tp);
    % data.zetap = full(zetap);
    data.pnlt = pnlt;
    data.data = dyout;
end

end