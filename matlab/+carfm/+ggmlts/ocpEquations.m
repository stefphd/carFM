function [xdot, c, l, data] = ocpEquations(t,x,u,aux,opts)
%OCPEQUATIONS Dynamic and path constraints and Lagrange cost of the OCP.
% 
% INPUT
% t: indepdent variable
% x: states [V, n, chi, at, an]
% u: control [jt, jn]
% aux: auxdata
% opts: options
%
% OUTPUT
% xd: derivative of states x
% c: path constraints
% l: Lagrange (i.e. running) cost integrands
% data: other output data for post-processing

%% Get unscaled data
% independent variable
s = t*opts.sscale;
% states
V = x(1,:)*opts.xscale(1);
n = x(2,:)*opts.xscale(2);
chi = x(3,:)*opts.xscale(3);
at = x(4,:)*opts.xscale(4);
an = x(5,:)*opts.xscale(5);
% control
jt = u(1,:)*opts.uscale(1);
jn = u(2,:)*opts.uscale(2);

%% Eval track
Omegaz = aux.track.Omegaz(s); % geodesic
rwr = aux.track.rwr(s);  % right half-width
rwl = aux.track.rwl(s); % left half-width
% This is to simplify the generated expressions in CASADI in the case of 2D track
if aux.track.isTrack3D
    % 3D track
    Omegax = aux.track.Omegax(s);
    Omegay = aux.track.Omegay(s);
    mu = aux.track.mu(s);
    phi = aux.track.phi(s);
else
    % 2D track
    Omegax = 0*t;
    Omegay = 0*t;
    mu = 0*t;
    phi = 0*t;
end

%% Eval RHA
if aux.track.isRHA % This is to semplify generated expressions in CASADI in the case of no RHA
    % With RHA
    rha = aux.track.rha(s);
    % Enforce either 0 or 1
    rha = 1*(rha>0.5) + 0;
    % Enforce RHA=0 in braking
    if opts.deactRHABrake
        rha = full(rha) .* (at > 0); % full needed for numerical eval, otherwise error
    end
else
    % No RHA
    rha = 0*t;
end

%% Others
wt = opts.wt;
wn = opts.wn;

%% Differential constraints
sdot = V.*cos(chi)./(1-n.*Omegaz);
ndot = V.*sin(chi);
w = 0; % assume negligible vertical velocity
omegax = (Omegax.*cos(chi) + Omegay.*sin(chi)).*sdot;
omegay = (Omegay.*cos(chi) - Omegax.*sin(chi)).*sdot;
omegaz = (an + w.*omegax)./V;
Vdot = at - w.*omegay;
chidot = omegaz - Omegaz.*sdot;
atdot = jt;
andot = jn;
zetap = V ./ sdot;
xdot = [Vdot   ./ sdot / opts.xscale(1)
        ndot   ./ sdot / opts.xscale(2)
        chidot ./ sdot / opts.xscale(3)
        atdot  ./ sdot / opts.xscale(4)
        andot  ./ sdot / opts.xscale(5)]*opts.sscale;

%% Path constraints
g = aux.g0;
% eval tilda accelerations
wdot = 0; % assume negligible rate of vertical velocity
az = wdot - V.*omegay; % vertical acceleration
ateq = at + g*(sin(mu).*cos(chi)-cos(mu).*sin(phi).*sin(chi)); %apparent tangential acceleration
aneq = an - g*(cos(mu).*sin(phi).*cos(chi)+sin(mu).*sin(chi)); %apparent normal acceleration
geq = -az + g*cos(mu).*cos(phi); % apparent gravity
% eval g-g map: rho{l}(alpha,V,gtilda), with alpha,rho computed from axtilda,aytilda
rhomax2 = []; rho2 = []; alpha = [];
% atan2mod = @(y,x) atan(tan(atan2(y, x))).*(+1*(x>=0) - 1*(x<0));
for l = 1 : numel(aux.rho)
    at0 = 0*V;
    for k = 1 : numel(V)
        at0(k) = aux.shift{l}(V(k));
    end
    alpha = [alpha; atan2(ateq-at0*g,aneq)]; 
    rhomax2 = [rhomax2; aux.rho{l}{1}([alpha(l,:);V(:)';geq(:)'/g]).^2];
    % calc rho
    rho2 = [rho2; ((ateq-at0*g).^2+aneq.^2) / g^2];
end
% weight g-g using rha
if aux.track.isRHA && numel(aux.rho)>1
    alpha = (alpha(1,:).*(1-rha) + alpha(2,:).*rha);
    rhomax2 = (rhomax2(1,:).*(1-rha) + rhomax2(2,:).*rha);
    rho2 = (rho2(1,:).*(1-rha) + rho2(2,:).*rha);
else
    rha = 0*t; % RHA not used
end
% take first vals
rho2 = rho2(1,:);
rhomax2 = rhomax2(1,:);
alpha = alpha(1,:);
% create c
c = [n./rwr
     n./rwl
     (rho2+1)./(rhomax2+1) % +1 is to avoid division by 0 if rhomax2=0 (note that rhomax2>=0)
     geq/g % constraint geq within gg map range
     ] ./ opts.cscale(:);

%% Lagrange cost
pnlt = wt*jt.^2+wn*jn.^2; % control penalty
tp = 1 ./ sdot; % dt/ds
l = [tp
     pnlt] * opts.sscale / opts.lscale;

%% Post-processing
if nargout > 3
    data.s = s;
    data.V = V;
    data.n = n;
    data.chi = chi;
    data.at = at;
    data.an = an;
    data.jt = jt;
    data.jn = jn;
    data.sdot = full(sdot);
    data.Vdot = full(Vdot);
    data.ndot = ndot;
    data.chidot = full(chidot);
    data.ateq = full(ateq);
    data.aneq = full(aneq);
    data.geq = full(geq);
    data.at0 = at0;
    data.alpha = full(alpha);
    data.rho = sqrt(full(rho2));
    data.rhomax = full(sqrt(rhomax2));
    xc = full(aux.track.x(s));
    yc = full(aux.track.y(s));
    zc = 0*t;
    % xl = full(aux.track.xl(s));
    % yl = full(aux.track.yl(s));
    % zl = 0*t;
    % xr = full(aux.track.xr(s));
    % yr = full(aux.track.yr(s));
    % zr = 0*t;
    % rw = full(aux.track.rw(s));
    theta = full(aux.track.theta(s));
    mu = 0*t;
    phi = 0*t;
    Omegax = 0*t;
    Omegay = 0*t;
    Omegaz = full(aux.track.Omegaz(s));
    if aux.track.isTrack3D
        zc = full(aux.track.z(s));
        % zl = full(aux.track.zl(s));
        % zr = full(aux.track.zr(s));
        mu = full(aux.track.mu(s));
        phi = full(aux.track.phi(s));
        Omegax = full(aux.track.Omegax(s));
        Omegay = full(aux.track.Omegay(s));
    end
    % if isnumeric(zetap)
    data.zeta = cumtrapz(data.s, full(zetap));
    % end
    [data.x, data.y, data.z] = carfm.common.getVehCoords(xc, yc, zc, theta, mu, phi, ...
                                                   data.n);
    [data.psi, data.sigma, data.beta] = carfm.common.getVehAngles(theta, mu, phi, ...
                                                            data.chi);
    [data.vx, data.vy, data.vz] = carfm.common.getVehVels(data.psi, data.sigma, data.V);
    data.omegax = full(omegax);
    data.omegay = full(omegay);
    data.omegaz = full(omegaz);
	data.Gammax = data.omegax./data.V;
	data.Gammay = data.omegay./data.V;
	data.Gammaz = data.omegaz./data.V;
    data.rha = full(rha);
    % if isnumeric(tp)
    data.t = cumtrapz(data.s, full(tp));
    % end
    % data.tp = full(tp);
    % data.zetap = full(zetap);
    data.pnlt = pnlt;
    
end

end

