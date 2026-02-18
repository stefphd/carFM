function [tyreFx,tyreFy,tyreMx,tyreMy,tyreMz,tyreLS,tyreSA] = tyreMFBasicModel(tyre,N,Vs,Vn,Vr,Omega,ca)
% tyreMFBasicModel
% MF tyre model using a subset of the full parameters. Employed coefficients are:
%   Nominal: FNOMIN, UNLOADED_RADIUS, CROSS_RADIUS
%   Longitudinal: PCX1, PDX1, PDX2, PEX1, PEX4, PKX1, RBX1, RBX2, RCX1
%   Lateral: PCY1, PDY1, PDY2, PEY1, PKY1, PKY6, RBY1, RBY2, RCY1
%   Scaling factors: LCX, LMUX, LEX, LKX, LCY, LMUY, LEY, LKY, LKYG, LXA, LYK
% All remeaning coefficients are implicitly set to zero.
% See also Sec. 4.3.2 of "Tyre and Vehicle Dynamics", 3rd edition, 2012 Hans B. Pacejka
%
% INPUT
% tyre:     strcuture containing tyre constants
% N:        load (N)
% Vs:       tangent speed  (m/s)
% Vn:       normal speed   (m/s)
% Vr:       rolling speed  (m/s)
% Omega:    angular speed  (rad/s)
% ca:       camber angle   (rad)
%
% OUTPUT
% tyreFx:   longitudinal force
% tyreFy:   lateral      force
% tyreMx:   overturning  moment
% tyreMy:   rolling      moment
% tyreMz:   aligning     moment
% tyreLS:   longitudinal slip
% tyreSA:   slip         angle

% calculates slips
R0 = tyre.UNLOADED_RADIUS;
tyreSA = atan(Vn/Vs);
tyreLS = -(1-Omega*R0/Vs); % approx Re=R0

% nominal load
Fz = N; % MF uses Fz as N
dfz = (Fz-tyre.FNOMIN)/tyre.FNOMIN;

% pure longitudinal
k = tyreLS;
kx = k;
Cx = tyre.PCX1*tyre.LCX;
mux = (tyre.PDX1+tyre.PDX2*dfz)*tyre.LMUX;
Dx = mux*Fz;
Ex = tyre.PEX1*(1-tyre.PEX4*sign(kx))*tyre.LEX;
Kxk = Fz*tyre.PKX1*tyre.LKX;
Bx = Kxk/(Cx*Dx);
Fx0 = Dx*sin(Cx*atan(Bx*kx-Ex*(Bx*kx-atan(Bx*kx))));

% pure lateral
alpha = tyreSA;
gamma = ca;
Cy = tyre.PCY1*tyre.LCY;
muy = (tyre.PDY1+tyre.PDY2*dfz)*tyre.LMUY;
Dy = muy*Fz;
Ey = tyre.PEY1*tyre.LEY;
Kya = Fz*tyre.PKY1*tyre.LKY;
By = Kya/(Cy*Dy);
Kyg0 = Fz*tyre.PKY6*tyre.LKYG;
SHy = Kyg0/Kya*gamma;
alphay = alpha + SHy;
Fy0 = Dy*sin(Cy*atan(By*alphay-Ey*(By*alphay-atan(By*alphay))));

% combined longitudinal
Bxa = tyre.RBX1*cos(atan(tyre.RBX2*k))*tyre.LXA;
Cxa = tyre.RCX1;
Gxa = cos(Cxa*atan(Bxa*alpha));
Fx = Gxa*Fx0;

% combined lateral
Byk = tyre.RBY1*cos(atan(tyre.RBY2*alpha))*tyre.LYK;
Cyk = tyre.RCY1;
Gyk = cos(Cyk*atan(Byk*k));
Fy = Gyk*Fy0;

% forces
tyreFx = Fx;
tyreFy = Fy;
tyreMx = 0;
tyreMy = 0;
tyreMz = 0;

end