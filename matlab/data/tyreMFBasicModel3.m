function [tyreFx,tyreFy,tyreMx,tyreMy,tyreMz,tyreLS,tyreSA] = tyreMFBasicModel3(tyre,N,Vs,Vn,Vr,Omega,ca,phit)
% tyreMFBasicModel3
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

% calculates slips
tyreSA = atan(Vn/Vs); % Sideslip
tyreLS = -(1-Vr/Vs); % Longitudinal slip

% saturation
fsat = @(x,xl,xu) xl*(x<xl) + xu*(x>xu) + x*(x<=xu)*(x>=xl);
tyreLS = fsat(tyreLS, tyre.KPUMIN, tyre.KPUMAX);
tyreSA = fsat(tyreSA, tyre.ALPMIN, tyre.ALPMAX);
% N = fsat(N, tyre.FZMIN, tyre.FZMAX);

%calc tyre forces
[tyreFx, tyreFy] = MFBasic_FxFy(tyre,tyreLS,tyreSA,N);
tyreMx = 0;
tyreMy = 0;
tyreMz = 0;

end

function [Fx, Fy] = MFBasic_FxFy(tyre,k,alpha,Fz)
    %% Equivalent slip
    sigmaeps = 1e-5; % a small (negligible) slip to avoid division by 0
    sigmax = k/(k+1);
    sigmay = tan(alpha)/(k+1);
    sigma = sqrt(sigmax^2+sigmay^2+sigmaeps^2);

    %% Pure forces
    Fx0 = MFBasic_Fx0(tyre,sigma,Fz);
    Fy0 = MFBasic_Fy0(tyre,sigma,Fz);

    %% Combined 
    Fx = Fx0 * sigmax / sigma;
    Fy = Fy0 * sigmay / sigma;

end

function Fx0 = MFBasic_Fx0(tyre,sigma,Fz)
    %% Pure slip
    dfz = (Fz-tyre.FNOMIN)/tyre.FNOMIN;
    % Peak coefficient
    mux = (tyre.PDX1 + tyre.PDX2*dfz)*tyre.LMUX;
    Dx = mux*Fz;
    % Shape factor
    Cx = tyre.PCX1*tyre.LCX;
    % Longitudinal stiffness
    Kxk = Fz*tyre.PKX1*exp(tyre.PKX3*dfz)*tyre.LKX;
    % B-factor
    Bx = Kxk/(Cx*Dx);
    % Curvature factor
    Ex = tyre.PEX1*tyre.LEX;
    % Longitudinal foce
    Fx0 = Dx*sin(Cx*atan(Bx*sigma-Ex*(Bx*sigma-atan(Bx*sigma))));
end

function Fy0 = MFBasic_Fy0(tyre,sigma, Fz)
    %% Pure slip
    dfz = (Fz-tyre.FNOMIN)/tyre.FNOMIN;
    % Peak coefficient
    muy = (tyre.PDY1 + tyre.PDY2*dfz)*tyre.LMUY;
    Dy = muy*Fz;
    % Shape factor
    Cy = tyre.PCY1*tyre.LCY;
    % Cornering stiffness
    Kya = tyre.FNOMIN*tyre.PKY1*sin(2*atan(Fz/tyre.PKY2/tyre.FNOMIN))*tyre.LKY;
    % B-factor
    By = Kya/(Cy*Dy);
    % Curvature factor
    Ey = tyre.PEY1*tyre.LEY;
    % Lateral force
    Fy0 = Dy*sin(Cy*atan(By*sigma-Ey*(By*sigma-atan(By*sigma))));
end