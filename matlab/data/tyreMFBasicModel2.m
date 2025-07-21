function [tyreFx,tyreFy,tyreMx,tyreMy,tyreMz,tyreLS,tyreSA] = tyreMFBasicModel2(tyre,N,Vs,Vn,Vr,Omega,ca,phit)
% tyreMFBasicModel2
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
tyreFx = MFBasic_Fx(tyre,tyreLS,tyreSA,ca,phit,N);
tyreFy = MFBasic_Fy(tyre,tyreLS,tyreSA,ca,phit,N);
tyreMx = 0;
tyreMy = 0;
tyreMz = 0;

end

function Fx = MFBasic_Fx(tyre,k,alpha,gamma,phi,Fz)
    %% Pure slip
    Fx0 = MFBasic_Fx0(tyre,k,alpha,gamma,phi,Fz);
    %% Combined slip
    % perfect ellipse from Eq. (3.35) in "Motorcycle Design: Vehicle
    % Dynamics Concepts and Applications"
    [Fy0,muy] = MFBasic_Fy0(tyre,k,alpha,gamma,phi,Fz);
    Gxa = sqrt(1-0.5*(Fy0/muy/Fz)^2);
    Fx = Fx0 * Gxa;
end

function Fy = MFBasic_Fy(tyre,k,alpha,gamma,phi,Fz)
    %% Pure slip
    Fy0 = MFBasic_Fy0(tyre,k,alpha,gamma,phi,Fz);
    %% Combined slip
    % perfect ellipse from Eq. (3.35) in "Motorcycle Design: Vehicle
    % Dynamics Concepts and Applications"
    [Fx0, mux] = MFBasic_Fx0(tyre,k,alpha,gamma,phi,Fz);
    Gyk = sqrt(1-0.5*(Fx0/mux/Fz)^2);
    Fy = Fy0 * Gyk;
    
end

function [Fx0, mux] = MFBasic_Fx0(tyre,kappa,~,~,~,Fz)
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
    Fx0 = Dx*sin(Cx*atan(Bx*kappa-Ex*(Bx*kappa-atan(Bx*kappa))));
end

function [Fy0,muy] = MFBasic_Fy0(tyre,~,alpha,~,~,Fz)
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
    Fy0 = Dy*sin(Cy*atan(By*alpha-Ey*(By*alpha-atan(By*alpha))));
end