function [tyreFx,tyreFy,tyreMx,tyreMy,tyreMz,tyreLS,tyreSA] = tyreMF62Model(tyre,N,Vs,Vn,Vr,Omega,ca,phit)
% tyreMF62Model
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
tyreSA = atan(Vn/Vs);  % Sideslip
tyreLS = -(1-Vr/Vs); % Longitudinal slip

% inflation pressure - from tyre data
p = tyre.INFLPRES;

% saturation
fsat = @(x,xl,xu) xl*(x<xl) + xu*(x>xu) + x*(x<=xu)*(x>=xl);
tyreLS = fsat(tyreLS, tyre.KPUMIN, tyre.KPUMAX);
tyreSA = fsat(tyreSA, tyre.ALPMIN, tyre.ALPMAX);
% N = fsat(N, tyre.FZMIN, tyre.FZMAX);

%calc tyre forces
tyreFx = MF62_Fx(tyre,tyreLS,tyreSA,ca,phit,N,p);
tyreFy = MF62_Fy(tyre,tyreLS,tyreSA,ca,phit,N,p);
tyreMx = MF62_Mx(tyre,tyreLS,tyreSA,ca,phit,N,p); 
tyreMy = MF62_My(tyre,tyreLS,tyreSA,ca,phit,N,p,Vs);
tyreMz = MF62_Mz(tyre,tyreLS,tyreSA,ca,phit,N,p);

end

function [Fx,Kxk,Gxa] = MF62_Fx(tyre,k,alpha,gamma,phi,Fz,P)
    % Tire longitudinal force described in the MF 6.2 model
    % Function inputs:
    % - tyre: structure containing all the *.tir coefficients
    % - k: longitudinal slip value
    % - alpha: sideslip value
    % - gamma: camber value
    % - phi: turnslip value
    % - Fz: vertical load value
    % - P: pressure value
    % Function outputs:
    % - Fx: tyre longitudinal force
    % - Kxk: longitudinal stiffness
    % - Gxa: longitudinal weighting function
    
    % Vertical load increment
    dfz = (Fz-tyre.FNOMIN)/tyre.FNOMIN;
    
    % Inflation pressure increment
    dpi = (P-tyre.NOMPRES)/tyre.NOMPRES;
    
    %% Turn slip
    
    if phi==0
        zeta1 = 1;
    else
        Bxphi = tyre.PDXP1*(1+tyre.PDXP2*dfz)*cos(atan(tyre.PDXP3*k));
        zeta1 = cos(atan(Bxphi*tyre.UNLOADED_RADIUS*phi));
    end
    
    %% Pure slip
    
    % Peak coefficient
    mux = (tyre.PDX1+tyre.PDX2*dfz)*(1+tyre.PPX3*dpi+tyre.PPX4*dpi^2)*(1-tyre.PDX3*gamma^2)*tyre.LMUX;
    Dx = mux*Fz*zeta1;
    
    % Shape factor
    Cx = tyre.PCX1*tyre.LCX;
    
    % Longitudinal stiffness
    Kxk = Fz*(tyre.PKX1+tyre.PKX2*dfz)*exp(tyre.PKX3*dfz)*(1+tyre.PPX1*dpi+tyre.PPX2*dpi^2)*tyre.LKX;
    
    % B-factor
    Bx = Kxk/(Cx*Dx);
    
    % Horizontal shift
    Shx = (tyre.PHX1+tyre.PHX2*dfz)*tyre.LHX;
    kx = k+Shx;
    
    % Curvature factor
    Ex = (tyre.PEX1+tyre.PEX2*dfz+tyre.PEX3*dfz^2)*(1-tyre.PEX4*sign(kx))*tyre.LEX;
    
    % Vertical shift
    Svx = Fz*(tyre.PVX1+tyre.PVX2*dfz)*tyre.LVX*tyre.LMUX*zeta1;
    
    %% Combined slip
    
    Cxa = tyre.RCX1;
    Bxa = (tyre.RBX1+tyre.RBX3*gamma^2)*cos(atan(tyre.RBX2*k))*tyre.LXAL;
    Shxa = tyre.RHX1;
    alphas = alpha+Shxa;
    Exa = tyre.REX1+tyre.REX2*dfz;
    Gxa = cos(Cxa*atan(Bxa*alphas-Exa*(Bxa*alphas-atan(Bxa*alphas))))/cos(Cxa*atan(Bxa*Shxa-Exa*(Bxa*Shxa-atan(Bxa*Shxa))));
    
    %% Longitudinal force
    
    Fx = (Dx*sin(Cx*atan(Bx*kx-Ex*(Bx*kx-atan(Bx*kx))))+Svx)*Gxa;

end

function [Fy,Kya,Kyg0,Gyk,By,Cy,Shy,Svy,muy] = MF62_Fy(tyre,k,alpha,gamma,phi,Fz,P)
    % Tire lateral force described in the MF 6.2 model
    % Function inputs:
    % - tyre: structure containing all the *.tir coefficients
    % - k: longitudinal slip value
    % - alpha: sideslip value
    % - gamma: camber value
    % - phi: turnslip value
    % - Fz: vertical load value
    % - P: pressure value
    % Function outputs:
    % - Fy: tyre lateral force
    % - Kya: cornering stiffness
    % - Kyg0: camber stiffness
    % - Gyk: lateral weighting function
    % - By: lateral B factor
    % - Cy: lateral shape factor
    % - Shy: horizontal shift factor
    % - Svy: vertical shift factor
    % - muy: lateral friction coefficient
    
    % Scaled nominal load
    Fz0p = tyre.LFZO*tyre.FNOMIN;
    
    % Vertical load increment
    dfz = (Fz-tyre.FNOMIN)/tyre.FNOMIN;
    
    % Inflation pressure increment
    dpi = (P-tyre.NOMPRES)/tyre.NOMPRES;
    
    %% Turn slip
    
    if phi==0
        zeta0 = 1;
        zeta2 = 1;
        zeta3 = 1;
        zeta4 = 1;
    else
        zeta0 = 0;
        Byphi = tyre.PDYP1*(1+tyre.PDYP2*dfz)*cos(atan(tyre.PDYP3*tan(alpha)));
        zeta2 = cos(atan(Byphi*(tyre.UNLOADED_RADIUS*abs(phi)+tyre.PDYP4*sqrt(tyre.UNLOADED_RADIUS*abs(phi)))));
        zeta3 = cos(atan(tyre.PKYP1*tyre.UNLOADED_RADIUS^2*phi^2));
    end
    
    %% Pure slip
    
    % Peak coefficient
    muy = (tyre.PDY1+tyre.PDY2*dfz)*(1+tyre.PPY3*dpi+tyre.PPY4*dpi^2)*(1-tyre.PDY3*gamma^2)*tyre.LMUY;
    Dy = muy*Fz*zeta2;
    
    % Shape factor
    Cy = tyre.PCY1*tyre.LCY;
    
    % Cornering stiffness
    Kya0 = tyre.PKY1*Fz0p*(1+tyre.PPY1*dpi)*sin(tyre.PKY4*atan(Fz/(tyre.PKY2*Fz0p*(1+tyre.PPY2*dpi))))*tyre.LKY;
    Kya = tyre.PKY1*Fz0p*(1+tyre.PPY1*dpi)*sin(tyre.PKY4*atan(Fz/((tyre.PKY2+tyre.PKY5*gamma^2)*Fz0p*(1+tyre.PPY2*dpi))))*(1-tyre.PKY3*abs(gamma))*tyre.LKY*zeta3;
    
    % B-factor
    By = Kya/(Cy*Dy);
    
    % Camber stiffness
    Kyg0 = (tyre.PKY6+tyre.PKY7*dfz)*Fz*tyre.LKYC*(1+tyre.PPY5*dpi);
    
    % Vertical shift
    Svy0 = Fz*(tyre.PVY1+tyre.PVY2*dfz)*tyre.LVY*tyre.LMUY;
    Svyg = Fz*(tyre.PVY3+tyre.PVY4*dfz)*gamma*tyre.LKYC*tyre.LMUY*zeta2;
    Svy = Svy0*zeta2+Svyg;
    
    % Turn slip
    if phi~=0
        Dhyphi = tyre.PHYP2+tyre.PHYP3*dfz;
        Chyphi = tyre.PHYP1;
        epsilony = tyre.PECP1*(1+tyre.PECP2*dfz);
        Kyrphi0 = Kyg0/(1-epsilony);
        Bhyphi = -Kyrphi0/(Chyphi*Dhyphi*Kya0);
        Ehyphi = tyre.PHYP4;
        Shyphi = Dhyphi*sin(Chyphi*atan(Bhyphi*tyre.UNLOADED_RADIUS*phi-Ehyphi*(Bhyphi*tyre.UNLOADED_RADIUS*phi-atan(Bhyphi*tyre.UNLOADED_RADIUS*phi))));
        zeta4 = 1+Shyphi-Svyg/Kya;
    end
    
    % Horizontal shift
    Shy0 = (tyre.PHY1+tyre.PHY2*dfz)*tyre.LHY;
    Shyg = (Kyg0*gamma-Svyg)/Kya*zeta0+zeta4-1;
    Shy = Shy0+Shyg;
    alphay = alpha+Shy;
    
    % Curvature factor
    Ey = (tyre.PEY1+tyre.PEY2*dfz)*(1+tyre.PEY5*gamma^2-(tyre.PEY3+tyre.PEY4*gamma)*sign(alphay))*tyre.LEY;
    
    % Lateral force
    Fyp = Dy*sin(Cy*atan(By*alphay-Ey*(By*alphay-atan(By*alphay))))+Svy;
    
    %% Combined slip
    
    % Weighting function
    Cyk = tyre.RCY1;
    Byk = (tyre.RBY1+tyre.RBY4*gamma^2)*cos(atan(tyre.RBY2*(alpha-tyre.RBY3)))*tyre.LYKA;
    Shyk = tyre.RHY1+tyre.RHY2*dfz;
    ks = k+Shyk;
    Eyk = tyre.REY1+tyre.REY2*dfz;
    Gyk = cos(Cyk*atan(Byk*ks-Eyk*(Byk*ks-atan(Byk*ks))))/cos(Cyk*atan(Byk*Shyk-Eyk*(Byk*Shyk-atan(Byk*Shyk))));
    
    % Braking induces plysteer
    Dvyk = muy*Fz*(tyre.RVY1+tyre.RVY2*dfz+tyre.RVY3*gamma)*cos(atan(tyre.RVY4*alpha))*zeta2;
    Svyk = Dvyk*sin(tyre.RVY5*atan(tyre.RVY6*k))*tyre.LVYKA;
    
    %% Lateral force
    
    Fy = Gyk*Fyp+Svyk;

end

function [Mx] = MF62_Mx(tyre,k,alpha,gamma,phi,Fz,P)
    % Tire overturning moment described in the MF 6.2 model
    % Function inputs:
    % - tyre: structure containing all the *.tir coefficients
    % - k: longitudinal slip value
    % - alpha: sideslip value
    % - gamma: camber value
    % - phi: turnslip value
    % - Fz: vertical load value
    % - P: pressure value
    % Function outputs:
    % - Mx: tyre overturning moment
    
    % Inflation pressure increment
    dpi = (P-tyre.NOMPRES)/tyre.NOMPRES;
    
    % Lateral force
    Fy = MF62_Fy(tyre,k,alpha,gamma,phi,Fz,P);
    
    %% Overturning moment
    
    Mx = tyre.UNLOADED_RADIUS*Fz*tyre.LMX*(tyre.QSX1*tyre.LVMX-tyre.QSX2*gamma*(1+tyre.PPMX1*dpi)+tyre.QSX3*Fy/tyre.FNOMIN+tyre.QSX4*cos(tyre.QSX5*atan((tyre.QSX6*Fz/tyre.FNOMIN)^2))*sin(tyre.QSX7*gamma+tyre.QSX8*atan(tyre.QSX9*Fy/tyre.FNOMIN))+tyre.QSX10*atan(tyre.QSX11*Fz/tyre.FNOMIN)*gamma)+...
         tyre.UNLOADED_RADIUS*tyre.LMX*(Fy*(tyre.QSX13+tyre.QSX14*abs(gamma))-Fz*tyre.QSX12*gamma*abs(gamma));

end

function [My] = MF62_My(tyre,k,alpha,gamma,phi,Fz,P,Vx)
    % Tire rolling resistance moment described in the MF 6.2 model
    % Function inputs:
    % - tyre: structure containing all the *.tir coefficients
    % - k: longitudinal slip value
    % - alpha: sideslip value
    % - gamma: camber value
    % - phi: turnslip value
    % - Fz: vertical load value
    % - P: pressure value
    % - Vx: forward velocity
    % Function outputs:
    % - Mx: tyre overturning moment
    
    % Longitudinal force
    Fx = MF62_Fx(tyre,k,alpha,gamma,phi,Fz,P);
    
    %% Rolling resistance moment
    
    My = -tyre.UNLOADED_RADIUS*tyre.FNOMIN*tyre.LMY*(tyre.QSY1+tyre.QSY2*Fx/tyre.FNOMIN+tyre.QSY3*abs(Vx/tyre.LONGVL)+tyre.QSY4*((Vx/tyre.LONGVL)^4)+tyre.QSY5*(gamma^2)+tyre.QSY6*Fz/tyre.FNOMIN*(gamma^2))*((Fz/tyre.FNOMIN)^tyre.QSY7*(P/tyre.NOMPRES)^tyre.QSY8);

end

function [Mz] = MF62_Mz(tyre,k,alpha,gamma,phi,Fz,P)
    % Tire self aligning moment described in the MF 6.2 model
    % Function inputs:
    % - tyre: structure containing all the *.tir coefficients
    % - k: longitudinal slip value
    % - alpha: sideslip value
    % - gamma: camber value
    % - phi: turnslip value
    % - Fz: vertical load value
    % - P: pressure value
    % Function outputs:
    % - Mz: tyre self aligning moment
    
    % Scaled nominal load
    Fz0p = tyre.LFZO*tyre.FNOMIN;
    
    % Vertical load increment
    dfz = (Fz-tyre.FNOMIN)/tyre.FNOMIN;
    
    % Inflation pressure increment
    dpi = (P-tyre.NOMPRES)/tyre.NOMPRES;
    
    % Longitudinal force
    [Fx,Kxk,~] = MF62_Fx(tyre,k,alpha,gamma,phi,Fz,P);
    
    % Lateral force
    [Fy,Kya,~,Gyk,By,Cy,Shy,Svy,muy] = MF62_Fy(tyre,k,alpha,gamma,phi,Fz,P);
    Fy0 = MF62_Fy(tyre,k,alpha,0,0,Fz,P);
    
    %% Turn slip
    
    if phi==0
        zeta0 = 1;
        zeta2 = 1;
        zeta5 = 1;
        zeta6 = 1;
        zeta7 = 1;
        zeta8 = 1;
    else
        zeta0 = 0;
        Byphi = tyre.PDYP1*(1+tyre.PDYP2*dfz)*cos(atan(tyre.PDYP3*tan(alpha)));
        zeta2 = cos(atan(Byphi*(tyre.UNLOADED_RADIUS*abs(phi)+tyre.PDYP4*sqrt(tyre.UNLOADED_RADIUS*abs(phi)))));
        zeta5 = cos(atan(tyre.QDTP1*tyre.UNLOADED_RADIUS*phi));
        zeta6 = cos(atan(tyre.QBRP1*tyre.UNLOADED_RADIUS*phi));
        Mzphiinf = tyre.QCRP1*abs(muy)*tyre.UNLOADED_RADIUS*Fz*sqrt(Fz/Fz0p)*tyre.LMP;
        Mzphi90 = Mzphiinf*2/pi*atan(tyre.QCRP2*tyre.UNLOADED_RADIUS*abs(phi))*Gyk;
        Cdrphi = tyre.QDRP1;
        Ddrphi = Mzphiinf/sin(pi/2*Cdrphi);
        Kzgr0 = Fz*tyre.UNLOADED_RADIUS*(tyre.QDZ8+tyre.QDZ9*dfz+(tyre.QDZ10+tyre.QDZ11*dfz)*abs(gamma))*tyre.LKZC;
        epsilony = tyre.PECP1*(1+tyre.PECP2*dfz);
        Bdrphi = Kzgr0/(Cdrphi*Ddrphi*(1-epsilony));
        Drphi = Ddrphi*sin(Cdrphi*atan(Bdrphi*tyre.UNLOADED_RADIUS*phi));
        zeta7 = 2/pi*acos(Mzphi90/abs(Drphi));
        zeta8 = 1+Drphi;
    end
    
    %% Combined slip
    
    Sht = tyre.QHZ1+tyre.QHZ2*dfz+(tyre.QHZ3+tyre.QHZ4*dfz)*gamma;
    alphat = alpha+Sht;
    alphateq = atan(sqrt(tan(alphat)^2+(Kxk/Kya)^2*k^2))*sign(alphat);
    
    alphar = alpha+Shy+Svy/Kya;
    alphareq = atan(sqrt(tan(alphar)^2+(Kxk/Kya)^2*k^2))*sign(alphar);
    s = (tyre.SSZ1+tyre.SSZ2*(Fy/tyre.FNOMIN)+(tyre.SSZ3+tyre.SSZ4*dfz)*gamma)*tyre.UNLOADED_RADIUS*tyre.LS;
    
    %% Pneumatic trail
    
    Dt = Fz*(tyre.QDZ1+tyre.QDZ2*dfz)*(1-tyre.PPZ1*dpi)*(1+tyre.QDZ3*gamma+tyre.QDZ4*gamma^2)*tyre.UNLOADED_RADIUS/tyre.FNOMIN*tyre.LTR*zeta5;
    Ct = tyre.QCZ1;
    Bt = (tyre.QBZ1+tyre.QBZ2*dfz+tyre.QBZ3*dfz^2)*(1+tyre.QBZ4*gamma+tyre.QBZ5*abs(gamma))*tyre.LKY/tyre.LMUY;
    Et = (tyre.QEZ1+tyre.QEZ2*dfz+tyre.QEZ3*dfz^2)*(1+(tyre.QEZ4+tyre.QEZ5*gamma)*2/pi*atan(Bt*Ct*alphat));
    t = Dt*cos(Ct*atan(Bt*alphateq-Et*(Bt*alphateq-atan(Bt*alphateq))))*cos(alpha);
    
    %% Residual moment
    
    Dr = Fz*tyre.UNLOADED_RADIUS*tyre.LMUY*cos(alpha)*((tyre.QDZ6+tyre.QDZ7*dfz)*tyre.LRES*zeta2+(tyre.QDZ8+tyre.QDZ9*dfz)*gamma*tyre.LKZC*(1+tyre.PPZ2*dpi)*zeta0+(tyre.QDZ10+tyre.QDZ11*dfz)*gamma*abs(gamma)*tyre.LKZC*zeta0)-zeta8+1;
    Br = (tyre.QBZ9*tyre.LKY/tyre.LMUY+tyre.QBZ10*By*Cy)*zeta6;
    Mzr = Dr*cos(zeta7*atan(Br*alphareq));
    
    %% Self aligning moment
    
    Mz = -t*Fy0+Mzr+s*Fx;

end
