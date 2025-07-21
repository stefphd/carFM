function [FD, FS, FL, MR, MP, MY] = basicAeroModel(aero, V, ax, ay, az, mu, phi, z)
%basicAeroModel 
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
% Aerodynamic reference frame is
% - x axis pointing backward  (-x SAE)
% - y axis pointing rightward (+y SAE)
% - z axis pointing upward    (-z SAE)

FD = 1/2*aero.cda*aero.rhoa*V^2;
FS = 0*V;
FLf = 1/2*aero.claf*aero.rhoa*V^2;
FLr = 1/2*aero.clar*aero.rhoa*V^2;
FL = FLf+FLr;
MR = 0*V;
MP = FLf*aero.w;
MY = 0*V;

end

