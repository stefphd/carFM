function [Tfl,Tfr,Trl,Trr] = basicTransmission(transmission, omegafl, omegafr, omegarl, omegarr, Td)
%basicTransmission 
%
% INPUT
% transmission: structure containing transmission constants
% omegaij:      wheel angular velocities (ij=fl,fr,rl,rr) (rad/s)
% Td:           total driving torque, including engine brake, i.e. eventually
%               negative in coast-down (Nm)
%
% OUTPUT
% Tij:          wheel torque (ij=fl,fr,rl,rr) (Nm)

% params
gammad = transmission.gammad; % drive ratio
kd = transmission.kd; % diff stiffness

% split front/rear depending on gammad
Tf = gammad*Td;
Tr = Td - Tf; % 0.5*(1-gammad)*Td

% split left/right dependin on differential stiffness
Tfl = 0.5*Tf + kd*(omegafl-omegafr);
Tfr = 0.5*Tf - kd*(omegafl-omegafr);
Trl = 0.5*Tr + kd*(omegarl-omegarr);
Trr = 0.5*Tr - kd*(omegarl-omegarr);

end