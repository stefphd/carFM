function [Fl,Fr] = suspensionLinearModel(susp, zl, zr, zldot, zrdot)
%suspensionLinearModel Suspension with linear model
%
% INPUT
% susp: structure containining suspension constants
% zl:  left suspension travel (m)
% zr:  right suspension travel
%   zi = 0 fully extended
%   zi > 0 in compression
% zldot: left suspension travel rate 
% zrdot: right suspension travel rate 
%
% OUTPUT
% Fl: left suspension force (>0 when compressed, as for suspension travel)
% Fr: right suspension force (>0 when compressed, as for suspension travel)
%
% NOTE: end-stroke pads are included in the force definition

% Susp model
Fa = susp.ka*(zl-zr);
Fl = susp.F0 + susp.k*zl + susp.c*zldot + Fa;
Fr = susp.F0 + susp.k*zr + susp.c*zrdot - Fa;

end