function [x,y,psi,phi,mu,Dx,Dy,Dpsi,Dphi,Dmu] = suspensionKinematics(susp, z, delta)
%suspensionKinematics Suspension kinematic function
%
% INPUT
% susp: structure containining suspension constants
% z:    suspension travel (m)
%   z = 0 fully extended
%   z > 0 in compression
% delta: steer angle, 0 for rear suspension (>0 clockwise) (rad)
%
% OUTPUT
% x: wheel center x-displacement (>0 forward)
% y: wheel center y-displacement (>0 rightward)
% psi: wheel yaw (>0 clockwise)
% phi: wheel camber (>0 clockwise)
% mu: wheel pitch
% Dx: dx/dz
% Dy: dy/dz
% Dpsi: dpsi/dz
% Dphi: dphi/dz
% Dmu: dmu/dz

% No suspension motions
x = 0*z;
y = 0*z;
psi = 0*z;
phi = 0*z;
mu = 0*z;
Dx = 0*z;
Dy = 0*z;
Dpsi = 0*z;
Dphi = 0*z;
Dmu = 0*z;

end

