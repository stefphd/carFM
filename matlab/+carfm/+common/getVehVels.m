function [vx, vy, vz] = getVehVels(psi, sigma, V)
%GETVEHVELS Computes the vehicle velocities.
% Function inputs:
%   - psi: vehicle heading
%   - sigma: vehicle slope
% Function outputs:
%   - vx: x velocity
%   - vy: y velocity
%   - vz: z velocity

vx = V .* cos(psi).*cos(sigma);
vy = V .* sin(psi).*cos(sigma);
vz = -V .* sin(sigma);

end