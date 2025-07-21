function [psi, sigma, beta] = getVehAngles(theta, mu, phi, chi)
%GETVEHANGLES Computes the vehicle angles.
% Function inputs:
%   - theta: road heading
%   - mu: road slope
%   - phi: road banking
%   - chi: vehicle relative orientation
% Function outputs:
%   - psi: vehicle heading
%   - sigma: vehicle slope
%   - beta: vehicle banking

psi = atan2(sin(theta).*cos(mu).*cos(chi) + (+cos(theta).*cos(phi) + sin(theta).*sin(mu).*sin(phi)).*sin(chi), ...
                        cos(theta).*cos(mu).*cos(chi) + (-sin(theta).*cos(phi) + cos(theta).*sin(mu).*sin(phi)).*sin(chi));

sigma = asin(sin(mu).*cos(chi) - cos(mu).*sin(phi).*sin(chi));

beta = atan2(sin(mu).*sin(chi) + cos(mu).*sin(phi).*cos(chi), ...
                         cos(mu).*cos(phi));

end