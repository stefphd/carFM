function [x,y,z] = getVehCoords(xc,yc,zc,theta,mu,phi,n)
% GETVEHCOORDS Computes the point XYZ coordinates with a given lateral
% position.
% Function inputs:
%   - xc      : x coordinate of centerline
%   - yc      : y coordinate of centerline
%   - zc      : z coordinate of centerline
%   - theta   : heading angle of track
%   - mu      : slope angle of track
%   - phi     : camber angle of track
%   - n       : lateral position
% Function outputs:
%   - x       : x coordinate of the point with given lateral position
%   - y       : y coordinate of the point with given lateral position
%   - z       : z coordinate of the point with given lateral position

x = xc+(-sin(theta).*cos(phi)+cos(theta).*sin(mu).*sin(phi)).*n; %x-coord
y = yc+( cos(theta).*cos(phi)+sin(theta).*sin(mu).*sin(phi)).*n; %y-coord
z = zc+( cos(mu).*sin(phi)).*n; %z-coord

end