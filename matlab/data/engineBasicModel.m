function [propulsionTorque, brakingTorque] = engineBasicModel(engine, omega)
%engineBasicModel
%
% INPUT
% engine: structure containing engine constants
% omega: engine angular velocity (rad/s)
%
% OUTPUT
% propulsionTorque: propulsion engine torque at the crankshaft
% brakingTorque: braking engine torque at thecrankshaft

% Propulsion torque
propulsionTorque = engine.Pmax ./ omega;

% Braking torque
brakingTorque = engine.Pbrake ./ omega;

end