%% Main computation of SS data from G-G MLTS 
clc,clear

% folder with functions and data
addpath('data')

% bike data - must be consistent with the G-G map
car = load('data/myCarData');

% G-G map
gg = load('gg-2d'); % full G-G

% MLTS result
mlts = load('ggmlts-fm-free');

% Options
opts = struct();
% opts.refineMLTS = true;
% opts.refineRange = [1000 2000];
% opts.mex = true;

% Call to ggmlts2ss
mlts = carfm.ggmlts2ss(car, gg, mlts, opts);
ssres = mlts.data; % this contains SSA data

% % Get tyre data
% frontTyre = [ssres.frontTyre];
% rearTyre = [ssres.rearTyre];
% 
% % Calculate the tyre usage metrics
% frontMetric = carfm.tyreUsageMetric(car.frontTyre, frontTyre);
% rearMetric = carfm.tyreUsageMetric(car.rearTyre, rearTyre);

% Plots
% pitch & roll angle
figure
subplot(211)
plot(mlts.s, [ssres.pitchAngle], 'LineWidth', 1)
ylabel('Pitch angle (deg)')
xlim(mlts.s([1 end]))
subplot(212)
plot(mlts.s, [ssres.rollAngle]*180/pi, 'LineWidth', 1)
ylabel('Roll angle (deg)')
xlabel('Travelled distance (m)')
xlim(mlts.s([1 end]))

% Equivalent acceleration
figure
subplot(211)
plot(mlts.s, [ssres.tangentialAcc]/car.gravity, 'LineWidth', 1)
ylabel('Eq. longitudinal acc (g)')
xlim(mlts.s([1 end]))

subplot(212)
plot(mlts.s,[ssres.normalAcc]/car.gravity, 'LineWidth', 1)
ylabel('Eq. normal acc (g)')
xlabel('Travelled distance (m)')
xlim(mlts.s([1 end]))

% Steer & drift angles
figure
subplot(211)
plot(mlts.s, [ssres.steerAngle]*180/pi, 'LineWidth', 1)
ylabel('Steer angle (deg)')
xlim(mlts.s([1 end]))

subplot(212)
plot(mlts.s, [ssres.driftAngle]*180/pi, 'LineWidth', 1)
ylabel('Drift angle (deg)')
xlabel('Travelled distance (m)')
xlim(mlts.s([1 end]))