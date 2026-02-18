%% Main yaw-moment-diagram
clc, clear

% folder with functions and data
addpath('data')

% car data
car = load('data/myCarData');

% SS long inputs
u_long = [100/3.6 % speed (m/s)
          0 % tangential acceleration (m/s2)
          0 % total driving/braking torque (Nm)
          ];

% Steer and drift angle vectors
delta = linspace(-10,10,21)'/180*pi;
lambda = linspace(-5,5,21)'/180*pi;

% options
opts = struct();
opts.ssActiveLongInputs = [true, true, false]; % only two must be true
opts.useLastSSA = true;

% call to ymd
ymdres = carfm.ymd(car,u_long,delta,lambda,opts);

% remove added path
rmpath('data')

