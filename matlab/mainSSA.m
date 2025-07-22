%% Main steady-state analysis
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

% SS lat inputs
u_lat = [0 % yaw rate (rad/s)
         0 % steering angle (rad)
         0 % drift angle (rad)
         ];

% options
opts = struct();
opts.ssActiveLongInputs = [true, true, false]; % only two must be true
opts.ssActiveLatInputs = [true, false, false]; % only one must be true
% opts.mex = true; % mex function to speed up
% opts.usePrebuilt = true; % use prebuilt mex file
% opts.iSuspensionType = 1; % 0 for rigid suspension
% opts.iTyreType = [1 1 1]; % direction z, y, x, e.g. [0 1 1] for rigid tyres
% opts.suspTrimRigid = [0 0 0 0.5]; % suspension travels for rigid suspension (fl,fr,rl,rr)
% opts.tyreDeformationRigid = [0 0 0 0]; % vertical tyre deformations for rigid tyres (fl,fr,rl,rr)

% call to ssa
ssres = carfm.ssa(car,u_long,u_lat,opts);

% unfold ssa output
carfm.unfold(ssres)

% remove added path
rmpath('data')