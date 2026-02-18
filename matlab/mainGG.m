%% Main g-g map computation
clc,clear 

% folder with functions and data
addpath('data')

% bike data
car = load('data/myCarData');

% speed (m/s)
V0 = 60;

% total gravity (g)
g0 = 1.0;

% options
opts = struct();

% opts.mex = true; % mex function to speed up
% opts.usePrebuilt = true; % use prebuilt mex file
opts.GGshift = @(V) -3e-5*V^2; % g-g shift function handle
% opts.iSuspensionType = 0; % 0 for rigid suspension
% opts.iTyreType = [0 1 1]; % direction z, y, x, e.g. [0 1 1] for rigid tyres
% opts.suspTrimRigid = [0 0 0 0.53]; % suspension travels for rigid suspension (fl,fr,rl,rr)
% opts.tyreDeformationRigid = [0 0 0 0]; % vertical tyre deformations for rigid tyres (fl,fr,rl,rr)
opts.sensitivityPar = {'cgB','cgH', ...
                       'frontTyre.LMUX','frontTyre.LMUY','rearTyre.LMUX','rearTyre.LMUY', 'engine.Pmax'};

% call to ggmap
ggres = carfm.ggmap(car, V0, g0, opts);

% plot
figure
plot(ggres.an/9.806,ggres.at/9.806)
xlabel('a_n (g)'), ylabel('a_t (g)')
axis equal