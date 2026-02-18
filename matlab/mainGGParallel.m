 %% Main g-g map computation using parfor
clc,clear 
addpath('data');

% bike data
car = load('data/myCarData');

% speed (m/s)
V0 = 15 : 5 : 100;

% total gravity (g)
g0 = 0.6 : 0.1 : 1.4;

% options
opts = struct();
opts.mex = true; % mex function to speed up <- MEX true or false
opts.GGshift = @(V) -3e-5*V^2; % g-g shift function handle
opts.sensitivityPar = {'cgB','cgH', ...
                       'frontTyre.LMUX','frontTyre.LMUY','rearTyre.LMUX','rearTyre.LMUY', 'engine.Pmax'};

% run one gg to generate mex if any
opts.buildOnly = true; % build only mex if any
carfm.ggmap(car, V0(1), g0(1), opts);

% % run w/o parfor
% opts.buildOnly = false; % reset buildOnly to default
% opts.usePrebuilt = true; % use pre-built MEX function (if opts.mex=true used)
% tic;
% all_res = mltsfm.ggmap(bike, V0, g0, opts);
% toc;

% run parfor
N = 1; % number of grid points solved for each ggmap call
N = ceil(numel(V0)*numel(g0)/N);
[V,G] = ndgrid(V0,g0); % V,g grid
V = V(:); % vectorize V (col)
G = G(:); % vectorize g (col)
n = numel(V); % num of V,g combinations
G = [G; nan(ceil(n/N)*N-n, 1)];
V = [V; nan(ceil(n/N)*N-n, 1)];
% reshape to (:,N), with each column the g,V values for each worker
G = reshape(G,[numel(G)/N, N]);
V = reshape(V,[numel(G)/N, N]);
% options
opts.buildOnly = false; % reset buildOnly to default
opts.usePrebuilt = true; % use pre-built MEX function (if opts.mex=true used)
% run parfor
res = cell(N,1);
tic;
parfor i = 1:N
    % input args
    g0i = G(:,i);
    V0i = V(:,i);
    % rem nan
    g0i = g0i(~isnan(g0i)); 
    V0i = V0i(~isnan(V0i)); 
    % call to ggmap
    res{i} = carfm.ggmap(car, V0i, g0i, opts);
end
toc;
% cat res
all_res = [];
for i = 1 : numel(res)
    all_res = [all_res; res{i}];
end

% At the end merge all_res to obtain the full GG
res = carfm.ssa.ggmerge(all_res);

% save gg
% save('gg-fm.mat','-struct','res');

rmpath('data');