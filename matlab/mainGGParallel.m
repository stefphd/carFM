%% Main g-g map computation using parfor
clc,clear 
addpath('data');

% bike data
car = load('data/myCarData');

% speed (m/s)
V0 = 10 : 5 : 105;

% total gravity (g)
g0 = 0.6 : 0.1 : 1.4;

% options
opts = struct();
% opts.mex = true; % mex function to speed up <- MEX true or false
opts.GGshift = @(V) -3e-5*V^2; % g-g shift function handle

% run one gg to generate mex if any
opts.buildOnly = true; % build only mex if any
carfm.ggmap(car, V0(1), g0(1), opts);

% % run w/o parfor
% opts.buildOnly = false; % reset buildOnly to default
% opts.usePrebuilt = true; % use pre-built MEX function (if opts.mex=true used)
% tic;
% all_res = carfm.ggmap(bike, V0, g0, opts);
% toc;

% % run parfor (loop over g)
% opts.buildOnly = false; % reset buildOnly to default
% opts.usePrebuilt = true; % use pre-built MEX function (if opts.mex=true used)
% tic;
% parfor ig = 1:numel(g0)
%     all_res(ig) = carfm.ggmap(bike, V0, g0(ig), opts);
% end
% toc;

% % run parfor (loop over V)
% opts.buildOnly = false; % reset buildOnly to default
% opts.usePrebuilt = true; % use pre-built MEX function (if opts.mex=true used)
% tic;
% parfor iV = 1:numel(V0)
%     all_res(iV) = carfm.ggmap(bike, V0(iV), g0, opts);
% end
% toc;

% loop over g-V grid points using 'spmd'
N = 8; % num of workers
[Vall,gall] = ndgrid(V0,g0); % V,g grid
Vall = Vall(:); % vectorize V (col)
gall = gall(:); % vectorize g (col)
nall = numel(Vall); % num of V,g combinations
% add nan at the end so that numel is a multiple of N
gall = [gall; nan(ceil(nall/N)*N-nall, 1)];
Vall = [Vall; nan(ceil(nall/N)*N-nall, 1)];
% reshape to (:,N), with each column the g,V values for each worker
gall = reshape(gall,[numel(gall)/N, N]);
Vall = reshape(Vall,[numel(gall)/N, N]);
% options
opts.buildOnly = false; % reset buildOnly to default
opts.usePrebuilt = true; % use pre-built MEX function (if opts.mex=true used)
% run matlab workers
delete(gcp('nocreate')); % shut-down existing pool (if any)
parpool(N); % initialize pool w/ N workers
tic;
spmd(N) % create N matlab workers
    i = spmdIndex; % spmdIndex is the index of matlab worker
    g0i = gall(:,i);
    V0i = Vall(:,i);
    g0i = g0i(~isnan(g0i)); % rem nan
    V0i = V0i(~isnan(V0i)); % rem nan
    worker_res = carfm.ggmap(car, V0i, g0i, opts); % run ggmap
end
toc;
% cat res
all_res = [];
for i = 1 : numel(worker_res)
    all_res = [all_res; worker_res{i}]; % worker_res{i} is res for i-th worker (just inspect worker_res)
end

% At the end merge all_res to obtain the full g-g
res = carfm.ssa.ggmerge(all_res);

% save gg
% save('gg-fm.mat','-struct','res');

rmpath('data');
