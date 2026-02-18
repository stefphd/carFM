%% Main g-g MLTS fixed
clc, clear

% g-g map
gg = load('gg-fm'); % full G-G

% trajectory
traj = load('ggmlts-fm-free');

% options
opts = struct();
opts.mex = true;
% opts.sRange = [4300 4800]; % bucine
% % x = [V, n, chi, at, an]
% opts.bcsFunc = @(xi, xf) [xi(1)-70.5; xf(1)-70.1; xi(2)-5.1; xf(2)-5.2];
% opts.numMeshPts = 500;

% call to ggmlts
mltsout = carfm.ggmltsfixed(gg, traj, opts);

% save
% save('ggmlts-fm-fixed.mat','-struct','mltsout');

% plots
figure
plot(mltsout.zeta, mltsout.V*3.6,'LineWidth',1)
xlim(mltsout.s([1 end]));
ylabel('V (kph)');

figure
subplot(211)
plot(mltsout.zeta, mltsout.at/9.806,'LineWidth',1)
xlim(mltsout.zeta([1 end]));
ylabel('a_t (g)');

subplot(212)
plot(mltsout.zeta, mltsout.an/9.806,'LineWidth',1)
xlim(mltsout.zeta([1 end]));
xlabel('s (m)');
ylabel('a_n (g)');

figure
plot(mltsout.an/9.806,mltsout.at/9.806,'o','LineWidth',1)
axis equal
ylabel('a_t (g)');
xlabel('a_n (g)');