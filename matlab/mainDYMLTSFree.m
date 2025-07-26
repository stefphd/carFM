%% Main dynamic MLTS
clc, clear

% folder with functions and data
addpath('data')

% car data
car = load('data/myCarData');

% track
track = load('adria');

% options
opts = struct();
opts.mex = true;
opts.debugSolve = true;
opts.linearSolver = 'ma27';

% guess
% guess from ggmlts
carSS = load('data/myCarData');
gg = load('gg-fm'); % full G-G w/o RHA
guess = load('ggmlts-fm-free');
guess = carfm.ggmlts2ss(carSS, gg, guess);
% guess from ssa
% V0 = 50; % guess speed
% ssres = carfm.ssa(car,[V0; 0; 0],zeros(3,1));
% guess.s = [track.s(1), track.s(end)];
% guess.n = [0, 0];
% guess.chi = [0, 0];
% guess.data = [ssres, ssres];
% empty guess (perform ssa internally)
% guess = struct();

% % analysis of single turn using G-G mlts for boundary conditions
% opts.sRange = [4300 4800]; % bucine
% opts.numMeshPts = 500;
% [~, i1] = min(abs(guess.s-opts.sRange(1)));
% [~, i2] = min(abs(guess.s-opts.sRange(2)));
% [x1,~,u1] = carfm.dymlts.getDynVars(guess.data(i1));
% [x2,~,u2] = carfm.dymlts.getDynVars(guess.data(i2));
% % x = [ phi,   mu,    z,    delta,  ft,   sa,   VP, lambdaP, Vz, 
% %      deltadot, ftdot, sadot, Omegaz, Omegax, Omegay, 
% %      omegar, omegaf, alphar, alphaf, n, chi]
% % u = [tau, Taur, Tauf]
% opts.bcsFunc = @(xi, ui, xf, uf) [ ...
%     xi([1 2 3 5 6 7 16 17])-x1([1 2 3 5 6 7 16 17]);
%     xi(20)-guess.n(i1);
%     xf(20)-guess.n(i2);
%     ];

% call to dymlts
mltsout = carfm.dymltsfree(car, track, opts, guess);

% save
% save('dymlts-fm-free.mat','-struct','mltsout');

% data
dyres = mltsout.data;

% plots
figure
subplot(211)
plot(mltsout.s, mltsout.V*3.6,'LineWidth',1)
xlim(mltsout.s([1 end]));
ylabel('V (kph)');

subplot(212)
plot(mltsout.s, mltsout.n,'LineWidth',1)
xlim(mltsout.s([1 end]));
xlabel('s (m)');
ylabel('n (m)');

figure
subplot(211)
plot(mltsout.s, mltsout.at/9.806,'LineWidth',1)
xlim(mltsout.s([1 end]));
ylabel('a_t (g)');

subplot(212)
plot(mltsout.s, mltsout.an/9.806,'LineWidth',1)
xlim(mltsout.s([1 end]));
xlabel('s (m)');
ylabel('a_n (g)');

figure
pp = plot(track.x,track.y,'k-.',track.xl,track.yl,'k-',track.xr,track.yr,'k-', ...
    mltsout.x,mltsout.y,'b-','LineWidth',1);
set(gca,'YDir','reverse');
axis equal
customDatatips = [dataTipTextRow("s (m): ", mltsout.s)
                  dataTipTextRow("V (m/s): ", mltsout.V)
                  dataTipTextRow("n (m): ", mltsout.n)
                  dataTipTextRow("\chi (rad): ", mltsout.chi)
                  dataTipTextRow("a_t (m/s^2): ", mltsout.at)
                  dataTipTextRow("a_n (m/s^2): ", mltsout.an)];
pp(end).DataTipTemplate.DataTipRows = customDatatips;

figure
plot(mltsout.an/9.806,mltsout.at/9.806,'o','LineWidth',1)
axis equal
ylabel('a_t (g)');
xlabel('a_n (g)');