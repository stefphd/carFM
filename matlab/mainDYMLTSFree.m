%% Main dynamic MLTS
clc, clear

% folder with functions and data
addpath('data')

% car data
car = load('data/myCarData');

% track
track = load('mugello.mat');

% options
opts = struct();
opts.mex = true;
opts.debugSolve = true;
opts.linearSolver = 'ma27';
opts.numThreads = 8;
% opts.refineSolution = true;
% opts.usePrebuilt = true;

% call to dymlts
mltsout = carfm.dymltsfree(car, track, opts);

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