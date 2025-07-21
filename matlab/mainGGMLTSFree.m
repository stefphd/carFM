%% Main g-g MLTS
clc, clear

% g-g map
gg(1) = load('gg-2d'); % full G-G

% track
track = load('adria');

% options
opts = struct();
% opts.mex = true;
% opts.sRange = [4300 4800]; % bucine
% % x = [V, n, chi, at, an]
% opts.bcsFunc = @(xi, xf) [xi(1)-70.5; xf(1)-70.1; xi(2)-5.1; xf(2)-5.2];
% opts.numMeshPts = 500;

% call to ggmlts
mltsout = carfm.ggmltsfree(gg, track, opts);

% save
% save('ggmlts-fm-free.mat','-struct','mltsout');

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

alpha = mltsout.alpha;
alpha(mltsout.alpha>pi/2) = pi - mltsout.alpha(mltsout.alpha>pi/2);
alpha(mltsout.alpha<-pi/2) = - pi - mltsout.alpha(mltsout.alpha<-pi/2);

figure
scatter(mltsout.V, alpha, 100, mltsout.rho./mltsout.rhomax,'Marker','.');
box on
colormap parula
cc = colorbar;
ylim([-pi/2 pi/2])
xlabel('V (m/s)')
ylabel('alpha (rad)')
cc.Label.String = 'rho/rhomax';

figure
scatter(mltsout.geq/9.806, alpha, 100, mltsout.rho./mltsout.rhomax,'Marker','.');
box on
colormap parula
cc = colorbar;
ylim([-pi/2 pi/2])
xlabel('gz (g)')
ylabel('alpha (rad)')
cc.Label.String = 'rho/rhomax';

figure
scatter(mltsout.V, mltsout.geq/9.806, 100, mltsout.rho./mltsout.rhomax,'Marker','.');
box on
colormap parula
cc = colorbar;
xlabel('V (m/s)')
ylabel('gz (g)')
cc.Label.String = 'rho/rhomax';
