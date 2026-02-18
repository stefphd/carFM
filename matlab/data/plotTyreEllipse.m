%% Plot the tyre friction ellipse
clc, clear

% data
tyre = load('myCarData.mat').frontTyre;
% tyre = load('myCarData.mat').rearTyre;

ca = 0;
N = 1000;
Vs = 60;
alpha = linspace(-0.25, 0.25, 101);
kappa = linspace(-0.25, 0.25, 101);

% Grid
[A,K] = meshgrid(alpha,kappa);
FX = 0*A;
FY = 0*A;

% Calc
for ik = 1 : numel(kappa)
    for ia = 1 : numel(alpha)
        Vn = Vs*tan(alpha(ia));
        Vr = Vs*(1+kappa(ik));
        Omega = Vr/tyre.UNLOADED_RADIUS;
        [FX(ik,ia),FY(ik,ia)] = tyre.Forces(tyre, N, Vs, Vn, Vr, Omega, ca);
    end
end
% envelope
kk = boundary(FX(:),FY(:));
FX0 = FX(kk);
FY0 = FY(kk);

% Plot
figure
hold on
plot(FY/N,FX/N,'r-');
plot(FY'/N,FX'/N,'b-');
plot(FY0/N,FX0/N,'k-','LineWidth',2);
xlabel('Fy/N'), ylabel('Fx/N')
box on
axis equal