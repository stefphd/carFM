%% Main g-g map computation
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
opts.mex = true; % mex function to speed up
opts.GGshift = @(V) -3e-5*V^2; % g-g shift function handle

% call to ggmap
res = carfm.ggmap(car, V0, g0, opts);

% save gg
% save('gg-fm.mat','-struct','res');

% g-g plot
if ~(numel(V0)==1 && numel(g0)>1)
    for k = 1 : numel(g0)
        figure
        hold on
        VV = repmat(res.V, [numel(res.alpha), 1]);
        if numel(V0) > 1
            surf(res.an(:,:,k)/9.806, res.at(:,:,k)/9.806, VV,...
                'LineWidth',0.1);
            colormap jet
            zlabel('Speed (m/s)');
            daspect([1 1 30])
            view([45 20])
        else
            plot(res.an(:,:,k)/9.806, res.at(:,:,k)/9.806, ...
                 'LineWidth',1);
            axis equal
        end
        hold off
        xlabel('Lat acc (g)'), ylabel('Long acc (g)')
        box on
        ylim([-2 1.5])
        xlim([-2 2])
        title(sprintf('g_z/g = %.2f', g0(k)));
    end
else % numel(V0)==1 && numel(g0)>1
    figure
    hold on
    gg = repmat(res.g, [numel(res.alpha), 1]);
    an = reshape(res.an(:,1,:), [numel(res.alpha), numel(res.g)]);
    at = reshape(res.at(:,1,:), [numel(res.alpha), numel(res.g)]);
    contour(an/9.806, at/9.806, gg,(g0-1)*(1-1e-9)+1,...
        'LineWidth',1);
    colormap jet
    c = colorbar;
    c.Label.String = 'g_z/g';
    xlabel('Lat acc (g)'), ylabel('Long acc (g)')
    box on
    ylim([-1.9 1.4])
    axis equal
    xlim([-1.9 1.9])
    hold off
end

rmpath('data');