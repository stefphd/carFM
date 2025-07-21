function iterCallbackFree(iter, obj, infpr, infdu, sol)
%ITERCALLBACKFREE Callback called during GGMLTSFREE solving for debugging

persistent f; % fig handle

if isempty(f) || ~isgraphics(f, 'figure') % create fig if not valid handle
    f = figure;
    subplot(211)
    xlabel('t');
    ylabel('x');
    xlim(sol.t([1 end]))
    box on
    legend;
    subplot(212)
    xlabel('t');
    ylabel('x');
    xlim(sol.t([1 end]))
    box on
    legend;
    drawnow;
end

% set curr fig
figure(f);

% draw axis
subplot(211)
cla;
hold on
title(sprintf("Iter: %d, Obj: %g, Inf pr: %g, Inf du: %g", iter, obj, infpr, infdu))
plot(sol.t, sol.x(1,:), 'DisplayName', 'x1')
plot(sol.t, sol.x(2,:), 'DisplayName', 'x2')
hold off

subplot(212)
cla;
hold on
plot(sol.t, sol.x(4,:), 'DisplayName', 'x4')
plot(sol.t, sol.x(5,:), 'DisplayName', 'x5')
hold off

% draw immediately
drawnow

end

