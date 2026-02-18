function iterCallbackFree(iter, obj, infpr, infdu, sol)
%ITERCALLBACKFREE Callback called during GGMLTSFREE solving for debugging

persistent f

if isempty(f) || ~isgraphics(f, 'figure') % create fig if not valid handle
    f = figure;
    % V
    subplot(411)
    xlabel('t');
    ylabel('x');
    xlim(sol.time([1 end]))
    box on
    legend;
    % lambda, n
    subplot(412)
    xlabel('t');
    ylabel('x');
    xlim(sol.time([1 end]))
    box on
    legend;
    drawnow;
    % delta, Tau__t
    subplot(413)
    xlabel('t');
    ylabel('x');
    xlim(sol.time([1 end]))
    box on
    legend;
    drawnow;
    % f
    subplot(414)
    xlabel('t');
    ylabel('f');
    xlim(sol.time([1 end]))
    box on
    drawnow;
end

% set curr fig
figure(f);

% draw axis
subplot(411)
cla;
hold on
title(sprintf("Iter: %d, Obj: %g, Inf pr: %g, Inf du: %g", iter, obj, infpr, infdu))
plot(sol.time, sol.state(9,:), 'DisplayName', 'x9')
hold off
legend;

subplot(412)
cla;
hold on
plot(sol.time, sol.state(10,:), 'DisplayName', 'x10')
plot(sol.time, sol.state(24,:), 'DisplayName', 'x24')
hold off
legend;
box on;

subplot(413)
cla;
hold on
plot(sol.time, sol.state(4,:), 'DisplayName', 'x4')
plot(sol.time, sol.state(27,:), 'DisplayName', 'x27')
hold off
legend;

subplot(414)
cla;
hold on
plot(sol.time(1:end-1), sol.dyn_constraint(:,:)', 'DisplayName', 'f')
hold off

% draw immediately
drawnow

end