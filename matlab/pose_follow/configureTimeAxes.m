function configureTimeAxes(ax, timeLimit, yLimit)
%CONFIGURETIMEAXES Apply shared limits before profile-phase shading.
hold(ax, 'on');
grid(ax, 'on');
box(ax, 'on');
xlim(ax, timeLimit);
ylim(ax, yLimit);
end
