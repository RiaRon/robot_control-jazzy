function figureHandle = plotIkEvents(runs, style, visible)
%PLOTIKEVENTS Plot latency and accepted/failed/superseded/refused counts.
figureHandle = newFigure('IK latency and outcomes', visible, numel(runs));
layout = tiledlayout(figureHandle, numel(runs), 3, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, ...
    'IK request latency, continuity cost, candidate outcomes, and profile phase', ...
    'Interpreter', 'none');
latencyLimit = 0;
costLimit = 0;
countLimit = 0;
for index = 1:numel(runs)
    [~, eventLatencyMs] = ikEventSeries(runs(index));
    latencyLimit = max(latencyLimit, maxFinite(eventLatencyMs));
    costLimit = max(costLimit, maxFinite(runs(index).continuity_cost));
    countLimit = max(countLimit, maxFinite([runs(index).ik.accepted, ...
        runs(index).ik.failed, runs(index).ik.superseded, ...
        runs(index).ik.continuity_exhausted]));
end
latencyLimit = positiveLimit(latencyLimit);
costLimit = positiveLimit(costLimit);
countLimit = max(1, ceil(countLimit * 1.15));
timeLimit = globalTimeLimit(runs);
outcomeColors = [ ...
    0.4660, 0.6740, 0.1880; ...
    0.6350, 0.0780, 0.1840; ...
    0.9290, 0.6940, 0.1250; ...
    0.4940, 0.1840, 0.5560];

for index = 1:numel(runs)
    run = runs(index);
    ax = nexttile(layout);
    configureTimeAxes(ax, timeLimit, [0, latencyLimit]);
    phaseHandles = addPhaseBands(ax, run.time_sec, run.phase, style);
    [eventTimes, latencyMs, outcomes] = ikEventSeries(run);
    eventHandles = gobjects(0, 1);
    eventLabels = ["accepted"; "failed"; "superseded"; "refused"];
    for outcomeIndex = 1:numel(eventLabels)
        if eventLabels(outcomeIndex) == "accepted"
            mask = outcomes == "accepted";
        else
            mask = contains(outcomes, eventLabels(outcomeIndex));
        end
        if any(mask)
            eventHandles(end + 1, 1) = scatter(ax, eventTimes(mask), ... %#ok<AGROW>
                latencyMs(mask), 28, outcomeColors(outcomeIndex, :), 'filled', ...
                'DisplayName', char(eventLabels(outcomeIndex)));
        end
    end
    if isempty(eventHandles)
        text(ax, 0.5, 0.55, 'IK event timing not recorded in this schema', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center', ...
            'Interpreter', 'none');
        if ~isempty(phaseHandles)
            legend(ax, phaseHandles, 'Location', 'best', 'Interpreter', 'none');
        end
    else
        legend(ax, [eventHandles; phaseHandles], 'Location', 'best', ...
            'Interpreter', 'none');
    end
    title(ax, run.experiment + " — request latency", 'Interpreter', 'none');
    xlabel(ax, 'Request elapsed time (s)');
    ylabel(ax, 'IK latency (ms)');

    ax = nexttile(layout);
    configureTimeAxes(ax, timeLimit, [0, costLimit]);
    addPhaseBands(ax, run.time_sec, run.phase, style);
    validCost = isfinite(run.time_sec) & isfinite(run.continuity_cost);
    if any(validCost)
        plot(ax, run.time_sec(validCost), run.continuity_cost(validCost), ...
            'Color', style.experiment_colors_rgb(index, :), 'LineWidth', 1.1);
    else
        text(ax, 0.5, 0.55, 'Continuity cost not recorded in this schema', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center', ...
            'Interpreter', 'none');
    end
    title(ax, run.experiment + " — selected continuity cost", 'Interpreter', 'none');
    xlabel(ax, 'Elapsed time (s)');
    ylabel(ax, 'Weighted joint distance');

    ax = nexttile(layout);
    counts = [run.ik.accepted, run.ik.failed, run.ik.superseded, ...
        run.ik.continuity_exhausted];
    counts(~isfinite(counts)) = 0;
    bars = bar(ax, counts, 'FaceColor', 'flat');
    bars.CData = outcomeColors;
    grid(ax, 'on');
    ylim(ax, [0, countLimit]);
    xticks(ax, 1:4);
    xticklabels(ax, {'accepted', 'failed', 'superseded', 'refused'});
    ylabel(ax, 'IK event count');
    title(ax, sprintf('%s — submitted %g', run.experiment, run.ik.submitted), ...
        'Interpreter', 'none');
end
end
