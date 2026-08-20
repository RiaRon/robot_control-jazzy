function handles = addPhaseBands(ax, timeSec, phases, style)
%ADDPHASEBANDS Shade contiguous profile phases using fixed colors.
handles = gobjects(0, 1);
valid = isfinite(timeSec);
timeSec = timeSec(valid);
phases = phases(valid);
if isempty(timeSec)
    return;
end
limits = ylim(ax);
changeStarts = [1; find(phases(2:end) ~= phases(1:end-1)) + 1];
changeEnds = [changeStarts(2:end) - 1; numel(phases)];
seen = strings(0, 1);
for segmentIndex = 1:numel(changeStarts)
    phaseName = phases(changeStarts(segmentIndex));
    colorIndex = find(style.phase_names == phaseName, 1);
    if isempty(colorIndex)
        colorIndex = find(style.phase_names == "unlabeled", 1);
    end
    x0 = timeSec(changeStarts(segmentIndex));
    x1 = timeSec(changeEnds(segmentIndex));
    if changeEnds(segmentIndex) < numel(timeSec)
        x1 = 0.5 * (x1 + timeSec(changeEnds(segmentIndex) + 1));
    end
    visibility = 'off';
    if ~any(seen == phaseName)
        visibility = 'on';
        seen(end + 1, 1) = phaseName; %#ok<AGROW>
    end
    patchHandle = patch(ax, [x0, x1, x1, x0], ...
        [limits(1), limits(1), limits(2), limits(2)], ...
        style.phase_colors_rgb(colorIndex, :), ...
        'FaceAlpha', 0.12, 'EdgeColor', 'none', ...
        'DisplayName', char("phase: " + phaseName), ...
        'HandleVisibility', visibility);
    if strcmp(visibility, 'on')
        handles(end + 1, 1) = patchHandle; %#ok<AGROW>
    end
end
end
