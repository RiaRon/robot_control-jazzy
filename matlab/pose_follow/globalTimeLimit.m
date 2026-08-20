function limit = globalTimeLimit(runs)
%GLOBALTIMELIMIT Shared elapsed-time axis for cross-experiment plots.
maximum = 0;
minimum = inf;
for index = 1:numel(runs)
    values = runs(index).time_sec(isfinite(runs(index).time_sec));
    if ~isempty(values)
        minimum = min(minimum, min(values));
        maximum = max(maximum, max(values));
    end
end
if ~isfinite(minimum)
    minimum = 0;
end
if maximum <= minimum
    maximum = minimum + 1;
end
limit = [min(0, minimum), maximum];
end
