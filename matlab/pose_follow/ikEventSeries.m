function [times, latencyMs, outcomes] = ikEventSeries(run)
%IKEVENTSERIES Select accepted latency, else completion latency, per request.
eventCount = numel(run.ik.events);
times = nan(eventCount, 1);
latencyMs = nan(eventCount, 1);
outcomes = strings(eventCount, 1);
for index = 1:eventCount
    event = run.ik.events(index);
    times(index) = event.requested_sec;
    latencyMs(index) = event.request_to_accepted_sec * 1000;
    if ~isfinite(latencyMs(index))
        latencyMs(index) = event.request_to_complete_sec * 1000;
    end
    outcomes(index) = event.outcome;
end
end
