function run = read_pose_follow_json(filePath, experimentName)
%READ_POSE_FOLLOW_JSON Normalize legacy and extended pose-follow JSON.
%   RUN = READ_POSE_FOLLOW_JSON(FILEPATH) reads the file with
%   JSONDECODE(FILEREAD(...)).  The returned structure has dense numeric
%   arrays for plotting and statistics while preserving source metadata.
%
%   The 2026-08-18 real files and the deterministic diagnostic files both
%   use schema_version 1.  They are distinguished by feature fields rather
%   than by assuming that a version number uniquely identifies the trace.

filePath = char(string(filePath));
if nargin < 2 || strlength(string(experimentName)) == 0
    [~, experimentName] = fileparts(filePath);
end
experimentName = char(string(experimentName));

% This is intentionally the only JSON loading path.  It keeps the analyzer
% independent of Python and of the robot_control runtime.
raw = jsondecode(fileread(filePath));

required = {'kind', 'joint_names', 'result', 'trace'};
for index = 1:numel(required)
    if ~isfield(raw, required{index})
        error('posefollow:InvalidJSON', ...
            'Missing required top-level field "%s" in %s.', ...
            required{index}, filePath);
    end
end
if ~strcmp(string(raw.kind), "pose_follow_diagnostics")
    error('posefollow:InvalidKind', ...
        'Expected pose_follow_diagnostics, got "%s" in %s.', ...
        string(raw.kind), filePath);
end

trace = raw.trace(:);
sampleCount = numel(trace);

jointNames = string(raw.joint_names(:));
jointCount = numel(jointNames);
layerNames = [ ...
    "live_marker_to_measured"; ...
    "accepted_marker_to_measured"; ...
    "marker_update_staleness"; ...
    "accepted_marker_to_ik_target"; ...
    "ik_target_to_command"; ...
    "command_to_measured"];

timeSec = nan(sampleCount, 1);
ikSequence = nan(sampleCount, 1);
continuityCost = nan(sampleCount, 1);
rawPhase = strings(sampleCount, 1);
positionErrorM = nan(sampleCount, numel(layerNames));
signedProjectionM = nan(sampleCount, numel(layerNames));
orientationErrorRad = nan(sampleCount, numel(layerNames));
ikTargetRad = nan(sampleCount, jointCount);
commandRad = nan(sampleCount, jointCount);
measuredRad = nan(sampleCount, jointCount);

for sampleIndex = 1:sampleCount
    sample = trace(sampleIndex);
    timeSec(sampleIndex) = scalarField(sample, 'elapsed_sec');
    ikSequence(sampleIndex) = scalarField(sample, 'ik_sequence');
    continuityCost(sampleIndex) = scalarField(sample, 'ik_continuity_cost');

    if isfield(sample, 'diagnostic_profile') && ...
            isstruct(sample.diagnostic_profile) && ...
            isfield(sample.diagnostic_profile, 'phase')
        rawPhase(sampleIndex) = string(sample.diagnostic_profile.phase);
    else
        rawPhase(sampleIndex) = "unlabeled";
    end

    for layerIndex = 1:numel(layerNames)
        fieldName = char(layerNames(layerIndex));
        positionErrorM(sampleIndex, layerIndex) = ...
            nestedScalar(sample, 'position_error_m', fieldName);
        orientationErrorRad(sampleIndex, layerIndex) = ...
            nestedScalar(sample, 'orientation_error_rad', fieldName);
        signedProjectionM(sampleIndex, layerIndex) = ...
            nestedScalar(sample, ...
                'position_error_signed_projection_m', fieldName);
    end

    ikTargetRad(sampleIndex, :) = nestedVector( ...
        sample, jointCount, 'joint_positions_rad', 'ik_target');
    commandRad(sampleIndex, :) = nestedVector( ...
        sample, jointCount, 'joint_positions_rad', 'command');
    measuredRad(sampleIndex, :) = nestedVector( ...
        sample, jointCount, 'joint_positions_rad', 'measured');

    % Legacy real traces do not store signed projections.  Reconstruct the
    % exact quantities from their five TCP positions using the same vector
    % definitions as the Jazzy serializer.
    if any(~isfinite(signedProjectionM(sampleIndex, :)))
        signedProjectionM(sampleIndex, :) = reconstructProjections( ...
            sample, layerNames, signedProjectionM(sampleIndex, :));
    end
end

phase = strings(sampleCount, 1);
for sampleIndex = 1:sampleCount
    phase(sampleIndex) = canonicalPhase(rawPhase(sampleIndex));
end

schemaVersion = scalarField(raw, 'schema_version');
hasExtendedTrace = sampleCount > 0 && ( ...
    isfield(trace(1), 'position_error_signed_projection_m') || ...
    isfield(trace(1), 'diagnostic_profile'));
hasEventTiming = isfield(raw.result, 'ik') && ...
    isfield(raw.result.ik, 'events');
hasJumpEvents = isfield(raw.result, 'ik_target_jumps');
if hasExtendedTrace || hasEventTiming || hasJumpEvents
    schemaVariant = "extended";
else
    schemaVariant = "legacy-2026-08-18";
end

ik = normalizeIk(raw.result, jointNames);
jumps = normalizeJumps(raw.result, jointNames);
refusal = normalizeRefusal(raw.result, jointCount);

run = struct();
run.experiment = string(experimentName);
run.source_file = string(java.io.File(filePath).getCanonicalPath());
run.schema_version = schemaVersion;
run.schema_variant = schemaVariant;
run.group = string(fieldOr(raw, 'group', ""));
run.profile = string(fieldOr(raw, 'profile', ""));
run.joint_names = jointNames;
run.layer_names = layerNames;
run.time_sec = timeSec;
run.raw_phase = rawPhase;
run.phase = phase;
run.ik_sequence = ikSequence;
run.continuity_cost = continuityCost;
run.position_error_m = positionErrorM;
run.position_error_signed_projection_m = signedProjectionM;
run.orientation_error_rad = orientationErrorRad;
run.joint_positions_rad = struct( ...
    'ik_target', ikTargetRad, ...
    'command', commandRad, ...
    'measured', measuredRad);
run.ik = ik;
run.ik_target_jumps = jumps;
run.termination = string(fieldOr(raw.result, 'termination', "unknown"));
run.is_partial = logicalField(raw.result, 'is_partial', false);
run.refusal = refusal;
run.settings = fieldOr(raw, 'settings', struct());
run.ready_posture = normalizeReady(raw, jointCount);
run.result_metadata = rmfieldIfPresent(raw.result, { ...
    'position_error_m', 'position_error_signed_projection_m', ...
    'orientation_error_rad', 'per_joint', 'ik', 'ik_target_jumps'});
end


function value = logicalField(parent, name, fallback)
value = fallback;
if isstruct(parent) && isfield(parent, name)
    candidate = parent.(name);
    if (islogical(candidate) || isnumeric(candidate)) && ...
            isscalar(candidate) && ~isempty(candidate)
        value = logical(candidate);
    end
end
end


function value = fieldOr(parent, name, fallback)
if isstruct(parent) && isfield(parent, name)
    value = parent.(name);
else
    value = fallback;
end
end


function value = scalarField(parent, name)
value = nan;
if isstruct(parent) && isfield(parent, name)
    candidate = parent.(name);
    if isnumeric(candidate) && isscalar(candidate) && ~isempty(candidate)
        value = double(candidate);
    end
end
end


function value = nestedScalar(parent, containerName, fieldName)
value = nan;
if ~isstruct(parent) || ~isfield(parent, containerName)
    return;
end
container = parent.(containerName);
if ~isstruct(container) || ~isfield(container, fieldName)
    return;
end
candidate = container.(fieldName);
if isnumeric(candidate) && isscalar(candidate) && ~isempty(candidate)
    value = double(candidate);
end
end


function vector = nestedVector(parent, width, containerName, fieldName)
vector = nan(1, width);
if ~isstruct(parent) || ~isfield(parent, containerName)
    return;
end
container = parent.(containerName);
if ~isstruct(container) || ~isfield(container, fieldName)
    return;
end
candidate = double(container.(fieldName));
candidate = candidate(:).';
if numel(candidate) == width
    vector = candidate;
end
end


function projections = reconstructProjections(sample, layerNames, projections)
positions = struct();
names = {'live_marker', 'accepted_marker', 'ik_target', 'command', 'measured'};
for index = 1:numel(names)
    positions.(names{index}) = nestedVector( ...
        sample, 3, 'tcp_positions_m', names{index});
end
if any(~isfinite(positions.live_marker)) || ...
        any(~isfinite(positions.measured))
    return;
end

liveVector = positions.live_marker - positions.measured;
liveNorm = norm(liveVector);
if liveNorm <= 1e-12
    direction = zeros(1, 3);
else
    direction = liveVector ./ liveNorm;
end

vectors = struct();
vectors.live_marker_to_measured = liveVector;
vectors.marker_update_staleness = ...
    positions.live_marker - positions.accepted_marker;
vectors.accepted_marker_to_ik_target = ...
    positions.accepted_marker - positions.ik_target;
vectors.ik_target_to_command = positions.ik_target - positions.command;
vectors.command_to_measured = positions.command - positions.measured;

for index = 1:numel(layerNames)
    name = char(layerNames(index));
    if isfield(vectors, name) && all(isfinite(vectors.(name)))
        projections(index) = dot(vectors.(name), direction);
    end
end
end


function phase = canonicalPhase(rawPhase)
rawPhase = lower(string(rawPhase));
if rawPhase == "origin_hold" || rawPhase == "complete"
    phase = "origin-hold";
elseif contains(rawPhase, "ramp_back")
    phase = "return";
elseif contains(rawPhase, "ramp_out")
    phase = "ramp";
elseif endsWith(rawPhase, "_hold")
    phase = "hold";
elseif strlength(rawPhase) == 0 || rawPhase == "unlabeled"
    phase = "unlabeled";
else
    phase = rawPhase;
end
end


function ik = normalizeIk(result, jointNames)
ikRaw = fieldOr(result, 'ik', struct());
ik = struct();
ik.submitted = scalarField(ikRaw, 'submitted');
ik.accepted = scalarField(ikRaw, 'succeeded');
ik.failed = scalarField(ikRaw, 'failed');
ik.superseded = scalarField(ikRaw, 'superseded');
ik.solve_attempts = scalarField(ikRaw, 'solve_attempts');
ik.candidate_count = scalarField(ikRaw, 'candidate_count');
ik.rejected_candidate_count = scalarField(ikRaw, 'rejected_candidate_count');
ik.continuity_rejected = scalarField(ikRaw, 'continuity_rejected');
ik.continuity_retries = scalarField(ikRaw, 'continuity_retries');
ik.continuity_exhausted = scalarField(ikRaw, 'continuity_exhausted');
ik.selection_events = struct( ...
    'sequence', {}, 'phase', {}, 'candidate_count', {}, ...
    'rejected_candidate_count', {}, 'selected_candidate', {}, ...
    'selected_cost', {}, 'solve_latency_sec', {}, 'batch_latency_sec', {});
if isfield(ikRaw, 'selection_events') && isstruct(ikRaw.selection_events)
    rawSelections = ikRaw.selection_events(:);
    selections = repmat(ik.selection_events, numel(rawSelections), 1);
    for selectionIndex = 1:numel(rawSelections)
        selection = rawSelections(selectionIndex);
        selections(selectionIndex).sequence = scalarField(selection, 'sequence');
        selections(selectionIndex).phase = string(fieldOr(selection, 'profile_phase', "unknown"));
        selections(selectionIndex).candidate_count = scalarField(selection, 'candidate_count');
        selections(selectionIndex).rejected_candidate_count = scalarField(selection, 'rejected_candidate_count');
        selections(selectionIndex).selected_candidate = scalarField(selection, 'selected_candidate');
        selections(selectionIndex).selected_cost = scalarField(selection, 'selected_cost');
        selections(selectionIndex).solve_latency_sec = scalarField(selection, 'solve_latency_sec');
        selections(selectionIndex).batch_latency_sec = scalarField(selection, 'batch_latency_sec');
    end
    ik.selection_events = selections;
end
ik.events = struct( ...
    'sequence', {}, 'outcome', {}, 'requested_sec', {}, ...
    'started_sec', {}, 'completed_sec', {}, 'accepted_sec', {}, ...
    'request_to_complete_sec', {}, 'request_to_accepted_sec', {}, ...
    'phase', {});

if ~isfield(ikRaw, 'events') || ~isstruct(ikRaw.events)
    ik.event_timing_available = false;
    ik.joint_names = jointNames;
    return;
end
eventsRaw = ikRaw.events(:);
events = repmat(ik.events, numel(eventsRaw), 1);
for index = 1:numel(eventsRaw)
    event = eventsRaw(index);
    events(index).sequence = scalarField(event, 'sequence');
    events(index).outcome = string(fieldOr(event, 'outcome', "unknown"));
    events(index).requested_sec = scalarField(event, 'requested_elapsed_sec');
    events(index).started_sec = scalarField(event, 'started_elapsed_sec');
    events(index).completed_sec = scalarField(event, 'completed_elapsed_sec');
    events(index).accepted_sec = scalarField(event, 'accepted_elapsed_sec');
    events(index).request_to_complete_sec = ...
        scalarField(event, 'request_to_complete_sec');
    events(index).request_to_accepted_sec = ...
        scalarField(event, 'request_to_accepted_sec');
    events(index).phase = "unassigned";
end
ik.events = events;
ik.event_timing_available = true;
ik.joint_names = jointNames;
end


function ready = normalizeReady(raw, jointCount)
ready = struct('name', "", 'target_rad', nan(1, jointCount), ...
    'actual_start_rad', nan(1, jointCount), ...
    'start_error_rad', nan(1, jointCount), 'passed', false, ...
    'available', false);
settings = fieldOr(raw, 'settings', struct());
settingReady = fieldOr(settings, 'ready_posture', struct());
result = fieldOr(raw, 'result', struct());
resultReady = fieldOr(result, 'ready_posture', struct());
if isstruct(settingReady) && isfield(settingReady, 'name')
    ready.name = string(settingReady.name);
    ready.target_rad = vectorField(settingReady, 'target_rad', jointCount);
    ready.available = true;
end
if isstruct(resultReady)
    if isfield(resultReady, 'name')
        ready.name = string(resultReady.name);
        ready.available = true;
    end
    ready.actual_start_rad = vectorField(resultReady, 'actual_start_rad', jointCount);
    ready.start_error_rad = vectorField(resultReady, 'start_error_rad', jointCount);
    ready.passed = logicalField(resultReady, 'passed', false);
end
end


function vector = vectorField(parent, name, width)
vector = nan(1, width);
if isstruct(parent) && isfield(parent, name) && isnumeric(parent.(name))
    candidate = double(parent.(name));
    candidate = candidate(:).';
    if numel(candidate) == width
        vector = candidate;
    end
end
end


function refusal = normalizeRefusal(result, jointCount)
refusal = struct( ...
    'available', false, 'reason', "", 'message', "", ...
    'refused_sequence', nan, 'profile_phase', "", ...
    'attempts', nan, 'max_attempts', nan, ...
    'reference_sequence', nan, ...
    'joint_delta_rad', nan(1, jointCount), ...
    'triggered_joints', {{}});
if ~isstruct(result) || ~isfield(result, 'refusal') || ...
        ~isstruct(result.refusal) || isempty(result.refusal)
    return;
end
raw = result.refusal;
refusal.available = true;
refusal.reason = string(fieldOr(raw, 'reason', "unknown"));
refusal.message = string(fieldOr(raw, 'message', ""));
refusal.refused_sequence = scalarField(raw, 'refused_sequence');
refusal.profile_phase = string(fieldOr(raw, 'profile_phase', "unknown"));
refusal.attempts = scalarField(raw, 'attempts');
refusal.max_attempts = scalarField(raw, 'max_attempts');
refusal.reference_sequence = scalarField(raw, 'reference_sequence');
if isfield(raw, 'joint_delta_rad') && isnumeric(raw.joint_delta_rad)
    delta = double(raw.joint_delta_rad(:)).';
    if numel(delta) == jointCount
        refusal.joint_delta_rad = delta;
    end
end
if isfield(raw, 'triggered_joints')
    refusal.triggered_joints = cellstr(string(raw.triggered_joints(:)));
end
end


function jumps = normalizeJumps(result, jointNames)
jointCount = numel(jointNames);
jumps = struct();
jumps.threshold_rad = nan;
jumps.transitions = nan;
jumps.times_sec = zeros(0, 1);
jumps.accepted_times_sec = zeros(0, 1);
jumps.from_sequence = zeros(0, 1);
jumps.to_sequence = zeros(0, 1);
jumps.joint_delta_rad = zeros(0, jointCount);
jumps.triggered_joints = cell(0, 1);
jumps.available = false;

if ~isfield(result, 'ik_target_jumps') || ...
        ~isstruct(result.ik_target_jumps)
    return;
end
jumpRaw = result.ik_target_jumps;
jumps.threshold_rad = scalarField(jumpRaw, 'threshold_rad');
jumps.transitions = scalarField(jumpRaw, 'transitions');
jumps.available = true;
if ~isfield(jumpRaw, 'events') || ~isstruct(jumpRaw.events)
    return;
end

events = jumpRaw.events(:);
eventCount = numel(events);
jumps.times_sec = nan(eventCount, 1);
jumps.accepted_times_sec = nan(eventCount, 1);
jumps.from_sequence = nan(eventCount, 1);
jumps.to_sequence = nan(eventCount, 1);
jumps.joint_delta_rad = nan(eventCount, jointCount);
jumps.triggered_joints = cell(eventCount, 1);
for index = 1:eventCount
    event = events(index);
    jumps.times_sec(index) = scalarField(event, 'observed_elapsed_sec');
    jumps.accepted_times_sec(index) = ...
        scalarField(event, 'accepted_elapsed_sec');
    jumps.from_sequence(index) = scalarField(event, 'from_sequence');
    jumps.to_sequence(index) = scalarField(event, 'to_sequence');
    if isfield(event, 'joint_delta_rad')
        delta = double(event.joint_delta_rad(:)).';
        if numel(delta) == jointCount
            jumps.joint_delta_rad(index, :) = delta;
        end
    end
    if isfield(event, 'triggered_joints')
        jumps.triggered_joints{index} = ...
            cellstr(string(event.triggered_joints(:)));
    else
        jumps.triggered_joints{index} = {};
    end
end
end


function output = rmfieldIfPresent(input, names)
output = input;
for index = 1:numel(names)
    if isfield(output, names{index})
        output = rmfield(output, names{index});
    end
end
end
