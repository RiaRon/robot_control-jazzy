function analysis = analyze_pose_follow(inputFiles, outputDir, varargin)
%ANALYZE_POSE_FOLLOW Build a reproducible pose-follow analysis bundle.
%   ANALYSIS = ANALYZE_POSE_FOLLOW(INPUTFILES, OUTPUTDIR) reads one or more
%   pose-follow JSON files without modifying them and writes:
%     summary.csv, analysis_summary.json, analysis.mat,
%     tcp_error_timeseries.png, error_layers.png, joint_tracking.png,
%     j1_j4_j7_detail.png, ik_events.png, phase_comparison.png, and
%     research_report.pdf.
%
%   INPUTFILES can be a character vector, string array, or cell array.
%   Name-value options:
%     'ExperimentNames' - labels in the same order as INPUTFILES
%     'Visible'         - 'off' (default) or 'on'
%     'CreatePDF'       - true (default) or false

%   This analyzer uses only base MATLAB functionality.  It does not import
%   robot_control code and never writes to an input JSON file.

parser = inputParser;
parser.FunctionName = 'analyze_pose_follow';
addRequired(parser, 'inputFiles');
addRequired(parser, 'outputDir', @(value) strlength(string(value)) > 0);
addParameter(parser, 'ExperimentNames', strings(0, 1));
addParameter(parser, 'Visible', 'off', ...
    @(value) any(strcmpi(string(value), ["on", "off"])));
addParameter(parser, 'CreatePDF', true, ...
    @(value) islogical(value) && isscalar(value));
parse(parser, inputFiles, outputDir, varargin{:});

files = normalizeFileList(parser.Results.inputFiles);
givenNames = parser.Results.ExperimentNames;
if ischar(givenNames)
    names = string(givenNames);
else
    names = string(givenNames(:));
end
if isempty(names)
    names = strings(numel(files), 1);
    for index = 1:numel(files)
        [~, stem] = fileparts(files{index});
        names(index) = string(stem);
    end
elseif numel(names) ~= numel(files)
    error('posefollow:NameCount', ...
        'ExperimentNames has %d entries for %d input files.', ...
        numel(names), numel(files));
end
if numel(unique(names)) ~= numel(names)
    error('posefollow:DuplicateNames', 'ExperimentNames must be unique.');
end

outputDir = char(string(parser.Results.outputDir));
if ~isfolder(outputDir)
    mkdir(outputDir);
end
for index = 1:numel(files)
    if ~isfile(files{index})
        error('posefollow:MissingInput', 'Input JSON not found: %s', files{index});
    end
end

runsCell = cell(numel(files), 1);
for index = 1:numel(files)
    runsCell{index} = read_pose_follow_json(files{index}, names(index));
end
runs = vertcat(runsCell{:});

[summaryTable, experimentSummaries] = buildSummary(runs);
style = analysisStyle(numel(runs));
generatedUtc = string(datetime('now', 'TimeZone', 'UTC', ...
    'Format', 'yyyy-MM-dd''T''HH:mm:ssXXX'));

bundleNames = [ ...
    "summary.csv"; "analysis_summary.json"; "analysis.mat"; ...
    "tcp_error_timeseries.png"; "error_layers.png"; ...
    "joint_tracking.png"; "j1_j4_j7_detail.png"; ...
    "ik_events.png"; "phase_comparison.png"];
if parser.Results.CreatePDF
    bundleNames(end + 1) = "research_report.pdf";
end

analysis = struct();
analysis.generated_utc = generatedUtc;
analysis.generator = "analyze_pose_follow";
analysis.generator_contract = "jazzy-8a700c0-pose-follow";
analysis.output_directory = string(java.io.File(outputDir).getCanonicalPath());
analysis.bundle_files = bundleNames;
analysis.phase_order = ["all"; "ramp"; "hold"; "return"; ...
    "origin-hold"; "unlabeled"];
analysis.layer_names = runs(1).layer_names;
analysis.style = style;
analysis.experiments = runs;
analysis.experiment_summaries = experimentSummaries;

writetable(summaryTable, fullfile(outputDir, 'summary.csv'));
save(fullfile(outputDir, 'analysis.mat'), ...
    'analysis', 'summaryTable', '-v7');

jsonSummary = struct();
jsonSummary.generated_utc = generatedUtc;
jsonSummary.generator = analysis.generator;
jsonSummary.generator_contract = analysis.generator_contract;
jsonSummary.input_files_are_unmodified = true;
jsonSummary.phase_order = analysis.phase_order;
jsonSummary.layer_names = analysis.layer_names;
jsonSummary.bundle_files = bundleNames;
jsonSummary.style = style;
jsonSummary.experiments = experimentSummaries;
jsonSummary.summary_rows = table2struct(summaryTable);
writeJson(fullfile(outputDir, 'analysis_summary.json'), jsonSummary);

visible = char(string(parser.Results.Visible));
pdfPath = fullfile(outputDir, 'research_report.pdf');
if parser.Results.CreatePDF && isfile(pdfPath)
    delete(pdfPath);
end

if parser.Results.CreatePDF
    cover = makeReportCover(runs, summaryTable, style, visible);
    exportgraphics(cover, pdfPath, 'ContentType', 'vector');
    close(cover);
end

figures = { ...
    plotTcpErrors(runs, style, visible), ...
    plotErrorLayers(runs, style, visible), ...
    plotJointTracking(runs, style, visible), ...
    plotJointDetails(runs, style, visible), ...
    plotIkEvents(runs, style, visible), ...
    plotPhaseComparison(runs, summaryTable, style, visible)};
figureFiles = { ...
    'tcp_error_timeseries.png', 'error_layers.png', ...
    'joint_tracking.png', 'j1_j4_j7_detail.png', ...
    'ik_events.png', 'phase_comparison.png'};
for index = 1:numel(figures)
    exportgraphics(figures{index}, fullfile(outputDir, figureFiles{index}), ...
        'Resolution', 180);
    if parser.Results.CreatePDF
        exportgraphics(figures{index}, pdfPath, ...
            'ContentType', 'vector', 'Append', true);
    end
    close(figures{index});
end

fprintf('Pose-follow analysis bundle: %s\n', analysis.output_directory);
for index = 1:numel(bundleNames)
    fprintf('  %s\n', bundleNames(index));
end
end


function files = normalizeFileList(value)
if ischar(value)
    files = {value};
elseif isstring(value)
    files = cellstr(value(:));
elseif iscell(value)
    files = cellfun(@(item) char(string(item)), value(:), ...
        'UniformOutput', false);
else
    error('posefollow:InputType', ...
        'inputFiles must be a character vector, string array, or cell array.');
end
if isempty(files)
    error('posefollow:NoInputs', 'At least one input JSON is required.');
end
end


function style = analysisStyle(experimentCount)
style = struct();
style.experiment_colors_rgb = lines(max(experimentCount, 1));
style.layer_names = [ ...
    "live marker to measured"; ...
    "marker update staleness"; ...
    "accepted marker to IK target"; ...
    "IK target to command"; ...
    "command to measured"];
style.layer_columns = [1, 3, 4, 5, 6];
style.layer_colors_rgb = [ ...
    0.0000, 0.4470, 0.7410; ...
    0.8500, 0.3250, 0.0980; ...
    0.9290, 0.6940, 0.1250; ...
    0.4940, 0.1840, 0.5560; ...
    0.4660, 0.6740, 0.1880];
style.phase_names = ["ramp"; "hold"; "return"; ...
    "origin-hold"; "unlabeled"];
style.phase_colors_rgb = [ ...
    0.65, 0.82, 0.98; ...
    0.70, 0.90, 0.70; ...
    0.98, 0.80, 0.58; ...
    0.82, 0.74, 0.94; ...
    0.86, 0.86, 0.86];
style.joint_state_names = ["IK target"; "command"; "measured"];
style.joint_state_colors_rgb = [ ...
    0.0000, 0.4470, 0.7410; ...
    0.8500, 0.3250, 0.0980; ...
    0.4660, 0.6740, 0.1880];
style.joint_state_line_styles = ["--"; "-"; ":"];
style.jump_color_rgb = [0.6350, 0.0780, 0.1840];
end


function [summaryTable, experimentSummaries] = buildSummary(runs)
phaseOrder = ["all"; "ramp"; "hold"; "return"; ...
    "origin-hold"; "unlabeled"];
rowTemplate = summaryRowTemplate();
rows = repmat(rowTemplate, numel(runs) * numel(phaseOrder), 1);
experimentTemplate = struct( ...
    'experiment', "", 'source_file', "", 'schema_version', nan, ...
    'schema_variant', "", 'group', "", 'profile', "", ...
    'samples', 0, 'duration_sec', nan, 'available_phases', strings(0, 1), ...
    'ik', struct(), 'ik_target_jumps', struct(), ...
    'layer_statistics', struct([]));
experimentSummaries = repmat(experimentTemplate, numel(runs), 1);

rowIndex = 0;
for runIndex = 1:numel(runs)
    run = runs(runIndex);
    for phaseIndex = 1:numel(phaseOrder)
        rowIndex = rowIndex + 1;
        rows(rowIndex) = summarizePhase(run, phaseOrder(phaseIndex));
    end
    experimentSummaries(runIndex) = summarizeExperiment(run);
end
summaryTable = struct2table(rows);
end


function row = summaryRowTemplate()
row = struct( ...
    'experiment', "", 'source_file', "", 'schema_version', nan, ...
    'schema_variant', "", 'group', "", 'profile', "", ...
    'phase', "", 'samples', 0, 'phase_duration_sec', nan, ...
    'tcp_position_mean_mm', nan, 'tcp_position_rms_mm', nan, ...
    'tcp_position_max_mm', nan, 'tcp_position_p95_mm', nan, ...
    'tcp_orientation_mean_deg', nan, 'tcp_orientation_rms_deg', nan, ...
    'tcp_orientation_max_deg', nan, 'tcp_orientation_p95_deg', nan, ...
    'ik_submitted', nan, 'ik_accepted', nan, 'ik_failed', nan, ...
    'ik_superseded', nan, 'ik_acceptance_rate', nan, ...
    'ik_failure_rate', nan, 'ik_superseded_rate', nan, ...
    'ik_latency_mean_ms', nan, 'ik_latency_rms_ms', nan, ...
    'ik_latency_max_ms', nan, 'ik_latency_p95_ms', nan, ...
    'ik_target_jump_events', nan);
end


function row = summarizePhase(run, phaseName)
row = summaryRowTemplate();
row.experiment = run.experiment;
row.source_file = run.source_file;
row.schema_version = run.schema_version;
row.schema_variant = run.schema_variant;
row.group = run.group;
row.profile = run.profile;
row.phase = phaseName;
if phaseName == "all"
    mask = true(size(run.time_sec));
else
    mask = run.phase == phaseName;
end
row.samples = sum(mask);
times = run.time_sec(mask & isfinite(run.time_sec));
if numel(times) > 1
    row.phase_duration_sec = max(times) - min(times);
elseif numel(times) == 1
    row.phase_duration_sec = 0;
end

positionMm = run.position_error_m(mask, 1) * 1000;
orientationDeg = rad2deg(run.orientation_error_rad(mask, 1));
positionStats = distributionStats(positionMm);
orientationStats = distributionStats(orientationDeg);
row.tcp_position_mean_mm = positionStats.mean;
row.tcp_position_rms_mm = positionStats.rms;
row.tcp_position_max_mm = positionStats.max;
row.tcp_position_p95_mm = positionStats.p95;
row.tcp_orientation_mean_deg = orientationStats.mean;
row.tcp_orientation_rms_deg = orientationStats.rms;
row.tcp_orientation_max_deg = orientationStats.max;
row.tcp_orientation_p95_deg = orientationStats.p95;

[eventMask, hasTiming] = ikEventMask(run, phaseName);
if phaseName == "all" || hasTiming
    if phaseName == "all"
        row.ik_submitted = run.ik.submitted;
        row.ik_accepted = run.ik.accepted;
        row.ik_failed = run.ik.failed;
        row.ik_superseded = run.ik.superseded;
    else
        outcomes = string({run.ik.events(eventMask).outcome});
        row.ik_submitted = sum(eventMask);
        row.ik_accepted = sum(outcomes == "accepted");
        row.ik_failed = sum(contains(outcomes, "failed"));
        row.ik_superseded = sum(contains(outcomes, "superseded"));
    end
    if isfinite(row.ik_submitted) && row.ik_submitted > 0
        row.ik_acceptance_rate = row.ik_accepted / row.ik_submitted;
        row.ik_failure_rate = row.ik_failed / row.ik_submitted;
        row.ik_superseded_rate = row.ik_superseded / row.ik_submitted;
    end
end

latencySec = eventLatency(run, eventMask);
latencyStats = distributionStats(latencySec * 1000);
row.ik_latency_mean_ms = latencyStats.mean;
row.ik_latency_rms_ms = latencyStats.rms;
row.ik_latency_max_ms = latencyStats.max;
row.ik_latency_p95_ms = latencyStats.p95;

if run.ik_target_jumps.available
    if phaseName == "all"
        row.ik_target_jump_events = numel(run.ik_target_jumps.times_sec);
    else
        jumpPhases = phaseAtTimes(run, run.ik_target_jumps.times_sec);
        row.ik_target_jump_events = sum(jumpPhases == phaseName);
    end
end
end


function summary = summarizeExperiment(run)
summary = struct();
summary.experiment = run.experiment;
summary.source_file = run.source_file;
summary.schema_version = run.schema_version;
summary.schema_variant = run.schema_variant;
summary.group = run.group;
summary.profile = run.profile;
summary.samples = numel(run.time_sec);
validTime = run.time_sec(isfinite(run.time_sec));
if isempty(validTime)
    summary.duration_sec = nan;
else
    summary.duration_sec = max(validTime) - min(validTime);
end
summary.available_phases = unique(run.phase, 'stable');
summary.ik = struct( ...
    'submitted', run.ik.submitted, 'accepted', run.ik.accepted, ...
    'failed', run.ik.failed, 'superseded', run.ik.superseded, ...
    'event_timing_available', run.ik.event_timing_available);
summary.ik_target_jumps = struct( ...
    'available', run.ik_target_jumps.available, ...
    'threshold_rad', run.ik_target_jumps.threshold_rad, ...
    'transitions', run.ik_target_jumps.transitions, ...
    'events', numel(run.ik_target_jumps.times_sec));

layerRows = struct('name', {}, 'distance', {}, 'signed_projection', {});
for layerIndex = 1:numel(run.layer_names)
    distance = distributionStats(run.position_error_m(:, layerIndex) * 1000);
    projection = distributionStats( ...
        run.position_error_signed_projection_m(:, layerIndex) * 1000);
    layerRows(layerIndex).name = run.layer_names(layerIndex);
    layerRows(layerIndex).distance = distance;
    layerRows(layerIndex).signed_projection = projection;
end
summary.layer_statistics = layerRows;
end


function stats = distributionStats(values)
values = double(values(:));
values = values(isfinite(values));
stats = struct('mean', nan, 'rms', nan, 'max', nan, 'p95', nan);
if isempty(values)
    return;
end
stats.mean = mean(values);
stats.rms = sqrt(mean(values .^ 2));
stats.max = max(values);
stats.p95 = percentileLinear(values, 0.95);
end


function value = percentileLinear(values, fraction)
values = sort(double(values(:)));
if numel(values) == 1
    value = values(1);
    return;
end
position = 1 + (numel(values) - 1) * fraction;
lowerIndex = floor(position);
upperIndex = ceil(position);
weight = position - lowerIndex;
value = values(lowerIndex) * (1 - weight) + values(upperIndex) * weight;
end


function [mask, available] = ikEventMask(run, phaseName)
available = run.ik.event_timing_available;
mask = false(numel(run.ik.events), 1);
if ~available
    return;
end
if phaseName == "all"
    mask(:) = true;
    return;
end
times = nan(numel(run.ik.events), 1);
for index = 1:numel(run.ik.events)
    times(index) = run.ik.events(index).requested_sec;
end
mask = phaseAtTimes(run, times) == phaseName;
end


function latency = eventLatency(run, mask)
latency = nan(sum(mask), 1);
selected = find(mask);
for outputIndex = 1:numel(selected)
    event = run.ik.events(selected(outputIndex));
    latency(outputIndex) = event.request_to_accepted_sec;
    if ~isfinite(latency(outputIndex))
        latency(outputIndex) = event.request_to_complete_sec;
    end
end
end


function phases = phaseAtTimes(run, times)
times = double(times(:));
phases = repmat("unassigned", size(times));
validSamples = isfinite(run.time_sec);
sampleTimes = run.time_sec(validSamples);
samplePhases = run.phase(validSamples);
if isempty(sampleTimes)
    return;
end
for index = 1:numel(times)
    if ~isfinite(times(index))
        continue;
    end
    [~, nearest] = min(abs(sampleTimes - times(index)));
    phases(index) = samplePhases(nearest);
end
end


function writeJson(path, value)
encoded = jsonencode(value, 'PrettyPrint', true);
file = fopen(path, 'w', 'n', 'UTF-8');
if file < 0
    error('posefollow:WriteFailed', 'Could not open %s for writing.', path);
end
cleanup = onCleanup(@() fclose(file));
fwrite(file, encoded, 'char');
fwrite(file, newline, 'char');
clear cleanup;
end
