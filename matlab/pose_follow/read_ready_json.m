function run = read_ready_json(filePath, experimentName, varargin)
%READ_READY_JSON Normalize a pose-ready snapshot without modifying it.
parser = inputParser;
addRequired(parser, 'filePath');
addRequired(parser, 'experimentName');
addParameter(parser, 'TargetRad', []);
addParameter(parser, 'PostureName', "");
addParameter(parser, 'GravityEnabled', nan);
parse(parser, filePath, experimentName, varargin{:});

filePath = char(string(filePath));
raw = jsondecode(fileread(filePath));
if ~isfield(raw, 'kind') || string(raw.kind) ~= "pose_snapshot"
    error('ready:InvalidKind', 'Expected pose_snapshot in %s.', filePath);
end
if ~isfield(raw, 'groups') || isempty(raw.groups)
    error('ready:MissingGroup', 'Missing group snapshot in %s.', filePath);
end
group = raw.groups(1);
jointNames = string(group.joint_names(:));
jointCount = numel(jointNames);
feedback = rowVector(group.joint_positions_rad, jointCount, 'feedback');
ready = struct();
if isfield(raw, 'ready_posture') && isstruct(raw.ready_posture)
    ready = raw.ready_posture;
end
result = struct();
if isfield(raw, 'ready_result') && isstruct(raw.ready_result)
    result = raw.ready_result;
end

target = optionalVector(result, 'target_rad', jointCount);
if any(~isfinite(target))
    target = optionalVector(ready, 'target_rad', jointCount);
end
fallbackTarget = parser.Results.TargetRad;
if any(~isfinite(target)) && isnumeric(fallbackTarget) && numel(fallbackTarget) == jointCount
    target = double(fallbackTarget(:)).';
end
reference = optionalVector(result, 'reference_rad', jointCount);
if isfield(result, 'feedback_rad')
    feedback = rowVector(result.feedback_rad, jointCount, 'feedback_rad');
end
errorRad = feedback - target;
if isfield(result, 'joint_error_rad')
    errorRad = rowVector(result.joint_error_rad, jointCount, 'joint_error_rad');
end

postureName = string(parser.Results.PostureName);
if isfield(result, 'posture_name')
    postureName = string(result.posture_name);
elseif isfield(ready, 'name')
    postureName = string(ready.name);
end
gravityEnabled = parser.Results.GravityEnabled;
if isfield(result, 'gravity_enabled')
    gravityEnabled = logical(result.gravity_enabled);
end
gravityScale = scalarOr(result, 'gravity_scale', nan);
gravityTorque = optionalVector(result, 'gravity_torque_nm', jointCount);
termination = "legacy_observed";
if isfield(result, 'termination')
    termination = string(result.termination);
end
passed = all(abs(errorRad) <= scalarOr(ready, 'tolerance_rad', 0.02));
if isfield(result, 'passed')
    passed = logical(result.passed);
end

run = struct();
run.experiment = string(experimentName);
run.source_file = string(java.io.File(filePath).getCanonicalPath());
run.posture_name = postureName;
run.joint_names = jointNames;
run.target_rad = target;
run.reference_rad = reference;
run.feedback_rad = feedback;
run.error_rad = errorRad;
run.max_abs_error_rad = max(abs(errorRad), [], 'omitnan');
run.rms_error_rad = sqrt(mean(errorRad .^ 2, 'omitnan'));
run.passed = passed;
run.termination = termination;
run.gravity_enabled = gravityEnabled;
run.gravity_scale = gravityScale;
run.gravity_torque_nm = gravityTorque;
run.settling_sec = scalarOr(result, 'settling_sec', nan);
end

function value = scalarOr(parent, name, fallback)
value = fallback;
if isstruct(parent) && isfield(parent, name) && isnumeric(parent.(name)) && isscalar(parent.(name))
    value = double(parent.(name));
end
end

function vector = optionalVector(parent, name, width)
vector = nan(1, width);
if isstruct(parent) && isfield(parent, name) && isnumeric(parent.(name))
    candidate = double(parent.(name)(:)).';
    if numel(candidate) == width
        vector = candidate;
    end
end
end

function vector = rowVector(value, width, label)
vector = double(value(:)).';
if numel(vector) ~= width
    error('ready:VectorWidth', '%s needs %d values.', label, width);
end
end
