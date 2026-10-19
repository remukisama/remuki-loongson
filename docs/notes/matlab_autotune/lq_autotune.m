function result = lq_autotune(varargin)
%LQ_AUTOTUNE MATLAB-side autotune bridge for Loongson SmartCar.
%
% Examples:
%   lq_autotune('mode','smoke')
%   lq_autotune('mode','build','stage','T11')
%   lq_autotune('mode','board','stage','T11','boardIp','192.168.5.100','iterations',3)
%
% Modes:
%   smoke  Parse bundled sample logs and score one candidate. No file edits.
%   build  Patch car_config.hpp with one candidate and build locally in WSL.
%   board  Patch, build, deploy, run, fetch board log, parse, and score.

opts = parse_options(varargin{:});
script_dir = fileparts(mfilename('fullpath'));
repo_win = clean_path(fullfile(script_dir, '..', '..'));
if isempty(opts.repoWin)
    opts.repoWin = repo_win;
end
if isempty(opts.repoWsl)
    opts.repoWsl = '/home/zoumaguanbei/SmartCar_Project/Loongson_2k300_301_Library-master/Loongson_2K300_301_LIB';
end

if strcmpi(opts.mode, 'smoke')
    sample_log = fullfile(script_dir, 'samples', lower([opts.stage '_sample.log']));
    text = read_text(sample_log);
    metrics = parse_stage_metrics(text, opts.stage);
    result = make_result(opts.stage, opts.mode, default_candidate(opts.stage), metrics);
    fprintf('[AUTOTUNE_SMOKE] stage=%s score=%.4f status=%s\n', result.stage, result.score, result.status);
    return;
end

if strcmpi(opts.mode, 'build') || strcmpi(opts.mode, 'board')
    if strcmpi(opts.mode, 'board') && isempty(opts.boardIp)
        error('board mode requires boardIp, for example: lq_autotune(''mode'',''board'',''boardIp'',''192.168.5.100'')');
    end
    best = [];
    candidates = make_candidates(opts.stage, opts.iterations);
    for i = 1:length(candidates)
        candidate = candidates(i);
        fprintf('[AUTOTUNE] iteration=%d/%d stage=%s\n', i, length(candidates), opts.stage);
        patch_config(opts.repoWin, candidate);
        write_stage_file(opts.repoWin, opts.stage);
        if strcmpi(opts.mode, 'board')
            install_board_stage_file(opts.boardIp, opts.stage);
        end
        run_build(opts.repoWsl, opts.boardIp, strcmpi(opts.mode, 'board'));
        if strcmpi(opts.mode, 'board')
            log_text = fetch_board_log(opts.boardIp, opts.stage, 45);
            metrics = parse_stage_metrics(log_text, opts.stage);
        else
            metrics = struct('samples', 0, 'scoreHint', 0, 'buildOnly', 1);
        end
        current = make_result(opts.stage, opts.mode, candidate, metrics);
        print_candidate_result(current);
        if isempty(best) || current.score < best.score
            best = current;
        end
    end
    result = best;
    fprintf('[AUTOTUNE_DONE] stage=%s best_score=%.4f status=%s\n', result.stage, result.score, result.status);
    return;
end

error('Unknown mode: %s', opts.mode);

end

function opts = parse_options(varargin)
opts = struct();
opts.mode = 'smoke';
opts.stage = 'T11';
opts.boardIp = '';
opts.iterations = 1;
opts.repoWin = '';
opts.repoWsl = '';
if mod(length(varargin), 2) ~= 0
    error('Options must be name/value pairs.');
end
for i = 1:2:length(varargin)
    key = lower(varargin{i});
    value = varargin{i + 1};
    switch key
        case 'mode'
            opts.mode = value;
        case 'stage'
            opts.stage = upper(value);
        case 'boardip'
            opts.boardIp = value;
        case 'iterations'
            opts.iterations = value;
        case 'repowin'
            opts.repoWin = value;
        case 'repowsl'
            opts.repoWsl = value;
        otherwise
            error('Unknown option: %s', varargin{i});
    end
end
end

function candidates = make_candidates(stage, iterations)
base = default_candidate(stage);
candidates = repmat(base, 1, max(1, iterations));
for i = 1:length(candidates)
    scale = 1.0 + 0.08 * (i - 1);
    for k = 1:length(candidates(i).names)
        if candidates(i).tunable(k)
            candidates(i).values(k) = candidates(i).values(k) * scale;
        end
    end
end
end

function candidate = default_candidate(stage)
candidate = struct();
candidate.stage = upper(stage);
if strcmpi(stage, 'T11')
    candidate.names = {'kGyroPidP', 'kGyroPidD', 'kGyroFeedforwardGain', 'kDiffOutputMaxDelta'};
    candidate.values = [0.40, 0.0045, 0.025, 3.6];
    candidate.tunable = [1, 1, 1, 1];
elseif strcmpi(stage, 'T06')
    candidate.names = {'kSteerKp', 'kSteerKd', 'kSteerFilterAlpha', 'kVisualGyroLimitDps'};
    candidate.values = [3.9, 3.0, 0.55, 180.0];
    candidate.tunable = [1, 1, 1, 0];
else
    error('Unsupported autotune stage: %s. Use T11 or T06.', stage);
end
end

function patch_config(repo_win, candidate)
config_path = fullfile(repo_win, 'user_app', 'include', 'car_config.hpp');
text = read_text(config_path);
backup_path = [config_path '.autotune_bak'];
if exist(backup_path, 'file') ~= 2
    write_text(backup_path, text);
end
for i = 1:length(candidate.names)
    name = candidate.names{i};
    value = candidate.values(i);
    pattern = ['(constexpr\s+[^=;]*\s' name '\s*=\s*)[-+]?\d+(\.\d+)?f?(;)'];
    replacement = ['$1' format_cpp_float(value) '$3'];
    [text, count] = regexprep(text, pattern, replacement, 'once');
    if count ~= 1
        error('Failed to patch %s in %s', name, config_path);
    end
end
write_text(config_path, text);
end

function write_stage_file(repo_win, stage)
write_text(fullfile(repo_win, 'lq_autotune_stage.txt'), [upper(stage) sprintf('\n')]);
end

function install_board_stage_file(board_ip, stage)
tmp_stage = 'C:\tmp\lq_autotune_stage.txt';
write_text(tmp_stage, [upper(stage) sprintf('\n')]);
cmd = sprintf('wsl bash -lc "echo %s | ssh -o ConnectTimeout=10 root@%s ''cat > ~/lq_autotune_stage.txt && cp ~/lq_autotune_stage.txt /tmp/lq_autotune_stage.txt''"', upper(stage), board_ip);
fprintf('[AUTOTUNE_STAGE_CMD] %s\n', cmd);
status = system(cmd);
if status ~= 0
    error('Failed to install board stage file, status %d.', status);
end
end

function run_build(repo_wsl, board_ip, run_on_board)
if nargin < 3
    run_on_board = false;
end
if isempty(board_ip)
    cmd = sprintf('wsl bash -lc "cd %s && ./main/build.sh"', shell_quote(repo_wsl));
elseif run_on_board
    cmd = sprintf('wsl bash -lc "cd %s && ./main/build.sh %s -r"', shell_quote(repo_wsl), board_ip);
else
    cmd = sprintf('wsl bash -lc "cd %s && ./main/build.sh %s"', shell_quote(repo_wsl), board_ip);
end
fprintf('[AUTOTUNE_BUILD_CMD] %s\n', cmd);
status = system(cmd);
if status ~= 0
    error('Build/deploy command failed with status %d.', status);
end
end

function log_text = fetch_board_log(board_ip, stage, timeout_sec)
if nargin < 3
    timeout_sec = 45;
end
summary_tag = ['[' upper(stage) '_SUMMARY]'];
deadline = now + timeout_sec / 86400;
log_text = '';
while now < deadline
    cmd = sprintf('wsl bash -lc "ssh -o ConnectTimeout=10 root@%s ''cat main.log 2>/dev/null || cat ~/main.log 2>/dev/null || true''"', board_ip);
    [status, current_text] = system(cmd);
    if status == 0
        log_text = current_text;
        if ~isempty(strfind(log_text, summary_tag)) || ~isempty(strfind(log_text, '[AUTOTUNE] stage_ret='))
            return;
        end
    end
    pause(2);
end
if isempty(log_text)
    error('Failed to fetch board log before timeout.');
end
end

function metrics = parse_stage_metrics(text, stage)
if strcmpi(stage, 'T11')
    metrics = parse_key_values(text, '\[T11_SUMMARY\]([^\n\r]*)');
    metrics.step = parse_key_values(text, '\[T11_STEP_SUMMARY\]([^\n\r]*)');
elseif strcmpi(stage, 'T06')
    metrics = parse_key_values(text, '\[T06_SUMMARY\]([^\n\r]*)');
else
    error('No parser for stage %s', stage);
end
end

function s = parse_key_values(text, line_pattern)
tokens = regexp(text, line_pattern, 'tokens');
s = struct();
if isempty(tokens)
    s.missing = 1;
    return;
end
line = tokens{end}{1};
pairs = regexp(line, '([A-Za-z0-9_]+)=([-+]?\d+(\.\d+)?)', 'tokens');
for i = 1:length(pairs)
    key = pairs{i}{1};
    val = str2double(pairs{i}{2});
    s.(key) = val;
end
end

function result = make_result(stage, mode, candidate, metrics)
score = score_metrics(stage, metrics);
result = struct();
result.stage = upper(stage);
result.mode = mode;
result.candidate = candidate;
result.metrics = metrics;
result.score = score;
if isfield(metrics, 'missing')
    result.status = 'missing_summary';
elseif score >= 1e5
    result.status = 'unsafe_or_failed';
else
    result.status = 'ok';
end
end

function score = score_metrics(stage, m)
if isfield(m, 'missing')
    score = 1e6;
    return;
end
if strcmpi(stage, 'T11')
    score = field_or(m, 'weak_track_steps', 0) * 80 + ...
            field_or(m, 'cap_steps', 0) * 60 + ...
            field_or(m, 'speed_weak_steps', 0) * 70 + ...
            field_or(m, 'reverse_steps', 0) * 500 + ...
            field_or(m, 'cap_rate', 0) * 60 + ...
            field_or(m, 'reverse_rate', 0) * 500 + ...
            field_or(m, 'fail_rate', 0) * 1000;
elseif strcmpi(stage, 'T06')
    score = field_or(m, 'GErr_abs', 0) * 2.5 + ...
            field_or(m, 'cap_rate', 0) * 80 + ...
            field_or(m, 'pd_sat_rate', 0) * 80 + ...
            field_or(m, 'img_bad', 0) * 1000 + ...
            field_or(m, 'img_warn', 0) * 80 + ...
            field_or(m, 'imu_bad', 0) * 1000 + ...
            field_or(m, 'SE_abs', 0);
else
    score = 1e6;
end
end

function v = field_or(s, name, default_value)
if isfield(s, name)
    v = s.(name);
else
    v = default_value;
end
end

function print_candidate_result(r)
fprintf('[AUTOTUNE_RESULT] stage=%s score=%.4f status=%s\n', r.stage, r.score, r.status);
for i = 1:length(r.candidate.names)
    fprintf('  %s=%.6g\n', r.candidate.names{i}, r.candidate.values(i));
end
end

function text = read_text(path)
fid = fopen(path, 'r');
if fid < 0
    error('Cannot open file for read: %s', path);
end
c = fread(fid, inf, '*char')';
fclose(fid);
text = c;
end

function write_text(path, text)
fid = fopen(path, 'w');
if fid < 0
    error('Cannot open file for write: %s', path);
end
fwrite(fid, text, 'char');
fclose(fid);
end

function out = format_cpp_float(value)
if abs(value - round(value)) < 1e-9
    out = sprintf('%.1ff', value);
else
    out = sprintf('%.6gf', value);
end
end

function q = shell_quote(path)
q = strrep(path, '''', '''"''"''');
q = ['''' q ''''];
end

function p = clean_path(p)
p = strrep(p, filesep, filesep);
end
