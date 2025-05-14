function [optimized_pid_params, info_all] = tuneJointPID(robot_model, joint_linear_models, initial_pid_params, options)
%TUNEJOINTPID  Tune joint‑level PID gains using Control System Toolbox only.
%
% This implementation relies exclusively on PIDTUNE.  It therefore works on
% any MATLAB installation that has the Control System Toolbox but *not* the
% Robust Control Toolbox.  It retains the same calling syntax as the
% original H‑infty variant, but silently ignores options that are specific
% to H‑infty design (Ms, Mt, Mu).  The optional "wb" or "PM" fields are now
% used to steer PIDTUNE toward a target bandwidth or phase‑margin.
%
% Inputs:
%   robot_model          –  (unused, kept for interface compatibility)
%   joint_linear_models  –  1×N cell array of LTI plants for each joint
%   initial_pid_params   –  1×N cell array with fields .Kp .Ki .Kd .joint_type
%   options (struct)     –  .joints_to_tune   (default: 1:N)
%                          .wb                desired bandwidth (rad/s)
%                          .PM                desired phase‑margin (deg)
%                          .Ctype             'pid'|'pi'|'p'  (default: 'pid')
%
% Outputs:
%   optimized_pid_params –  same shape as initial_pid_params with tuned gains
%   info_all             –  cell array of the INFO structs returned by PIDTUNE
%
% Example:
%   opt.wb = 12; opt.PM = 60;
%   tuned = tuneJointPID(model, plants, init, opt);
%
% 2025‑05‑14  –  OpenAI assistance

%% -------------------- default option handling --------------------------
if nargin < 4
    options = struct();
end
num_joints = numel(initial_pid_params);

defaults = struct( ...
    'joints_to_tune',  1:num_joints, ...
    'wb',              [], ...            % leave empty to let PIDTUNE choose
    'PM',              60, ...            % target phase margin (deg)
    'Ctype',           'pid' ...          % controller structure requested
);
fields = fieldnames(defaults);
for k = 1:numel(fields)
    f = fields{k};
    if ~isfield(options,f)
        options.(f) = defaults.(f);
    end
end

%% -------------------- pre‑allocate outputs ----------------------------
optimized_pid_params = initial_pid_params;
info_all             = cell(1,num_joints);

%% -------------------- set up PIDTUNE options --------------------------
optTune = pidtuneOptions('PhaseMargin',options.PM);

%% -------------------- main tuning loop --------------------------------
for j = options.joints_to_tune
    P = joint_linear_models{j};
    if isempty(P)
        warning('Joint %d: no plant model – skipping.',j);
        continue;
    end

    fprintf('\n>> Joint %d  (%s)\n',j, initial_pid_params{j}.joint_type);
    Ctypes = {options.Ctype,'pi','p'};   % graceful fall‑back list
    tuned   = [];
    info    = struct();
    for ct = Ctypes
        try
            if isempty(options.wb)
                [tuned,info] = pidtune(P,ct{1},optTune);
            else
                [tuned,info] = pidtune(P,ct{1},options.wb,optTune);
            end
            fprintf('   using structure %s – achieved PM %.1f° at %.2f rad/s\n',ct{1},info.PhaseMargin,info.CrossoverFrequency);
            break;   % success
        catch ME
            fprintf('   -> %s failed (%s)\n',ct{1},ME.message);
        end
    end

    % if everything failed, keep original gains
    if isempty(tuned)
        warning('Joint %d: PIDTUNE failed for all structures, keeping original gains.',j);
        continue;
    end

    % extract gains and save
    [Kp,Ki,Kd,~] = piddata(tuned);
    optimized_pid_params{j}.Kp = Kp;
    optimized_pid_params{j}.Ki = Ki;
    optimized_pid_params{j}.Kd = Kd;
    info_all{j} = info;   %#ok<AGROW>

    fprintf('   →  Kp = %.4g   Ki = %.4g   Kd = %.4g\n',Kp,Ki,Kd);
end

fprintf('\nPID tuning complete. Tuned %d joints.\n',numel(options.joints_to_tune));

%% -------------------- optional closed‑loop analysis -------------------
if nargout == 0 || (isfield(options,'plot') && options.plot)
    analyzeClosedLoopPerformance(joint_linear_models,optimized_pid_params,options);
end
end

% ======================================================================
function analyzeClosedLoopPerformance(joint_models,pid_params,options)
% simple post‑tuning diagnostic plots and metrics (unchanged from original)

fprintf('\nClosed‑Loop Performance Summary\n');
fprintf('Joint |  BW(rad/s)  |  PM(deg)  |  GM(dB)  | Rise(s) | Settling(s) | Overshoot(%%)\n');

for j = options.joints_to_tune
    P = joint_models{j};
    if isempty(P); continue; end

    C = pid(pid_params{j}.Kp,pid_params{j}.Ki,pid_params{j}.Kd);
    L = P*C;
    CL= feedback(L,1);

    [Gm,Pm,~,Wcp] = margin(L);
    gm_db = 20*log10(Gm);

    S = stepinfo(CL);
    bw = Wcp;   % approximate BW ~ gain crossover

    fprintf('%5d | %10.2f | %9.2f | %8.2f | %7.3f | %11.3f | %10.2f\n', ...
        j,bw,Pm,gm_db,S.RiseTime,S.SettlingTime,S.Overshoot);
end
fprintf('\n');
end
