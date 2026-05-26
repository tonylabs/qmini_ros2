function plot_bus_jitter(csv_path, policy_rate_hz)
% PLOT_BUS_JITTER  Visualize the motor-bus /joint_states timing jitter.
%
%   plot_bus_jitter('data/<date>_bus_jitter/periods.csv')
%   plot_bus_jitter(csv_path, 50)   % overlay the policy-rate period line
%
% periods.csv is written by analysis/analyze_bus_jitter_bag.py (columns:
%   t_s, period_ms). Base MATLAB only (no toolboxes).
%
% The authoritative green/red vs Isaac Lab (bus rate >= policy rate) is
% diff_against_isaaclab.py; policy_rate_hz here just draws a reference line.

    if nargin < 2, policy_rate_hz = []; end

    T = readmatrix(csv_path);
    t = T(:,1);  p = T(:,2);          % seconds, milliseconds
    med = median(p);
    dropped = p > 2*med;

    fprintf('Bus timing: %d periods, mean %.3f ms (%.1f Hz), std %.3f ms\n', ...
        numel(p), mean(p), 1000/mean(p), std(p));
    fprintf('  min/max %.3f/%.3f ms, p95 %.3f, p99 %.3f, dropped (>2x median) %d\n', ...
        min(p), max(p), prctile_local(p,95), prctile_local(p,99), sum(dropped));

    % --- Fig 1: period over time, dropped ticks flagged ---
    figure('Name','Bus period over time');
    plot(t, p, '-'); hold on; grid on;
    if any(dropped), plot(t(dropped), p(dropped), 'rx', 'MarkerSize',10, ...
        'DisplayName','dropped (>2x median)'); end
    if ~isempty(policy_rate_hz)
        yline(1000/policy_rate_hz, 'g--', sprintf('policy %.0f Hz', policy_rate_hz));
    end
    xlabel('t [s]'); ylabel('publish period [ms]');
    title('Motor-bus /joint\_states period (jitter = spread; spikes = dropped ticks)');

    % --- Fig 2: period histogram ---
    figure('Name','Bus period histogram');
    histogram(p, 60); grid on; hold on;
    xline(mean(p), 'k-', 'mean'); xline(med, 'b--', 'median');
    if ~isempty(policy_rate_hz), xline(1000/policy_rate_hz,'g--','policy period'); end
    xlabel('period [ms]'); ylabel('count');
    title(sprintf('Period distribution (\\sigma=%.3f ms)', std(p)));
end

function v = prctile_local(x, q)
% Percentile without the Statistics Toolbox.
    x = sort(x(:)); n = numel(x);
    if n == 1, v = x; return; end
    idx = min(n, max(1, round(q/100 * (n-1)) + 1));
    v = x(idx);
end
