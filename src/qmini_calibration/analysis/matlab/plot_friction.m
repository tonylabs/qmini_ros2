function plot_friction(csv_path, static_range)
% PLOT_FRICTION  Visualize the inclined-plane foot-floor friction trials.
%
%   plot_friction('data/<date>_friction/samples.csv')
%   plot_friction(csv_path, [0.1 2.0])   % overlay the trained static_friction_range
%
% samples.csv is written by analysis/analyze_friction_bag.py (columns:
%   t_s, tilt_deg, accel_mag). slip_markers.csv (column t_s) sits beside it and
% marks the detected breakaway instants. Base MATLAB only (no toolboxes).
%
% The breakaway angle per trial is the PEAK tilt before each slip; mu_static =
% tan(theta). The authoritative green/red vs Isaac Lab is diff_against_isaaclab.py;
% static_range here just draws reference bands as mu = tan(theta) angles.

    if nargin < 2, static_range = []; end

    T = readmatrix(csv_path);
    t = T(:,1);  tilt = T(:,2);  amag = T(:,3);

    mpath = fullfile(fileparts(csv_path), 'slip_markers.csv');
    slips = [];
    if isfile(mpath)
        M = readmatrix(mpath);
        slips = M(:);  slips(isnan(slips)) = [];
    end

    % breakaway angle per trial = peak tilt in the window ending at each slip
    win_start = t(1);
    fprintf('Friction trials:\n');
    for k = 1:numel(slips)
        seg = tilt(t >= win_start & t <= slips(k));
        if ~isempty(seg)
            theta = max(seg);
            fprintf('  trial %d: breakaway %.1f deg -> mu_static = %.3f\n', ...
                k, theta, tand(theta));
        end
        win_start = slips(k);
    end

    % --- Fig 1: tilt over time, slip instants flagged ---
    figure('Name','Inclined-plane tilt over time');
    plot(t, tilt, '-'); hold on; grid on;
    for k = 1:numel(slips)
        xline(slips(k), 'r--', sprintf('slip %d', k));
    end
    if ~isempty(static_range)
        yline(atand(static_range(1)), 'g:', sprintf('\\mu=%.1f', static_range(1)));
        yline(atand(static_range(2)), 'g:', sprintf('\\mu=%.1f', static_range(2)));
    end
    xlabel('t [s]'); ylabel('tilt from vertical [deg]');
    title('Board tilt (peak before each slip = breakaway angle)');

    % --- Fig 2: accel magnitude (slip = spike above gravity) ---
    figure('Name','Specific-force magnitude');
    plot(t, amag, '-'); hold on; grid on;
    yline(9.80665, 'k--', 'gravity');
    for k = 1:numel(slips), xline(slips(k), 'r--'); end
    xlabel('t [s]'); ylabel('|a| [m/s^2]');
    title('Specific-force magnitude (breakaway = spike above gravity)');
end
