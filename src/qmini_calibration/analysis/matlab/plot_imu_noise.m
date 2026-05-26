function plot_imu_noise(csv_path, dr_angvel, dr_projgrav)
% PLOT_IMU_NOISE  Visualize the IMU noise/mount calibration window.
%
%   plot_imu_noise('data/<date>_imu_noise/samples.csv')
%   plot_imu_noise(csv_path, 0.35, 0.1)   % overlay Isaac Lab DR bands
%
% samples.csv is written by analysis/analyze_imu_bag.py (columns:
%   t_s, gx,gy,gz, ax,ay,az, pgx,pgy,pgz). Base MATLAB only (no toolboxes).
%
% The authoritative green/red vs Isaac Lab is diff_against_isaaclab.py. The
% optional DR args here just draw reference lines for a visual sanity check;
% pass the numbers the diff prints (ang_vel DR is in obs units = raw*0.2).

    if nargin < 2, dr_angvel = []; end
    if nargin < 3, dr_projgrav = []; end

    T = readmatrix(csv_path);
    t  = T(:,1);
    gyro = T(:,2:4);   accel = T(:,5:7);   pg = T(:,8:10);
    fs = (numel(t)-1) / (t(end)-t(1));
    ax = {'x','y','z'};

    fprintf('IMU window: %d samples, %.1f Hz, %.1f s\n', numel(t), fs, t(end)-t(1));
    fprintf('  gyro  bias  (rad/s): %s\n', mat2str(mean(gyro),3));
    fprintf('  gyro  noise std     : %s\n', mat2str(std(gyro),3));
    fprintf('  accel mean  (m/s^2) : %s   |g|=%.3f\n', mat2str(mean(accel),4), norm(mean(accel)));
    fprintf('  proj-gravity mean   : %s\n', mat2str(mean(pg),4));
    fprintf('  proj-gravity std    : %s\n', mat2str(std(pg),3));

    % --- Fig 1: time series ---
    figure('Name','IMU time series');
    subplot(3,1,1); plot(t,gyro);  grid on; ylabel('gyro [rad/s]');
        legend(ax); title('Gyro / accel / projected-gravity over static window');
    subplot(3,1,2); plot(t,accel); grid on; ylabel('accel [m/s^2]'); legend(ax);
    subplot(3,1,3); plot(t,pg);    grid on; ylabel('proj-gravity'); xlabel('t [s]'); legend(ax);

    % --- Fig 2: per-axis gyro histograms (noise shape) ---
    figure('Name','Gyro noise histograms');
    for i=1:3
        subplot(1,3,i); histogram(gyro(:,i)-mean(gyro(:,i)),40); grid on;
        title(sprintf('gyro %s  (\\sigma=%.2e)',ax{i},std(gyro(:,i)))); xlabel('rad/s');
    end

    % --- Fig 3: Allan deviation of gyro (classic IMU noise plot) ---
    figure('Name','Gyro Allan deviation');
    loglog_done = false;
    for i=1:3
        [tau,ad] = local_allan(gyro(:,i), fs);
        loglog(tau,ad,'-o','DisplayName',['gyro ' ax{i}]); hold on; loglog_done = true;
    end
    if loglog_done, grid on; xlabel('\tau [s]'); ylabel('Allan deviation [rad/s]');
        legend show; title('Gyro Allan deviation'); end

    % --- Fig 4: measured noise vs DR band ---
    figure('Name','Noise vs Isaac Lab DR');
    subplot(1,2,1);
    bar(std(gyro)*0.2); grid on; set(gca,'xticklabel',ax);
    ylabel('ang\_vel noise (obs units = std*0.2)'); title('vs DR \pm0.35');
    if ~isempty(dr_angvel), yline(dr_angvel,'r--','DR'); end
    subplot(1,2,2);
    bar(std(pg)); grid on; set(gca,'xticklabel',ax);
    ylabel('proj-gravity noise std'); title('vs DR \pm0.1');
    if ~isempty(dr_projgrav), yline(dr_projgrav,'r--','DR'); end
end

function [tau, adev] = local_allan(x, fs)
% Overlapping-bin Allan deviation, no toolbox needed.
    x = x(:); N = numel(x);
    ms = unique(round(logspace(0, log10(floor(N/2)), 40)));
    tau = ms / fs;  adev = zeros(size(ms));
    for k = 1:numel(ms)
        m = ms(k); nb = floor(N/m);
        y = mean(reshape(x(1:nb*m), m, nb), 1);   % bin averages
        adev(k) = sqrt(0.5 * mean(diff(y).^2));
    end
end
