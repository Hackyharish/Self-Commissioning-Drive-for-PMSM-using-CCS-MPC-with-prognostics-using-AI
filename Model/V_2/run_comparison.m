% run_comparison.m
% Compare CCS-MPC and FOC models for PMSM Drive

params_pmsm_inverter;

% Simulation duration
sim_time = '0.5';
sim_time = '0.5';
load_system('PMSM_CCSMPC_ClosedLoop');
set_param('PMSM_CCSMPC_ClosedLoop/SpeedRef', 'Time', '0.05');
save_system('PMSM_CCSMPC_ClosedLoop');

load_system('PMSM_FOC_ClosedLoop');
set_param('PMSM_FOC_ClosedLoop/SpeedRef', 'Time', '0.05');
save_system('PMSM_FOC_ClosedLoop');

disp('Simulating CCS-MPC Model...');
out_mpc = sim('PMSM_CCSMPC_ClosedLoop', 'StopTime', sim_time);
% Run FOC Model
disp('Simulating FOC Model...');
out_foc = sim('PMSM_FOC_ClosedLoop', 'StopTime', sim_time);

try
    time_mpc = out_mpc.omega_e.Time;
    spd_mpc = out_mpc.omega_e.Data;
    iq_mpc = out_mpc.iq_meas.Data;
catch
    time_mpc = out_mpc.omega_e_log.Time;
    spd_mpc = out_mpc.omega_e_log.Data;
    iq_mpc = out_mpc.iq_meas_log.Data;
end

% Extract FOC signals
try
    time_foc = out_foc.omega_e.Time;
    spd_foc = out_foc.omega_e.Data;
    iq_foc = out_foc.iq_meas.Data;
catch
    time_foc = out_foc.omega_e_log.Time;
    spd_foc = out_foc.omega_e_log.Data;
    iq_foc = out_foc.iq_meas_log.Data;
end

% Calculate Metrics
% Rise Time (10% to 90% of 200 rad/s)
target_spd = 200;
lower_th = 0.1 * target_spd;
upper_th = 0.9 * target_spd;

% MPC Rise Time (ignoring the 0.05s delay)
idx_low = find(spd_mpc >= lower_th, 1);
idx_high = find(spd_mpc >= upper_th, 1);
if isempty(idx_low) || isempty(idx_high)
    rt_mpc = NaN;
else
    rt_mpc = time_mpc(idx_high) - time_mpc(idx_low);
end

% FOC Rise Time
idx_low_f = find(spd_foc >= lower_th, 1);
idx_high_f = find(spd_foc >= upper_th, 1);
if isempty(idx_low_f) || isempty(idx_high_f)
    rt_foc = NaN;
else
    rt_foc = time_foc(idx_high_f) - time_foc(idx_low_f);
end

% Ripple (steady state standard deviation from t = 2.5 to 3.0)
ss_mpc_idx = time_mpc > 2.5;
ss_mpc_idx = time_mpc > 0.4;
ss_foc_idx = time_foc > 0.4;
ripple_spd_mpc = std(spd_mpc(ss_mpc_idx));
ripple_iq_mpc = std(iq_mpc(ss_mpc_idx));

ripple_spd_foc = std(spd_foc(ss_foc_idx));
ripple_iq_foc = std(iq_foc(ss_foc_idx));

% Display Results
fprintf('\n=== Performance Comparison ===\n');
fprintf('Metric           | CCS-MPC      | FOC\n');
fprintf('-----------------|--------------|--------------\n');
fprintf('Max Speed (rad/s)| %10.4f | %10.4f\n', max(spd_mpc), max(spd_foc));
fprintf('Rise Time (s)    | %10.4f | %10.4f\n', rt_mpc, rt_foc);
fprintf('Speed Ripple (rad/s)| %9.4f | %9.4f\n', ripple_spd_mpc, ripple_spd_foc);
fprintf('Iq Ripple (A)    | %10.4f | %10.4f\n', ripple_iq_mpc, ripple_iq_foc);

% Plotting
figure('Name', 'CCS-MPC vs FOC Comparison', 'Position', [100 100 800 600]);

subplot(2,1,1);
plot(time_mpc, spd_mpc, 'b', 'LineWidth', 1.5); hold on;
plot(time_foc, spd_foc, 'r--', 'LineWidth', 1.5);
yline(target_spd, 'k:', 'Reference');
title('Speed Tracking Response');
xlabel('Time (s)');
ylabel('Speed (rad/s)');
legend('CCS-MPC', 'FOC', 'Reference', 'Location', 'Best');
grid on;

subplot(2,1,2);
plot(time_mpc, iq_mpc, 'b', 'LineWidth', 1.2); hold on;
plot(time_foc, iq_foc, 'r--', 'LineWidth', 1.2);
title('Q-axis Current (Iq)');
xlabel('Time (s)');
ylabel('Current (A)');
legend('CCS-MPC', 'FOC', 'Location', 'Best');
grid on;

saveas(gcf, 'comparison_results_3s.png');
disp('Plot saved as comparison_results_3s.png');




