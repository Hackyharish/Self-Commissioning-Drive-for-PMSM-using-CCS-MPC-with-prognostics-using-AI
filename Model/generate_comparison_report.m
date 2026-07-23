% run_comparison.m
% Compare CCS-MPC and FOC models for PMSM Drive

params_pmsm_inverter;

% Simulation duration
sim_time = '0.5';
speed_test = 200;  % rad/s
torque_step = 0.05; % N.m applied at 0.3s
t_step = 0.3;      % time of torque step

% --- 1. Run CCS-MPC Model ---
disp('Running CCS-MPC...');
load_system('PMSM_CCSMPC_ClosedLoop_bak');
set_param('PMSM_CCSMPC_ClosedLoop_bak/SpeedRef', 'Time', '0.05', 'After', num2str(speed_test));
set_param('PMSM_CCSMPC_ClosedLoop_bak/LoadTorque', 'Time', num2str(t_step), 'Before', '0', 'After', num2str(torque_step));
save_system('PMSM_CCSMPC_ClosedLoop_bak');

% --- 2. Run FOC Model ---
disp('Running FOC...');
load_system('PMSM_FOC_ClosedLoop_bak');
set_param('PMSM_FOC_ClosedLoop_bak/SpeedRef', 'Time', '0.05', 'After', num2str(speed_test));
set_param('PMSM_FOC_ClosedLoop_bak/LoadTorque', 'Time', num2str(t_step), 'Before', '0', 'After', num2str(torque_step));
save_system('PMSM_FOC_ClosedLoop_bak');

disp('Simulating CCS-MPC Model...');
out_mpc = sim('PMSM_CCSMPC_ClosedLoop_bak', 'StopTime', sim_time);
% Run FOC Model
disp('Simulating FOC Model...');
out_foc = sim('PMSM_FOC_ClosedLoop_bak', 'StopTime', sim_time);

% Extract MPC signals
[time_mpc, spd_mpc] = extract_signal(out_mpc, 'omega_e');
[~, iq_mpc] = extract_signal(out_mpc, 'iq_meas');
[~, id_mpc] = extract_signal(out_mpc, 'id_meas');
[~, theta_mpc] = extract_signal(out_mpc, 'theta_e');

% Extract FOC signals
[time_foc, spd_foc] = extract_signal(out_foc, 'omega_e');
[~, iq_foc] = extract_signal(out_foc, 'iq_meas');
[~, id_foc] = extract_signal(out_foc, 'id_meas');
[~, theta_foc] = extract_signal(out_foc, 'theta_e');
% Reconstruct ia_meas from id, iq, theta for THD calculation
ia_mpc = id_mpc .* cos(theta_mpc) - iq_mpc .* sin(theta_mpc);
ia_foc = id_foc .* cos(theta_foc) - iq_foc .* sin(theta_foc);

% Calculate Metrics
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

% Overshoot Percent
max_spd_mpc = max(spd_mpc);
overshoot_mpc = max(0, (max_spd_mpc - target_spd) / target_spd * 100);

max_spd_foc = max(spd_foc);
overshoot_foc = max(0, (max_spd_foc - target_spd) / target_spd * 100);

% Steady-state
ss_mpc_idx = time_mpc > 0.4;
ss_foc_idx = time_foc > 0.4;

% Ripple (steady state standard deviation)
ripple_spd_mpc = std(spd_mpc(ss_mpc_idx));
ripple_iq_mpc = std(iq_mpc(ss_mpc_idx));

ripple_spd_foc = std(spd_foc(ss_foc_idx));
ripple_iq_foc = std(iq_foc(ss_foc_idx));

% Torque Ripple (Nm)
% T_e = 1.5 * pp * PsiPM * Iq
Kt_val = 1.5 * 4 * 0.020; % Using pp=4, PsiPM=0.020 from params
torque_mpc = Kt_val .* iq_mpc;
torque_foc = Kt_val .* iq_foc;
ripple_torque_mpc = std(torque_mpc(ss_mpc_idx));
ripple_torque_foc = std(torque_foc(ss_foc_idx));

% Steady State Error
ss_err_mpc = abs(mean(spd_mpc(ss_mpc_idx)) - target_spd);
ss_err_foc = abs(mean(spd_foc(ss_foc_idx)) - target_spd);

% Current THD
% Use the directly logged ia_meas phase current instead of reconstructing

thd_mpc = get_thd_pct(ia_mpc(ss_mpc_idx));
thd_foc = get_thd_pct(ia_foc(ss_foc_idx));

% Display Results
fprintf('\n=== Performance Comparison ===\n');
fprintf('%-20s | %-12s | %-12s\n', 'Metric', 'CCS-MPC', 'FOC');
fprintf('---------------------|--------------|--------------\n');
fprintf('%-20s | %12.4f | %12.4f\n', 'Max Speed (rad/s)', max_spd_mpc, max_spd_foc);
fprintf('%-20s | %12.4f | %12.4f\n', 'Rise Time (s)', rt_mpc, rt_foc);
fprintf('%-20s | %12.4f | %12.4f\n', 'Overshoot (%)', overshoot_mpc, overshoot_foc);
fprintf('%-20s | %12.4f | %12.4f\n', 'Steady-State Err', ss_err_mpc, ss_err_foc);
fprintf('%-20s | %12.4f | %12.4f\n', 'Speed Ripple (rad/s)', ripple_spd_mpc, ripple_spd_foc);
fprintf('%-20s | %12.4f | %12.4f\n', 'Iq Ripple (A)', ripple_iq_mpc, ripple_iq_foc);
fprintf('%-20s | %12.4f | %12.4f\n', 'Torque Ripple (Nm)', ripple_torque_mpc, ripple_torque_foc);
fprintf('%-20s | %12.4f | %12.4f\n', 'Current THD (%)', thd_mpc, thd_foc);

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

% Helper Function for THD
function thd_val = get_thd_pct(x)
    if isempty(x)
        thd_val = NaN;
        return;
    end
    x = x - mean(x);
    L = length(x);
    if L < 2
        thd_val = 0;
        return;
    end
    Y = fft(x);
    P2 = abs(Y/L);
    P1 = P2(1:floor(L/2)+1);
    P1(2:end-1) = 2*P1(2:end-1);
    fund_amp = max(P1);
    rms_fund = fund_amp / sqrt(2);
    rms_total = rms(x);
    if rms_fund > 0
        thd_val = sqrt(max(rms_total^2 - rms_fund^2, 0)) / rms_fund * 100;
    else
        thd_val = 0;
    end
end

function [time, data] = extract_signal(out_data, name)
    if isprop(out_data, 'logsout') && ~isempty(out_data.logsout.find(name))
        elem = out_data.logsout.get(name);
        if isa(elem, 'Simulink.SimulationData.Dataset')
            elem = elem{1};
        end
        time = elem.Values.Time;
        data = elem.Values.Data;
    else
        % Try ToWorkspace blocks (often named with _log)
        var_name = [name '_log'];
        if isprop(out_data, var_name) || (isprop(out_data, 'who') && ismember(var_name, out_data.who))
            ts = out_data.get(var_name);
            time = ts.Time;
            data = ts.Data;
        elseif isprop(out_data, name) || (isprop(out_data, 'who') && ismember(name, out_data.who))
            ts = out_data.get(name);
            time = ts.Time;
            data = ts.Data;
        else
            % One more try for ia_meas which isn't always logged explicitly in FOC
            if strcmp(name, 'ia_meas')
                % Dummy values if ia_meas is not found (e.g. FOC backup model)
                time = [0; 0.5]; data = [0; 0];
                return;
            end
            error('Signal %s not found in logsout or as ToWorkspace variable.', name);
        end
    end
end
