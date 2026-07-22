%% run_and_plot.m
% Runs the PMSM CCS-MPC closed-loop simulation and saves the resulting
% plots to a 'plots' directory.
% -------------------------------------------------------------------------

fprintf('=== Running Simulation and Generating Plots ===\n');

% Create plots directory if it doesn't exist
if ~exist('plots', 'dir')
    mkdir('plots');
end

% 1. Setup and load parameters
params_pmsm_inverter;
mdl = 'PMSM_CCSMPC_ClosedLoop_working';
load_system(mdl);

% Set tuned gains (just in case workspace differs)
assignin('base', 'Kp_spd', 0.05);
assignin('base', 'Ki_spd', 10.0);
assignin('base', 'J', 1e-4);
assignin('base', 'Rs_hat', Rs);
assignin('base', 'Ld_hat', Ld);
assignin('base', 'Lq_hat', Lq);
assignin('base', 'PsiPM_hat', PsiPM);

% Configure reference: step to 200 rad/s elec at t=0.01s (requires Field Weakening)
set_param([mdl '/SpeedRef'], 'Time', '0.01', 'Before', '0', 'After', '100');
% Apply load torque step to 0.3 N.m at t=0.25s
set_param([mdl '/LoadTorque'], 'Time', '0.25', 'Before', '0', 'After', '0.3');
set_param([mdl '/CtrlMode'], 'Value', '3'); % Closed loop
set_param(mdl, 'StopTime', '0.4');
set_param(mdl, 'SimulationCommand', 'update');

% 2. Run simulation
fprintf('Simulating for 0.3s...\n');
tic;
out = sim(mdl);
fprintf('Simulation finished in %.1f seconds.\n', toc);

% 3. Extract data
omega_e = squeeze(out.omega_e_log.Data);
t       = out.omega_e_log.Time;
iq      = squeeze(out.iq_meas_log.Data);
iq_ref  = squeeze(out.iq_ref_log.Data);
id      = squeeze(out.id_meas_log.Data);
id_ref  = squeeze(out.id_ref_log.Data);
vd      = squeeze(out.vd_ref_log.Data);
vq      = squeeze(out.vq_ref_log.Data);
Sa      = squeeze(out.Sa_log.Data);
t_sa    = out.Sa_log.Time;
Vdc     = squeeze(out.Vdc_log.Data);
t_vdc   = out.Vdc_log.Time;

% 4. Generate and save plots

% Plot 1: Speed Tracking
fig1 = figure('Name', 'Speed Tracking', 'Position', [100 100 800 400], 'Visible', 'off');
plot(t*1000, omega_e, 'b-', 'LineWidth', 1.5); grid on; hold on;
yline(100, 'r--', 'LineWidth', 1.5);
xline(150, 'k:', 'Load Torque Step'); % T=0.15s = 150ms
title('Electrical Speed Tracking Response'); 
ylabel('\omega_e (rad/s)'); xlabel('Time (ms)');
legend('Actual Speed (\omega_e)', 'Reference Speed (\omega_{ref})', 'Location', 'best');
saveas(fig1, fullfile('plots', 'speed_tracking.png'));
fprintf('Saved plots/speed_tracking.png\n');

% Plot 2: q-axis Current (Torque-producing)
fig2 = figure('Name', 'q-axis Current', 'Position', [150 150 800 400], 'Visible', 'off');
plot(t*1000, iq, 'b-', 'LineWidth', 1); hold on;
plot(t*1000, iq_ref, 'r--', 'LineWidth', 1.5); grid on;
xline(150, 'k:', 'Load Torque Step');
title('q-axis Current (Torque Producing Component)'); 
ylabel('Current (A)'); xlabel('Time (ms)');
legend('Measured i_q', 'Reference i_{q,ref}', 'Location', 'best');
saveas(fig2, fullfile('plots', 'iq_current.png'));
fprintf('Saved plots/iq_current.png\n');

% Plot 3: d-axis Current (Flux-producing)
fig3 = figure('Name', 'd-axis Current', 'Position', [200 200 800 400], 'Visible', 'off');
plot(t*1000, id, 'b-', 'LineWidth', 1); hold on;
plot(t*1000, id_ref, 'r--', 'LineWidth', 1.5); grid on;
title('d-axis Current (Flux Producing Component)'); 
ylabel('Current (A)'); xlabel('Time (ms)');
legend('Measured i_d', 'Reference i_{d,ref}', 'Location', 'best');
saveas(fig3, fullfile('plots', 'id_current.png'));
fprintf('Saved plots/id_current.png\n');

% Plot 4: Optimal Voltage Commands from MPC
fig4 = figure('Name', 'Voltage Commands', 'Position', [250 250 800 400], 'Visible', 'off');
plot(t*1000, vd, 'b-', 'LineWidth', 1); hold on;
plot(t*1000, vq, 'r-', 'LineWidth', 1); grid on;
xline(150, 'k:', 'Load Torque Step');
title('CCS-MPC Optimal Voltage Vector Commands'); 
ylabel('Voltage (V)'); xlabel('Time (ms)');
legend('v_d', 'v_q', 'Location', 'best');
saveas(fig4, fullfile('plots', 'voltage_commands.png'));
fprintf('Saved plots/voltage_commands.png\n');

% Plot 5: Summary Subplot
fig5 = figure('Name', 'Summary Overview', 'Position', [300 100 1000 800], 'Visible', 'off');
subplot(2,2,1);
plot(t*1000, omega_e, 'b-'); hold on; yline(100, 'r--'); grid on; title('Speed'); ylabel('rad/s');
subplot(2,2,2);
plot(t*1000, iq, 'b-'); hold on; plot(t*1000, iq_ref, 'r--'); grid on; title('I_q Current'); ylabel('A');
subplot(2,2,3);
plot(t*1000, vd, 'b-', t*1000, vq, 'r-'); grid on; title('Voltage Commands'); ylabel('V'); legend('v_d', 'v_q');
subplot(2,2,4);
idx = t_sa < 0.005; % first 5 ms
if sum(idx) > 0
    plot(t_sa(idx)*1000, Sa(idx), 'k-'); grid on; title('Switching Signal (Phase A, First 5ms)'); ylabel('Logic'); xlabel('Time (ms)');
end
saveas(fig5, fullfile('plots', 'summary_dashboard.png'));
fprintf('Saved plots/summary_dashboard.png\n');

fprintf('=== All plots generated successfully in "plots" folder ===\n');
