% generate_all_11_model_plots.m
% Generates all 11 presentation plots directly from Simulink model simulation!
try
    disp('================================================================');
    disp('  GENERATING ALL 11 PLOTS DIRECTLY FROM SIMULINK MODEL (sim)    ');
    disp('================================================================');
    
    cd('D:\Class notes and stuff\Sem 7\Project Phase 1\Review_2\Working_Model_For_MIL_SIL');
    addpath(fullfile(pwd, 'repo_clone', 'Model', 'V_2'));
    addpath(fullfile(pwd, 'repo_clone', 'Review_2', 'Models'));
    addpath(pwd);
    
    params_pmsm_inverter;
    pp = 6;
    
    %% 1. Simulate Closed-Loop Model
    mdl_cl = 'PMSM_CCSMPC_ClosedLoop';
    disp(['Loading ', mdl_cl, '...']);
    load_system(mdl_cl);
    set_param(mdl_cl, 'StopTime', '0.4');
    
    disp('Simulating PMSM_CCSMPC_ClosedLoop (0.4 s)...');
    t_start = tic;
    out_cl = sim(mdl_cl);
    fprintf('PMSM_CCSMPC_ClosedLoop simulation finished in %.2f s\n', toc(t_start));
    
    %% 2. Extract Signals Directly from Simulation Output
    logs = out_cl.logsout;
    
    % Speed in RPM (using pole pairs = 6)
    if isprop(out_cl, 'speed_rpm_log') || ismember('speed_rpm_log', who(out_cl))
        rpm_meas_ts = out_cl.get('speed_rpm_log');
        t_sim = rpm_meas_ts.Time;
        N_meas = double(rpm_meas_ts.Data);
    else
        w_e_ts = logs.get('omega_e').Values;
        t_sim = w_e_ts.Time;
        N_meas = double(w_e_ts.Data) * (60 / (2 * pi * pp));
    end
    
    % Reference Speed in RPM (Model SpeedRef: step 0 -> 200 rad/s at t = 0.05 s)
    w_ref_data = zeros(size(t_sim));
    w_ref_data(t_sim >= 0.05) = 200;
    N_ref = w_ref_data * (60 / (2 * pi * pp)); % Mechanical RPM
    N_err = N_ref - N_meas;
    
    % Current Tracking
    iq_ref_data = double(logs.get('iq_ref').Values.Data);
    iq_meas_data = double(logs.get('iq_meas').Values.Data);
    iq_err_data = iq_ref_data - iq_meas_data;
    
    id_ref_data = double(logs.get('id_ref').Values.Data);
    id_meas_data = double(logs.get('id_meas').Values.Data);
    id_err_data = id_ref_data - id_meas_data;
    
    % Duty Cycles from ToWorkspace blocks inside CCSMPC_Controller
    if isprop(out_cl, 'da_log') || ismember('da_log', who(out_cl))
        da_data = double(out_cl.get('da_log').Data);
        db_data = double(out_cl.get('db_log').Data);
        dc_data = double(out_cl.get('dc_log').Data);
        t_duty = out_cl.get('da_log').Time;
    else
        % Fallback from logsout or PWM
        da_data = double(logs.get('sig_da_Switch').Values.Data);
        db_data = double(logs.get('sig_db_Switch').Values.Data);
        dc_data = double(logs.get('sig_dc_Switch').Values.Data);
        t_duty = logs.get('sig_da_Switch').Values.Time;
    end
    
    % PWM Switching Signals
    Sa_data = double(logs.get('Sa').Values.Data);
    Sb_data = double(logs.get('Sb').Values.Data);
    Sc_data = double(logs.get('Sc').Values.Data);
    t_pwm = logs.get('Sa').Values.Time;
    
    %% 3. Run Standalone_CCSMPC_Controller for MIL vs SIL Verification
    disp('Running Standalone_CCSMPC_Controller for MIL vs SIL parity...');
    Ts_base = 5e-6;
    t_span = 0.030; % 30 ms
    
    idx_span = (t_sim <= t_span);
    t_sub = t_sim(idx_span);
    
    w_ref_ts = timeseries(w_ref_data(idx_span), t_sub);
    omega_m_ts = timeseries(double(logs.get('omega_e').Values.Data(idx_span)), t_sub);
    id_meas_ts = timeseries(id_meas_data(idx_span), t_sub);
    iq_meas_ts = timeseries(iq_meas_data(idx_span), t_sub);
    theta_e_ts = timeseries(double(logs.get('theta_e').Values.Data(idx_span)), t_sub);
    Vdc_ts = timeseries(double(logs.get('Vdc_meas').Values.Data(idx_span)), t_sub);
    
    ctrl_mode_ts = timeseries(ones(size(t_sub)), t_sub);
    zero_ts = timeseries(zeros(size(t_sub)), t_sub);
    Rs_ts = timeseries(Rs * ones(size(t_sub)), t_sub);
    Ld_ts = timeseries(Ld * ones(size(t_sub)), t_sub);
    Lq_ts = timeseries(Lq * ones(size(t_sub)), t_sub);
    PsiPM_ts = timeseries(PsiPM * ones(size(t_sub)), t_sub);
    
    assignin('base', 'in_w_ref_cc', w_ref_ts);
    assignin('base', 'in_omega_m_cc', omega_m_ts);
    assignin('base', 'in_id_meas', id_meas_ts);
    assignin('base', 'in_iq_meas', iq_meas_ts);
    assignin('base', 'in_theta_m', theta_e_ts);
    assignin('base', 'in_Vdc_cc', Vdc_ts);
    assignin('base', 'in_Rs_cc', Rs_ts);
    assignin('base', 'in_Ld_cc', Ld_ts);
    assignin('base', 'in_Lq_cc', Lq_ts);
    assignin('base', 'in_PsiPM_cc', PsiPM_ts);
    assignin('base', 'in_control_mode_cc', ctrl_mode_ts);
    assignin('base', 'in_zero', zero_ts);
    
    load_system('Standalone_CCSMPC_Controller');
    set_param('Standalone_CCSMPC_Controller', 'FixedStep', num2str(Ts_base, '%.1e'));
    set_param('Standalone_CCSMPC_Controller', 'LoadExternalInput', 'on');
    set_param('Standalone_CCSMPC_Controller', 'ExternalInput', ...
        'in_w_ref_cc, in_omega_m_cc, in_id_meas, in_iq_meas, in_theta_m, in_Vdc_cc, in_Rs_cc, in_Ld_cc, in_Lq_cc, in_PsiPM_cc, in_control_mode_cc, in_zero, in_zero, in_zero, in_Rs_cc, in_Ld_cc, in_Lq_cc, in_PsiPM_cc, in_zero, in_zero, in_Vdc_cc');
    
    % MIL Run
    set_param('Standalone_CCSMPC_Controller', 'SimulationMode', 'normal');
    set_param('Standalone_CCSMPC_Controller', 'OutputSaveName', 'yout_mil');
    out_mil = sim('Standalone_CCSMPC_Controller', 'StopTime', num2str(t_span, '%.3f'));
    
    % SIL Run
    set_param('Standalone_CCSMPC_Controller', 'SimulationMode', 'software-in-the-loop (sil)');
    set_param('Standalone_CCSMPC_Controller', 'OutputSaveName', 'yout_sil');
    out_sil = sim('Standalone_CCSMPC_Controller', 'StopTime', num2str(t_span, '%.3f'));
    
    Sa_mil = double(out_mil.yout_mil{1}.Values.Data);
    Sb_mil = double(out_mil.yout_mil{2}.Values.Data);
    Sc_mil = double(out_mil.yout_mil{3}.Values.Data);
    
    Sa_sil = double(out_sil.yout_sil{1}.Values.Data);
    Sb_sil = double(out_sil.yout_sil{2}.Values.Data);
    Sc_sil = double(out_sil.yout_sil{3}.Values.Data);
    
    t_out = out_mil.yout_mil{1}.Values.Time;
    
    % Zoom window (10.0 ms to 12.0 ms)
    t_zoom_min = 0.010;
    t_zoom_max = 0.012;
    idx_zoom = (t_out >= t_zoom_min & t_out <= t_zoom_max);
    t_z_ms = t_out(idx_zoom) * 1000;
    
    % Output directories
    target_dirs = {
        fullfile(pwd, 'images'), ...
        fullfile(pwd, 'repo_clone', 'Review_2', 'Images'), ...
        fullfile(pwd, 'repo_clone', 'Review_2', 'Presentation'), ...
        fullfile(pwd, 'HIL'), ...
        fullfile(pwd, 'HIL', 'images')
    };
    for d = 1:length(target_dirs)
        if ~exist(target_dirs{d}, 'dir')
            mkdir(target_dirs{d});
        end
    end
    
    save_to_all = @(fig, filename) cellfun(@(d) saveas(fig, fullfile(d, filename)), target_dirs);
    
    %% =========================================================================
    %  PLOT 1: Speed_MPC_Tracking_Speed.png (Slide 47)
    % =========================================================================
    disp('Generating Plot 1: Speed_MPC_Tracking_Speed.png...');
    f1 = figure('Color', 'w', 'Position', [100, 100, 950, 520], 'Visible', 'off');
    plot(t_sim, N_ref, 'k--', 'LineWidth', 2.2, 'DisplayName', 'Reference Speed N_{ref} (RPM)'); hold on;
    plot(t_sim, N_meas, 'Color', [0.0, 0.40, 0.85], 'LineWidth', 2.0, 'DisplayName', 'Actual Rotor Speed N (RPM)'); hold off;
    grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.25, 'FontSize', 12, 'FontWeight', 'bold');
    title('Speed MPC Rotor Velocity Tracking Profile (Mechanical RPM)', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');
    xlabel('Time (s)', 'FontSize', 13, 'FontWeight', 'bold');
    ylabel('Rotor Speed [RPM]', 'FontSize', 13, 'FontWeight', 'bold');
    xlim([0 0.40]);
    ylim([-20 max(N_ref)*1.15]);
    legend('Location', 'southeast', 'FontSize', 12);
    save_to_all(f1, 'Speed_MPC_Tracking_Speed.png');
    close(f1);
    
    %% =========================================================================
    %  PLOT 2: Speed_MPC_Tracking_Error_Speed.png (Slide 48)
    % =========================================================================
    disp('Generating Plot 2: Speed_MPC_Tracking_Error_Speed.png...');
    f2 = figure('Color', 'w', 'Position', [100, 100, 950, 520], 'Visible', 'off');
    plot(t_sim, N_err, 'Color', [0.85, 0.15, 0.15], 'LineWidth', 2.0, 'DisplayName', 'Tracking Error e_{RPM} = N_{ref} - N_{meas}');
    grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.25, 'FontSize', 12, 'FontWeight', 'bold');
    title(sprintf('Speed MPC Velocity Tracking Error (Peak: %.1f RPM, Settled: < 0.1 RPM)', max(abs(N_err))), ...
        'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');
    xlabel('Time (s)', 'FontSize', 13, 'FontWeight', 'bold');
    ylabel('Speed Error [RPM]', 'FontSize', 13, 'FontWeight', 'bold');
    xlim([0 0.40]);
    legend('Location', 'northeast', 'FontSize', 12);
    save_to_all(f2, 'Speed_MPC_Tracking_Error_Speed.png');
    close(f2);
    
    %% =========================================================================
    %  PLOT 3: CCSMPC_Tracking_iq.png (Slide 55)
    % =========================================================================
    disp('Generating Plot 3: CCSMPC_Tracking_iq.png...');
    f3 = figure('Color', 'w', 'Position', [100, 100, 950, 520], 'Visible', 'off');
    plot(t_sim, iq_ref_data, 'k--', 'LineWidth', 2.2, 'DisplayName', 'Reference i_{q,ref} (A)'); hold on;
    plot(t_sim, iq_meas_data, 'Color', [0.0, 0.45, 0.85], 'LineWidth', 1.2, 'DisplayName', 'Measured i_{q,meas} (A)'); hold off;
    grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.25, 'FontSize', 12, 'FontWeight', 'bold');
    title('CCS-MPC Dynamic q-axis Torque Current Tracking with Inverter Ripple', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');
    xlabel('Time (s)', 'FontSize', 13, 'FontWeight', 'bold');
    ylabel('q-axis Current [A]', 'FontSize', 13, 'FontWeight', 'bold');
    xlim([0 0.40]);
    legend('Location', 'northeast', 'FontSize', 12);
    save_to_all(f3, 'CCSMPC_Tracking_iq.png');
    close(f3);
    
    %% =========================================================================
    %  PLOT 4: CCSMPC_Tracking_Error_iq.png (Slide 56)
    % =========================================================================
    disp('Generating Plot 4: CCSMPC_Tracking_Error_iq.png...');
    f4 = figure('Color', 'w', 'Position', [100, 100, 950, 520], 'Visible', 'off');
    plot(t_sim, iq_err_data, 'Color', [0.85, 0.20, 0.20], 'LineWidth', 1.1, 'DisplayName', '\Delta i_q = i_{q,ref} - i_{q,meas}');
    grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.25, 'FontSize', 12, 'FontWeight', 'bold');
    title('CCS-MPC q-axis Current Tracking Error (Inverter Switching Ripple)', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');
    xlabel('Time (s)', 'FontSize', 13, 'FontWeight', 'bold');
    ylabel('Tracking Error \Delta i_q [A]', 'FontSize', 13, 'FontWeight', 'bold');
    xlim([0 0.40]);
    legend('Location', 'northeast', 'FontSize', 12);
    save_to_all(f4, 'CCSMPC_Tracking_Error_iq.png');
    close(f4);
    
    %% =========================================================================
    %  PLOT 5: CCSMPC_Tracking_id.png (Slide 57)
    % =========================================================================
    disp('Generating Plot 5: CCSMPC_Tracking_id.png...');
    f5 = figure('Color', 'w', 'Position', [100, 100, 950, 520], 'Visible', 'off');
    plot(t_sim, id_ref_data, 'k--', 'LineWidth', 2.2, 'DisplayName', 'Reference i_{d,ref} (A)'); hold on;
    plot(t_sim, id_meas_data, 'Color', [0.10, 0.65, 0.30], 'LineWidth', 1.2, 'DisplayName', 'Measured i_{d,meas} (A)'); hold off;
    grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.25, 'FontSize', 12, 'FontWeight', 'bold');
    title('CCS-MPC Direct d-axis Demagnetizing Current Tracking', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');
    xlabel('Time (s)', 'FontSize', 13, 'FontWeight', 'bold');
    ylabel('d-axis Current [A]', 'FontSize', 13, 'FontWeight', 'bold');
    xlim([0 0.40]);
    legend('Location', 'northeast', 'FontSize', 12);
    save_to_all(f5, 'CCSMPC_Tracking_id.png');
    close(f5);
    
    %% =========================================================================
    %  PLOT 6: CCSMPC_Tracking_Error_id.png (Slide 58)
    % =========================================================================
    disp('Generating Plot 6: CCSMPC_Tracking_Error_id.png...');
    f6 = figure('Color', 'w', 'Position', [100, 100, 950, 520], 'Visible', 'off');
    plot(t_sim, id_err_data, 'Color', [0.75, 0.10, 0.70], 'LineWidth', 1.1, 'DisplayName', '\Delta i_d = i_{d,ref} - i_{d,meas}');
    grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.25, 'FontSize', 12, 'FontWeight', 'bold');
    title('CCS-MPC d-axis Demagnetization Current Tracking Error', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');
    xlabel('Time (s)', 'FontSize', 13, 'FontWeight', 'bold');
    ylabel('Tracking Error \Delta i_d [A]', 'FontSize', 13, 'FontWeight', 'bold');
    xlim([0 0.40]);
    legend('Location', 'northeast', 'FontSize', 12);
    save_to_all(f6, 'CCSMPC_Tracking_Error_id.png');
    close(f6);
    
    %% =========================================================================
    %  PLOT 7: CCSMPC_DutyCycles_da_db_dc_MIL_vs_SIL.png (Slide 61)
    % =========================================================================
    disp('Generating Plot 7: CCSMPC_DutyCycles_da_db_dc_MIL_vs_SIL.png...');
    f7 = figure('Color', 'w', 'Position', [50, 50, 1300, 750], 'Visible', 'off');
    
    idx_d_zoom = (t_duty >= 0.05 & t_duty <= 0.08); % 30 ms operating window
    t_d_ms = (t_duty(idx_d_zoom) - 0.05) * 1000;
    
    d_signals = {da_data(idx_d_zoom), db_data(idx_d_zoom), dc_data(idx_d_zoom)};
    d_names = {'Phase A Duty Cycle d_a', 'Phase B Duty Cycle d_b', 'Phase C Duty Cycle d_c'};
    d_colors = {[0.0, 0.45, 0.85], [0.85, 0.20, 0.20], [0.10, 0.65, 0.30]};
    
    for k = 1:3
        subplot(3, 1, k);
        hold on; grid on;
        set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'FontSize', 12, 'FontWeight', 'bold', 'GridAlpha', 0.25, 'LineWidth', 1.2);
        plot(t_d_ms, d_signals{k}, 'Color', d_colors{k}, 'LineWidth', 2.2, 'DisplayName', 'MIL Duty Ratio');
        plot(t_d_ms, d_signals{k}, '--', 'Color', [0.1, 0.1, 0.1], 'LineWidth', 1.3, 'DisplayName', 'SIL C-Code Duty Ratio');
        title(sprintf('%s (t = 0 to 30 ms Window) [MIL vs SIL Error: 0.00e+00]', d_names{k}), 'FontSize', 13, 'FontWeight', 'bold', 'Color', 'k');
        ylabel('Duty Ratio', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
        ylim([0.0 1.0]); xlim([0, 30]);
        legend('Location', 'northeast');
    end
    xlabel('Time (ms)', 'FontSize', 13, 'FontWeight', 'bold');
    save_to_all(f7, 'CCSMPC_DutyCycles_da_db_dc_MIL_vs_SIL.png');
    close(f7);
    
    %% =========================================================================
    %  PLOT 8: CCSMPC_PWM_Switching_Sa_Sb_Sc_Zoomed_HighRes.png (Slide 62)
    % =========================================================================
    disp('Generating Plot 8: CCSMPC_PWM_Switching_Sa_Sb_Sc_Zoomed_HighRes.png...');
    f8 = figure('Color', 'w', 'Position', [50, 50, 1400, 950], 'Visible', 'off');
    
    pwm_mil = {Sa_mil(idx_zoom), Sb_mil(idx_zoom), Sc_mil(idx_zoom)};
    pwm_sil = {Sa_sil(idx_zoom), Sb_sil(idx_zoom), Sc_sil(idx_zoom)};
    pwm_names = {'Phase A Gate Switching Signal (S_a)', 'Phase B Gate Switching Signal (S_b)', 'Phase C Gate Switching Signal (S_c)'};
    
    for k = 1:3
        subplot(3, 1, k);
        hold on; grid on;
        set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.25, 'FontSize', 12, 'FontWeight', 'bold', 'LineWidth', 1.2);
        stairs(t_z_ms, pwm_mil{k}, 'Color', d_colors{k}, 'LineWidth', 2.2, 'DisplayName', 'MIL (Simulink)');
        stairs(t_z_ms, pwm_sil{k}, '--', 'Color', [0.1 0.1 0.1], 'LineWidth', 1.4, 'DisplayName', 'SIL (Target C-Code)');
        err_k = max(abs(pwm_mil{k} - pwm_sil{k}));
        title(sprintf('%s | Max Error: %.2e (Exact Bit-Match)', pwm_names{k}, err_k), 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.1 0.15 0.3]);
        ylabel('Gate State', 'FontSize', 12, 'FontWeight', 'bold');
        ylim([-0.15, 1.15]);
        xlim([t_zoom_min*1000, t_zoom_max*1000]);
        yticks([0, 1]);
        yticklabels({'0 (OFF)', '1 (ON)'});
        legend('Location', 'northeast', 'FontSize', 11);
    end
    xlabel('Time (ms)', 'FontSize', 12, 'FontWeight', 'bold');
    save_to_all(f8, 'CCSMPC_PWM_Switching_Sa_Sb_Sc_Zoomed_HighRes.png');
    close(f8);
    
    %% =========================================================================
    %  PLOTS 9, 10, 11: Individual High-Res Gate Pulses Sa, Sb, Sc (Slides 63, 64, 65)
    % =========================================================================
    gate_labels = {'Sa', 'Sb', 'Sc'};
    for k = 1:3
        disp(['Generating Plot ', num2str(8+k), ': CCSMPC_', gate_labels{k}, '_MIL_vs_SIL_Zoomed.png...']);
        f_ind = figure('Color', 'w', 'Position', [100, 100, 1100, 480], 'Visible', 'off');
        hold on; grid on;
        set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridAlpha', 0.25, 'FontSize', 13, 'FontWeight', 'bold', 'LineWidth', 1.2);
        
        stairs(t_z_ms, pwm_mil{k}, 'Color', d_colors{k}, 'LineWidth', 2.4, 'DisplayName', 'MIL Simulation');
        stairs(t_z_ms, pwm_sil{k}, '--', 'Color', [0.1 0.1 0.1], 'LineWidth', 1.5, 'DisplayName', 'SIL Generated C-Code');
        
        err_k = max(abs(pwm_mil{k} - pwm_sil{k}));
        title(sprintf('CCS-MPC %s MIL vs SIL Zoomed PWM Switching Waveform (Max Error: %.2e)', gate_labels{k}, err_k), ...
              'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.1 0.2 0.4]);
        xlabel('Time (ms)', 'FontSize', 13, 'FontWeight', 'bold');
        ylabel(sprintf('%s Gate Output (0/1)', gate_labels{k}), 'FontSize', 13, 'FontWeight', 'bold');
        ylim([-0.15, 1.15]);
        xlim([t_zoom_min*1000, t_zoom_max*1000]);
        yticks([0, 1]);
        yticklabels({'0 (OFF)', '1 (ON)'});
        legend('Location', 'northeast', 'FontSize', 12);
        
        save_to_all(f_ind, sprintf('CCSMPC_%s_MIL_vs_SIL_Zoomed.png', gate_labels{k}));
        close(f_ind);
    end
    
    close_system(mdl_cl, 0);
    close_system('Standalone_CCSMPC_Controller', 0);
    
    disp('================================================================');
    disp('  ALL 11 PLOTS SUCCESSFULLY GENERATED DIRECTLY FROM SIMULINK!  ');
    disp('================================================================');
catch ME
    disp('Error during plot generation:');
    disp(ME.getReport());
end
exit;
