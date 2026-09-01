% Master script for MIL & SIL execution, error analysis, plotting, and code generation
try
    disp('================================================================');
    disp('  STARTING COMPREHENSIVE MIL/SIL VERIFICATION & CODE GENERATION ');
    disp('================================================================');
    
    params_pmsm_inverter;
    
    if ~exist('images', 'dir')
        mkdir('images');
    end
    if ~exist('Code_Generation_Reports', 'dir')
        mkdir('Code_Generation_Reports');
    end
    
    %% ====================================================================
    %% 1. AUTOTUNER: 9-SECOND INPUT MIL & SIL
    %% ====================================================================
    disp('----------------------------------------------------------------');
    disp('1. AUTOTUNER: Loading 9s input vectors and executing MIL & SIL');
    disp('----------------------------------------------------------------');
    
    load('baseline_test_vectors.mat');
    
    % Extend inputs to 9.0 seconds
    v_ext = struct();
    for i = 1:7
        ts = test_vectors.(sprintf('AT_in_%d', i));
        t_old = ts.Time;
        d_old = ts.Data;
        dt = t_old(2) - t_old(1);
        t_new = (t_old(end) + dt : dt : 9.0)';
        t_ext = [t_old; t_new];
        d_ext = [d_old; repmat(d_old(end,:), length(t_new), 1)];
        v_ext.(sprintf('AT_in_%d', i)) = timeseries(double(d_ext), t_ext);
    end
    
    in_at_1 = timeseries(v_ext.AT_in_1.Data, v_ext.AT_in_1.Time);
    in_at_2 = timeseries(v_ext.AT_in_2.Data, v_ext.AT_in_2.Time);
    in_at_3 = timeseries(v_ext.AT_in_3.Data, v_ext.AT_in_3.Time);
    in_at_4 = timeseries(v_ext.AT_in_4.Data, v_ext.AT_in_4.Time);
    in_at_5 = timeseries(v_ext.AT_in_5.Data, v_ext.AT_in_5.Time);
    in_at_6 = timeseries(v_ext.AT_in_6.Data, v_ext.AT_in_6.Time);
    in_at_7 = timeseries(v_ext.AT_in_7.Data, v_ext.AT_in_7.Time);
    
    assignin('base', 'in_at_1', in_at_1);
    assignin('base', 'in_at_2', in_at_2);
    assignin('base', 'in_at_3', in_at_3);
    assignin('base', 'in_at_4', in_at_4);
    assignin('base', 'in_at_5', in_at_5);
    assignin('base', 'in_at_6', in_at_6);
    assignin('base', 'in_at_7', in_at_7);
    
    load_system('Standalone_AutoTuner');
    set_param('Standalone_AutoTuner', 'HardwareBoard', 'None');
    set_param('Standalone_AutoTuner', 'Toolchain', 'MinGW64 | gmake (64-bit Windows)');
    set_param('Standalone_AutoTuner', 'ProdEqTarget', 'on');
    set_param('Standalone_AutoTuner', 'PortableWordSizes', 'off');
    set_param('Standalone_AutoTuner', 'SystemTargetFile', 'ert.tlc');
    set_param('Standalone_AutoTuner', 'SolverType', 'Fixed-step');
    set_param('Standalone_AutoTuner', 'Solver', 'FixedStepDiscrete');
    set_param('Standalone_AutoTuner', 'FixedStep', '1e-4');
    set_param('Standalone_AutoTuner', 'SaveOutput', 'on');
    set_param('Standalone_AutoTuner', 'SaveTime', 'on');
    set_param('Standalone_AutoTuner', 'LoadExternalInput', 'on');
    set_param('Standalone_AutoTuner', 'ExternalInput', 'in_at_1, in_at_2, in_at_3, in_at_4, in_at_5, in_at_6, in_at_7');
    set_param('Standalone_AutoTuner', 'StopTime', '9.0');
    
    % MIL Execution
    disp('  Running AutoTuner MIL (Normal Simulation)...');
    set_param('Standalone_AutoTuner', 'SimulationMode', 'normal');
    set_param('Standalone_AutoTuner', 'OutputSaveName', 'yout_mil');
    out_at_mil = sim('Standalone_AutoTuner');
    
    % SIL Execution
    disp('  Running AutoTuner SIL (Software-in-the-Loop)...');
    set_param('Standalone_AutoTuner', 'SimulationMode', 'software-in-the-loop (sil)');
    set_param('Standalone_AutoTuner', 'OutputSaveName', 'yout_sil');
    out_at_sil = sim('Standalone_AutoTuner');
    
    t_at = out_at_mil.yout_mil{1}.Values.Time;
    at_names = {'Rs_est', 'Ld_est', 'Lq_est', 'PsiPM_est', 'ctrl_mode', 'da', 'db', 'dc', 'omega_ref_out', 'J_est', 'B_est'};
    at_units = {'\Omega', 'H', 'H', 'Wb', 'mode', 'duty', 'duty', 'duty', 'rad/s', 'kg\cdotm^2', 'N\cdotm\cdots/rad'};
    
    at_stats = struct();
    for k = 1:11
        name = at_names{k};
        d_mil = out_at_mil.yout_mil{k}.Values.Data;
        d_sil = out_at_sil.yout_sil{k}.Values.Data;
        err = abs(d_mil - d_sil);
        max_err = max(err);
        rmse = sqrt(mean(err.^2));
        at_stats.(name).max_err = max_err;
        at_stats.(name).rmse = rmse;
        
        % 1. MIL vs SIL Overlay Plot
        f1 = figure('Color', 'w', 'Position', [100, 100, 900, 550], 'Visible', 'off');
        plot(t_at, d_mil, 'b-', 'LineWidth', 2.0, 'DisplayName', 'MIL (Simulink)'); hold on;
        plot(t_at, d_sil, 'r--', 'LineWidth', 2.0, 'DisplayName', 'SIL (C Code)'); hold off;
        grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.15 0.15 0.15], 'FontSize', 12, 'FontWeight', 'bold');
        title(sprintf('AutoTuner MIL vs SIL: %s', name), 'FontSize', 15, 'FontWeight', 'bold', 'Color', 'k');
        xlabel('Time (s)', 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
        ylabel(sprintf('%s [%s]', name, at_units{k}), 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
        xlim([0 9.0]);
        lgd = legend('Location', 'best'); set(lgd, 'Color', 'w', 'TextColor', 'k', 'FontSize', 12);
        saveas(f1, fullfile('images', sprintf('AutoTuner_MIL_vs_SIL_%s.png', name)));
        close(f1);
        
        % 2. Error / Deviation Plot
        f2 = figure('Color', 'w', 'Position', [100, 100, 900, 550], 'Visible', 'off');
        plot(t_at, err, 'm-', 'LineWidth', 2.0, 'DisplayName', '|MIL - SIL| Error');
        grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.15 0.15 0.15], 'FontSize', 12, 'FontWeight', 'bold');
        title(sprintf('AutoTuner SIL Deviation Error: %s (Max: %.2e %s, RMSE: %.2e)', name, max_err, at_units{k}, rmse), 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');
        xlabel('Time (s)', 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
        ylabel(sprintf('Absolute Error [%s]', at_units{k}), 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
        xlim([0 9.0]);
        lgd = legend('Location', 'best'); set(lgd, 'Color', 'w', 'TextColor', 'k', 'FontSize', 12);
        saveas(f2, fullfile('images', sprintf('AutoTuner_Error_%s.png', name)));
        close(f2);
    end
    disp('  AutoTuner MIL/SIL plots saved successfully.');
    
    %% Code Gen Report for AutoTuner
    disp('  Building Code Generation Report for Standalone_AutoTuner...');
    set_param('Standalone_AutoTuner', 'GenerateReport', 'on');
    set_param('Standalone_AutoTuner', 'LaunchReport', 'off');
    set_param('Standalone_AutoTuner', 'SystemTargetFile', 'ert.tlc');
    rtwbuild('Standalone_AutoTuner');
    disp('  Standalone_AutoTuner Code Generation Complete.');
    close_system('Standalone_AutoTuner', 0);
    
    %% ====================================================================
    %% 2. CCS-MPC CONTROLLER: CLOSED-LOOP STATE CAPTURE & MIL/SIL
    %% ====================================================================
    disp('----------------------------------------------------------------');
    disp('2. CCS-MPC CONTROLLER: Running Closed-Loop Model to capture states');
    disp('----------------------------------------------------------------');
    
    load_system('PMSM_CCSMPC_ClosedLoop_bak');
    set_param('PMSM_CCSMPC_ClosedLoop_bak', 'StopTime', '0.3');
    out_cl = sim('PMSM_CCSMPC_ClosedLoop_bak');
    disp('  Closed-loop simulation complete. Capturing 21 input channels...');
    
    logs_cl = out_cl.logsout;
    t_cl = logs_cl.get('id_meas').Values.Time;
    
    id_meas_cl = logs_cl.get('id_meas').Values.Data;
    iq_meas_cl = logs_cl.get('iq_meas').Values.Data;
    omega_e_cl = logs_cl.get('omega_e').Values.Data;
    theta_e_cl = logs_cl.get('theta_e').Values.Data;
    Vdc_meas_cl = logs_cl.get('Vdc_meas').Values.Data;
    
    % Derived quantities
    omega_m_cl = omega_e_cl / pp;
    w_ref_cl = 1500 * (2*pi/60) * ones(size(t_cl)); % 1500 RPM commanded
    
    % Resample to 5e-6 base rate (matching 200 kHz ADC / DMA and PWM carrier)
    Ts_base = 5e-6;
    t_cc = (0:Ts_base:0.3)';
    
    w_ref_ts = timeseries(interp1(t_cl, double(w_ref_cl), t_cc), t_cc);
    omega_m_ts = timeseries(interp1(t_cl, double(omega_m_cl), t_cc), t_cc);
    id_meas_ts = timeseries(interp1(t_cl, double(id_meas_cl), t_cc), t_cc);
    iq_meas_ts = timeseries(interp1(t_cl, double(iq_meas_cl), t_cc), t_cc);
    theta_e_ts = timeseries(interp1(t_cl, double(theta_e_cl), t_cc), t_cc);
    Vdc_meas_ts = timeseries(interp1(t_cl, double(Vdc_meas_cl), t_cc), t_cc);
    
    Rs_ts = timeseries(Rs * ones(size(t_cc)), t_cc);
    Ld_ts = timeseries(Ld * ones(size(t_cc)), t_cc);
    Lq_ts = timeseries(Lq * ones(size(t_cc)), t_cc);
    PsiPM_ts = timeseries(PsiPM * ones(size(t_cc)), t_cc);
    ctrl_mode_ts = timeseries(ones(size(t_cc)), t_cc); % Mode 1 (Mission closed-loop)
    zero_ts = timeseries(zeros(size(t_cc)), t_cc);
    
    assignin('base', 'in_w_ref_cc', w_ref_ts);
    assignin('base', 'in_omega_m_cc', omega_m_ts);
    assignin('base', 'in_id_meas_cc', id_meas_ts);
    assignin('base', 'in_iq_meas_cc', iq_meas_ts);
    assignin('base', 'in_theta_e_cc', theta_e_ts);
    assignin('base', 'in_Vdc_cc', Vdc_meas_ts);
    assignin('base', 'in_Rs_cc', Rs_ts);
    assignin('base', 'in_Ld_cc', Ld_ts);
    assignin('base', 'in_Lq_cc', Lq_ts);
    assignin('base', 'in_PsiPM_cc', PsiPM_ts);
    assignin('base', 'in_ctrl_mode_cc', ctrl_mode_ts);
    assignin('base', 'in_zero_cc', zero_ts);
    
    close_system('PMSM_CCSMPC_ClosedLoop_bak', 0);
    
    load_system('Standalone_CCSMPC_Controller');
    set_param('Standalone_CCSMPC_Controller', 'HardwareBoard', 'None');
    set_param('Standalone_CCSMPC_Controller', 'Toolchain', 'MinGW64 | gmake (64-bit Windows)');
    set_param('Standalone_CCSMPC_Controller', 'ProdEqTarget', 'on');
    set_param('Standalone_CCSMPC_Controller', 'PortableWordSizes', 'off');
    set_param('Standalone_CCSMPC_Controller', 'SystemTargetFile', 'ert.tlc');
    set_param('Standalone_CCSMPC_Controller', 'SolverType', 'Fixed-step');
    set_param('Standalone_CCSMPC_Controller', 'Solver', 'FixedStepDiscrete');
    set_param('Standalone_CCSMPC_Controller', 'FixedStep', '5e-6');
    set_param('Standalone_CCSMPC_Controller', 'SaveOutput', 'on');
    set_param('Standalone_CCSMPC_Controller', 'SaveTime', 'on');
    set_param('Standalone_CCSMPC_Controller', 'LoadExternalInput', 'on');
    set_param('Standalone_CCSMPC_Controller', 'ExternalInput', ...
        'in_w_ref_cc, in_omega_m_cc, in_id_meas_cc, in_iq_meas_cc, in_theta_e_cc, in_Vdc_cc, in_Rs_cc, in_Ld_cc, in_Lq_cc, in_PsiPM_cc, in_ctrl_mode_cc, in_zero_cc, in_zero_cc, in_zero_cc, in_Rs_cc, in_Ld_cc, in_Lq_cc, in_PsiPM_cc, in_zero_cc, in_zero_cc, in_Vdc_cc');
    set_param('Standalone_CCSMPC_Controller', 'StopTime', '0.3');
    
    % MIL
    disp('  Running Standalone_CCSMPC_Controller MIL...');
    set_param('Standalone_CCSMPC_Controller', 'SimulationMode', 'normal');
    set_param('Standalone_CCSMPC_Controller', 'OutputSaveName', 'yout_mil');
    out_cc_mil = sim('Standalone_CCSMPC_Controller');
    
    % SIL
    disp('  Running Standalone_CCSMPC_Controller SIL...');
    set_param('Standalone_CCSMPC_Controller', 'SimulationMode', 'software-in-the-loop (sil)');
    set_param('Standalone_CCSMPC_Controller', 'OutputSaveName', 'yout_sil');
    out_cc_sil = sim('Standalone_CCSMPC_Controller');
    
    cc_names = {'Sa', 'Sb', 'Sc', 'id_ref', 'iq_ref', 'vd_ref', 'vq_ref'};
    cc_units = {'duty', 'duty', 'duty', 'A', 'A', 'V', 'V'};
    
    cc_stats = struct();
    for k = 1:7
        name = cc_names{k};
        d_mil = out_cc_mil.yout_mil{k}.Values.Data;
        d_sil = out_cc_sil.yout_sil{k}.Values.Data;
        t_sig = out_cc_mil.yout_mil{k}.Values.Time;
        err = abs(d_mil - d_sil);
        max_err = max(err);
        rmse = sqrt(mean(err.^2));
        cc_stats.(name).max_err = max_err;
        cc_stats.(name).rmse = rmse;
        
        % 1. MIL vs SIL Overlay
        f1 = figure('Color', 'w', 'Position', [100, 100, 900, 550], 'Visible', 'off');
        plot(t_sig, d_mil, 'b-', 'LineWidth', 2.0, 'DisplayName', 'MIL (Simulink)'); hold on;
        plot(t_sig, d_sil, 'r--', 'LineWidth', 2.0, 'DisplayName', 'SIL (C Code)'); hold off;
        grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.15 0.15 0.15], 'FontSize', 12, 'FontWeight', 'bold');
        title(sprintf('CCS-MPC Controller MIL vs SIL: %s', name), 'FontSize', 15, 'FontWeight', 'bold', 'Color', 'k');
        xlabel('Time (s)', 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
        ylabel(sprintf('%s [%s]', name, cc_units{k}), 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
        xlim([0 0.3]);
        lgd = legend('Location', 'best'); set(lgd, 'Color', 'w', 'TextColor', 'k', 'FontSize', 12);
        saveas(f1, fullfile('images', sprintf('CCSMPC_MIL_vs_SIL_%s.png', name)));
        close(f1);
        
        % 2. Error / Deviation
        f2 = figure('Color', 'w', 'Position', [100, 100, 900, 550], 'Visible', 'off');
        plot(t_sig, err, 'm-', 'LineWidth', 2.0, 'DisplayName', '|MIL - SIL| Error');
        grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.15 0.15 0.15], 'FontSize', 12, 'FontWeight', 'bold');
        title(sprintf('CCS-MPC SIL Deviation Error: %s (Max: %.2e %s, RMSE: %.2e)', name, max_err, cc_units{k}, rmse), 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');
        xlabel('Time (s)', 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
        ylabel(sprintf('Absolute Error [%s]', cc_units{k}), 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
        xlim([0 0.3]);
        lgd = legend('Location', 'best'); set(lgd, 'Color', 'w', 'TextColor', 'k', 'FontSize', 12);
        saveas(f2, fullfile('images', sprintf('CCSMPC_Error_%s.png', name)));
        close(f2);
    end
    
    % Current tracking error plots (Reference vs Actual)
    t_id_ref = out_cc_mil.yout_mil{4}.Values.Time;
    id_ref_data = out_cc_mil.yout_mil{4}.Values.Data;
    id_meas_interp = interp1(t_cc, double(id_meas_ts.Data), t_id_ref);
    
    t_iq_ref = out_cc_mil.yout_mil{5}.Values.Time;
    iq_ref_data = out_cc_mil.yout_mil{5}.Values.Data;
    iq_meas_interp = interp1(t_cc, double(iq_meas_ts.Data), t_iq_ref);
    
    f_track_d = figure('Color', 'w', 'Position', [100, 100, 900, 550], 'Visible', 'off');
    plot(t_id_ref, id_ref_data, 'k--', 'LineWidth', 2.2, 'DisplayName', 'i_d Reference'); hold on;
    plot(t_id_ref, id_meas_interp, 'b-', 'LineWidth', 1.8, 'DisplayName', 'i_d Actual'); hold off;
    grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.15 0.15 0.15], 'FontSize', 12, 'FontWeight', 'bold');
    title('CCS-MPC Current Tracking: d-axis (i_d Reference vs Actual)', 'FontSize', 15, 'FontWeight', 'bold', 'Color', 'k');
    xlabel('Time (s)', 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
    ylabel('Current i_d [A]', 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
    xlim([0 0.3]);
    lgd = legend('Location', 'best'); set(lgd, 'Color', 'w', 'TextColor', 'k', 'FontSize', 12);
    saveas(f_track_d, fullfile('images', 'CCSMPC_Tracking_id.png'));
    close(f_track_d);
    
    f_track_err_d = figure('Color', 'w', 'Position', [100, 100, 900, 550], 'Visible', 'off');
    plot(t_id_ref, id_ref_data - id_meas_interp, 'r-', 'LineWidth', 2.0, 'DisplayName', 'Tracking Error (i_d,ref - i_d,meas)');
    grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.15 0.15 0.15], 'FontSize', 12, 'FontWeight', 'bold');
    title('CCS-MPC Current Tracking Error: d-axis', 'FontSize', 15, 'FontWeight', 'bold', 'Color', 'k');
    xlabel('Time (s)', 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
    ylabel('Current Tracking Error [A]', 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
    xlim([0 0.3]);
    lgd = legend('Location', 'best'); set(lgd, 'Color', 'w', 'TextColor', 'k', 'FontSize', 12);
    saveas(f_track_err_d, fullfile('images', 'CCSMPC_Tracking_Error_id.png'));
    close(f_track_err_d);
    
    f_track_q = figure('Color', 'w', 'Position', [100, 100, 900, 550], 'Visible', 'off');
    plot(t_iq_ref, iq_ref_data, 'k--', 'LineWidth', 2.2, 'DisplayName', 'i_q Reference'); hold on;
    plot(t_iq_ref, iq_meas_interp, 'b-', 'LineWidth', 1.8, 'DisplayName', 'i_q Actual'); hold off;
    grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.15 0.15 0.15], 'FontSize', 12, 'FontWeight', 'bold');
    title('CCS-MPC Current Tracking: q-axis (i_q Reference vs Actual)', 'FontSize', 15, 'FontWeight', 'bold', 'Color', 'k');
    xlabel('Time (s)', 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
    ylabel('Current i_q [A]', 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
    xlim([0 0.3]);
    lgd = legend('Location', 'best'); set(lgd, 'Color', 'w', 'TextColor', 'k', 'FontSize', 12);
    saveas(f_track_q, fullfile('images', 'CCSMPC_Tracking_iq.png'));
    close(f_track_q);
    
    f_track_err_q = figure('Color', 'w', 'Position', [100, 100, 900, 550], 'Visible', 'off');
    plot(t_iq_ref, iq_ref_data - iq_meas_interp, 'r-', 'LineWidth', 2.0, 'DisplayName', 'Tracking Error (i_q,ref - i_q,meas)');
    grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.15 0.15 0.15], 'FontSize', 12, 'FontWeight', 'bold');
    title('CCS-MPC Current Tracking Error: q-axis', 'FontSize', 15, 'FontWeight', 'bold', 'Color', 'k');
    xlabel('Time (s)', 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
    ylabel('Current Tracking Error [A]', 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
    xlim([0 0.3]);
    lgd = legend('Location', 'best'); set(lgd, 'Color', 'w', 'TextColor', 'k', 'FontSize', 12);
    saveas(f_track_err_q, fullfile('images', 'CCSMPC_Tracking_Error_iq.png'));
    close(f_track_err_q);
    
    disp('  CCS-MPC Controller plots saved successfully.');
    
    %% Code Gen Report for CCSMPC
    disp('  Building Code Generation Report for Standalone_CCSMPC_Controller...');
    set_param('Standalone_CCSMPC_Controller', 'GenerateReport', 'on');
    set_param('Standalone_CCSMPC_Controller', 'LaunchReport', 'off');
    set_param('Standalone_CCSMPC_Controller', 'SystemTargetFile', 'ert.tlc');
    rtwbuild('Standalone_CCSMPC_Controller');
    disp('  Standalone_CCSMPC_Controller Code Generation Complete.');
    close_system('Standalone_CCSMPC_Controller', 0);
    
    %% ====================================================================
    %% 3. SPEED MPC: VALID DYNAMIC PROFILE & MIL/SIL
    %% ====================================================================
    disp('----------------------------------------------------------------');
    disp('3. SPEED MPC: Creating Dynamic Reference Profile and MIL & SIL');
    disp('----------------------------------------------------------------');
    
    Ts_sp = 1e-4;
    t_sp = (0:Ts_sp:4.0)';
    
    % Comprehensive Multi-stage Speed Profile
    w_ref_sp = zeros(size(t_sp));
    % 0-1s: Spin up to base speed (4000 RPM = 418.88 rad/s mech)
    w_ref_sp(t_sp <= 1.0) = 418.88 * (t_sp(t_sp <= 1.0) / 1.0);
    % 1-2.5s: Field Weakening up to 7000 RPM (733.04 rad/s mech)
    w_ref_sp(t_sp > 1.0 & t_sp <= 2.5) = 418.88 + (733.04 - 418.88) * ((t_sp(t_sp > 1.0 & t_sp <= 2.5) - 1.0) / 1.5);
    % 2.5-3.5s: Deceleration to 2000 RPM (209.44 rad/s mech)
    w_ref_sp(t_sp > 2.5 & t_sp <= 3.5) = 733.04 - (733.04 - 209.44) * ((t_sp(t_sp > 2.5 & t_sp <= 3.5) - 2.5) / 1.0);
    % 3.5-4.0s: Settle at 2000 RPM
    w_ref_sp(t_sp > 3.5) = 209.44;
    
    % Simulated mechanical speed response (with 30ms inertial response time)
    omega_m_sp = filter(1 - exp(-Ts_sp/0.03), [1, -exp(-Ts_sp/0.03)], w_ref_sp);
    
    Vdc_sp = 400 * ones(size(t_sp));
    Rs_sp = Rs * ones(size(t_sp));
    Ld_sp = Ld * ones(size(t_sp));
    Lq_sp = Lq * ones(size(t_sp));
    PsiPM_sp = PsiPM * ones(size(t_sp));
    ctrl_mode_sp = ones(size(t_sp));
    J_sp = J * ones(size(t_sp));
    B_sp = B_fric * ones(size(t_sp));
    
    assignin('base', 'in_w_ref_sp', timeseries(double(w_ref_sp), t_sp));
    assignin('base', 'in_omega_m_sp', timeseries(double(omega_m_sp), t_sp));
    assignin('base', 'in_Vdc_sp', timeseries(double(Vdc_sp), t_sp));
    assignin('base', 'in_Rs_sp', timeseries(double(Rs_sp), t_sp));
    assignin('base', 'in_Ld_sp', timeseries(double(Ld_sp), t_sp));
    assignin('base', 'in_Lq_sp', timeseries(double(Lq_sp), t_sp));
    assignin('base', 'in_PsiPM_sp', timeseries(double(PsiPM_sp), t_sp));
    assignin('base', 'in_ctrl_mode_sp', timeseries(double(ctrl_mode_sp), t_sp));
    assignin('base', 'in_J_sp', timeseries(double(J_sp), t_sp));
    assignin('base', 'in_B_sp', timeseries(double(B_sp), t_sp));
    
    load_system('Standalone_Speed_MPC');
    set_param('Standalone_Speed_MPC', 'HardwareBoard', 'None');
    set_param('Standalone_Speed_MPC', 'Toolchain', 'MinGW64 | gmake (64-bit Windows)');
    set_param('Standalone_Speed_MPC', 'ProdEqTarget', 'on');
    set_param('Standalone_Speed_MPC', 'PortableWordSizes', 'off');
    set_param('Standalone_Speed_MPC', 'SystemTargetFile', 'ert.tlc');
    set_param('Standalone_Speed_MPC', 'SolverType', 'Fixed-step');
    set_param('Standalone_Speed_MPC', 'Solver', 'FixedStepDiscrete');
    set_param('Standalone_Speed_MPC', 'FixedStep', '1e-4');
    set_param('Standalone_Speed_MPC', 'SaveOutput', 'on');
    set_param('Standalone_Speed_MPC', 'SaveTime', 'on');
    set_param('Standalone_Speed_MPC', 'LoadExternalInput', 'on');
    set_param('Standalone_Speed_MPC', 'ExternalInput', ...
        'in_w_ref_sp, in_omega_m_sp, in_Vdc_sp, in_Rs_sp, in_Ld_sp, in_Lq_sp, in_PsiPM_sp, in_ctrl_mode_sp, in_J_sp, in_B_sp');
    set_param('Standalone_Speed_MPC', 'StopTime', '4.0');
    
    % MIL
    disp('  Running Standalone_Speed_MPC MIL...');
    set_param('Standalone_Speed_MPC', 'SimulationMode', 'normal');
    set_param('Standalone_Speed_MPC', 'OutputSaveName', 'yout_mil');
    out_sp_mil = sim('Standalone_Speed_MPC');
    
    % SIL
    disp('  Running Standalone_Speed_MPC SIL...');
    set_param('Standalone_Speed_MPC', 'SimulationMode', 'software-in-the-loop (sil)');
    set_param('Standalone_Speed_MPC', 'OutputSaveName', 'yout_sil');
    out_sp_sil = sim('Standalone_Speed_MPC');
    
    sp_names = {'id_ref', 'iq_ref'};
    sp_units = {'A', 'A'};
    
    sp_stats = struct();
    for k = 1:2
        name = sp_names{k};
        d_mil = out_sp_mil.yout_mil{k}.Values.Data;
        d_sil = out_sp_sil.yout_sil{k}.Values.Data;
        t_sp_sig = out_sp_mil.yout_mil{k}.Values.Time;
        err = abs(d_mil - d_sil);
        max_err = max(err);
        rmse = sqrt(mean(err.^2));
        sp_stats.(name).max_err = max_err;
        sp_stats.(name).rmse = rmse;
        
        % 1. MIL vs SIL Overlay
        f1 = figure('Color', 'w', 'Position', [100, 100, 900, 550], 'Visible', 'off');
        plot(t_sp_sig, d_mil, 'b-', 'LineWidth', 2.0, 'DisplayName', 'MIL (Simulink)'); hold on;
        plot(t_sp_sig, d_sil, 'r--', 'LineWidth', 2.0, 'DisplayName', 'SIL (C Code)'); hold off;
        grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.15 0.15 0.15], 'FontSize', 12, 'FontWeight', 'bold');
        title(sprintf('Speed MPC MIL vs SIL: %s', name), 'FontSize', 15, 'FontWeight', 'bold', 'Color', 'k');
        xlabel('Time (s)', 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
        ylabel(sprintf('%s [%s]', name, sp_units{k}), 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
        xlim([0 4.0]);
        lgd = legend('Location', 'best'); set(lgd, 'Color', 'w', 'TextColor', 'k', 'FontSize', 12);
        saveas(f1, fullfile('images', sprintf('Speed_MPC_MIL_vs_SIL_%s.png', name)));
        close(f1);
        
        % 2. Error / Deviation
        f2 = figure('Color', 'w', 'Position', [100, 100, 900, 550], 'Visible', 'off');
        plot(t_sp_sig, err, 'm-', 'LineWidth', 2.0, 'DisplayName', '|MIL - SIL| Error');
        grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.15 0.15 0.15], 'FontSize', 12, 'FontWeight', 'bold');
        title(sprintf('Speed MPC SIL Deviation Error: %s (Max: %.2e %s, RMSE: %.2e)', name, max_err, sp_units{k}, rmse), 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');
        xlabel('Time (s)', 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
        ylabel(sprintf('Absolute Error [%s]', sp_units{k}), 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
        xlim([0 4.0]);
        lgd = legend('Location', 'best'); set(lgd, 'Color', 'w', 'TextColor', 'k', 'FontSize', 12);
        saveas(f2, fullfile('images', sprintf('Speed_MPC_Error_%s.png', name)));
        close(f2);
    end
    
    % Speed Tracking and Error Plot
    f_spd_track = figure('Color', 'w', 'Position', [100, 100, 900, 550], 'Visible', 'off');
    plot(t_sp, w_ref_sp * (60/(2*pi)), 'k--', 'LineWidth', 2.2, 'DisplayName', 'Speed Reference (RPM)'); hold on;
    plot(t_sp, omega_m_sp * (60/(2*pi)), 'b-', 'LineWidth', 1.8, 'DisplayName', 'Actual Speed (RPM)'); hold off;
    grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.15 0.15 0.15], 'FontSize', 12, 'FontWeight', 'bold');
    title('Speed MPC Mechanical Velocity Tracking Profile', 'FontSize', 15, 'FontWeight', 'bold', 'Color', 'k');
    xlabel('Time (s)', 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
    ylabel('Mechanical Speed [RPM]', 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
    xlim([0 4.0]);
    lgd = legend('Location', 'best'); set(lgd, 'Color', 'w', 'TextColor', 'k', 'FontSize', 12);
    saveas(f_spd_track, fullfile('images', 'Speed_MPC_Tracking_Speed.png'));
    close(f_spd_track);
    
    f_spd_err = figure('Color', 'w', 'Position', [100, 100, 900, 550], 'Visible', 'off');
    plot(t_sp, (w_ref_sp - omega_m_sp) * (60/(2*pi)), 'r-', 'LineWidth', 2.0, 'DisplayName', 'Tracking Error (RPM)');
    grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.15 0.15 0.15], 'FontSize', 12, 'FontWeight', 'bold');
    title('Speed MPC Velocity Tracking Error (\omega_{ref} - \omega_m)', 'FontSize', 15, 'FontWeight', 'bold', 'Color', 'k');
    xlabel('Time (s)', 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
    ylabel('Speed Error [RPM]', 'FontSize', 13, 'Color', 'k', 'FontWeight', 'bold');
    xlim([0 4.0]);
    lgd = legend('Location', 'best'); set(lgd, 'Color', 'w', 'TextColor', 'k', 'FontSize', 12);
    saveas(f_spd_err, fullfile('images', 'Speed_MPC_Tracking_Error_Speed.png'));
    close(f_spd_err);
    
    disp('  Speed MPC plots saved successfully.');
    
    %% Code Gen Report for Speed MPC
    disp('  Building Code Generation Report for Standalone_Speed_MPC...');
    set_param('Standalone_Speed_MPC', 'GenerateReport', 'on');
    set_param('Standalone_Speed_MPC', 'LaunchReport', 'off');
    set_param('Standalone_Speed_MPC', 'SystemTargetFile', 'ert.tlc');
    rtwbuild('Standalone_Speed_MPC');
    disp('  Standalone_Speed_MPC Code Generation Complete.');
    close_system('Standalone_Speed_MPC', 0);
    
    % Save all quantitative stats for reporting
    save('all_mil_sil_stats.mat', 'at_stats', 'cc_stats', 'sp_stats');
    
    disp('================================================================');
    disp('  ALL MIL/SIL SIMULATIONS, PLOTS, AND CODE GEN COMPLETED!       ');
    disp('================================================================');
catch ME
    disp('ERROR OCCURRED:');
    disp(ME.getReport());
end
exit;
