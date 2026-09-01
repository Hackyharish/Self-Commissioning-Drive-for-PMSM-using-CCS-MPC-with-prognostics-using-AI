%% plot_mtpa_mtpv_envelope.m
% =========================================================================
% BMW i3 IPMSM - MTPA & MTPV COMPLETE CONTROL ENVELOPE & TORQUE-SPEED MAP
% =========================================================================
% Panel 1: Clean (i_d, i_q) Current Vector Plane (Zero In-Plot Obstruction)
% Panel 2: Maximum Torque & Mechanical Power vs. Speed (0 - 11400 RPM)
% =========================================================================

clear; clc; close all;

%% 1. Plant Parameters (BMW i3 IPMSM)
params_pmsm_inverter;

p      = pp;                  % 6
dL     = Lq - Ld;            % 70.1 uH
Imax   = I_peak;             % 565.685 A
Vmax   = Vdc / sqrt(3.0);    % 230.94 V
id_ctr = -PsiPM / Ld;        % -612.36 A
n_base = 4000;               % Base speed = 4000 RPM
n_B    = 8500;               % MTPV transition speed = 8500 RPM
n_max  = 11400;              % Maximum vehicle speed = 11400 RPM

%% 2. Key Operating Points
% Point A: Rated MTPA Intersection (Te = 258.2 Nm, Is = 565.7 A)
id_A = -184.5;
iq_A = 534.8;
Te_rated = 258.2;                                      % Nm
P_rated  = Te_rated * (n_base * 2*pi / 60) / 1000.0;  % 108.15 kW

% Point B: MTPV Transition Corner (8500 RPM)
id_B = -466.0;
iq_B = 320.0;

% Point C: Max Speed MTPV Operating Point (11400 RPM)
id_C = -515.0;
iq_C = 175.0;

%% 3. Generate Analytical Torque & Power Capability Profiles vs Speed
n_vec = linspace(0, 11400, 1000);
Te_vec = zeros(size(n_vec));
P_vec  = zeros(size(n_vec));
id_vec = zeros(size(n_vec));
iq_vec = zeros(size(n_vec));

for i = 1:length(n_vec)
    nr = n_vec(i);
    wm = nr * (2*pi/60);
    
    if nr <= n_base
        % Region I: Constant Maximum Torque Mode (0 - 4000 RPM)
        Te_vec(i) = Te_rated;
        P_vec(i)  = Te_rated * wm / 1000.0; % Linear ramp: 0 -> 108.15 kW
        id_vec(i) = id_A;
        iq_vec(i) = iq_A;
    elseif nr <= n_B
        % Region II: Constant Power Field-Weakening Mode (4000 - 8500 RPM)
        P_vec(i)  = P_rated;                % Strictly flat at 108.15 kW
        Te_vec(i) = (P_rated * 1000.0) / wm; % Drops inversely with speed (258.2 -> 121.5 Nm)
        
        % Current slides along Imax circle (A -> B)
        ratio_ab = (nr - n_base) / (n_B - n_base);
        id_vec(i) = id_A + ratio_ab * (id_B - id_A);
        iq_vec(i) = sqrt(max(0, Imax^2 - id_vec(i)^2));
    else
        % Region III: MTPV High-Speed Power Roll-Off (8500 - 11400 RPM)
        Te_vec(i) = (P_rated * 1000.0 / (n_B * 2*pi/60)) * (n_B / nr)^2;
        P_vec(i)  = Te_vec(i) * wm / 1000.0; % Rolls off from 108.15 kW -> 80.6 kW
        
        % Current tracks MTPV locus inside circle (B -> C)
        ratio_bc = (nr - n_B) / (n_max - n_B);
        id_vec(i) = id_B + ratio_bc * (id_C - id_B);
        iq_vec(i) = iq_B + ratio_bc * (iq_C - iq_B);
    end
end

%% 4. Figure Layout (2 Clean Side-by-Side Panels)
f = figure('Name', 'BMW i3 IPMSM - Control Envelope & Capability Curves', ...
           'Color', 'w', 'Position', [60, 60, 1350, 680]);

%% =========================================================================
%% PANEL 1 (LEFT): UNCLUTTERED CURRENT VECTOR PLANE (i_d, i_q)
%% =========================================================================
ax1 = subplot(1, 2, 1);
hold(ax1, 'on');
grid(ax1, 'on');
box(ax1, 'on');

xlim(ax1, [-750, 40]);
ylim(ax1, [0, 620]);

set(ax1, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
         'GridColor', [0.85 0.85 0.85], 'GridAlpha', 1.0, ...
         'FontSize', 11, 'FontWeight', 'bold', 'LineWidth', 1.1);

xlabel(ax1, 'Direct Current  i_d (A)  [Demagnetizing \rightarrow]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel(ax1, 'Quadrature Current  i_q (A)  [Torque Producing]', 'FontSize', 12, 'FontWeight', 'bold');
title(ax1, 'Current Vector Plane (i_d, i_q)', 'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.05 0.15 0.4]);

% 1. Zero Reference Axes
plot(ax1, [-750, 40], [0, 0], '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.9, 'HandleVisibility', 'off');
plot(ax1, [0, 0], [0, 620], '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.9, 'HandleVisibility', 'off');

% 2. Current Limit Circle Arc
th_c = linspace(pi/2, pi, 200);
h_Ilim = plot(ax1, Imax*cos(th_c), Imax*sin(th_c), '--', 'Color', [0.15 0.65 0.30], 'LineWidth', 2.2, ...
              'DisplayName', 'Current Limit Circle (I_{max} = 565.7 A)');

% 3. Voltage Limit Concentric Ellipses (4000, 8500, 11400 RPM)
th_e = linspace(0, pi, 300);
speeds_el = [4000, 8500, 11400];
for k = 1:length(speeds_el)
    we_k = speeds_el(k) * (2*pi/60) * p;
    Vlim_k = Vmax / we_k;
    xe_k = id_ctr + (Vlim_k/Ld)*cos(th_e);
    ye_k = (Vlim_k/Lq)*sin(th_e);
    mask = (xe_k >= -750 & xe_k <= 40 & ye_k >= 0 & ye_k <= 620);
    if k == 1
        h_Vlim = plot(ax1, xe_k(mask), ye_k(mask), ':', 'Color', [0.1 0.45 0.85], 'LineWidth', 1.8, ...
                      'DisplayName', 'Voltage Ellipses (4000, 8500, 11400 RPM)');
    else
        plot(ax1, xe_k(mask), ye_k(mask), ':', 'Color', [0.1 0.45 0.85], 'LineWidth', 1.6, 'HandleVisibility', 'off');
    end
end

% 4. Constant Torque Hyperbolas (100, 180, 258.2 Nm)
iq_h = linspace(50, 600, 300);
torques = [100, 180, 258.2];
for k = 1:length(torques)
    Tk = torques(k);
    id_h = (PsiPM / dL) - (2.0 * Tk) ./ (3.0 * p * dL * iq_h);
    mask = (id_h >= -750 & id_h <= 40);
    if k == 3
        h_Trq = plot(ax1, id_h(mask), iq_h(mask), '-', 'Color', [0.85 0.50 0.50], 'LineWidth', 1.4, ...
                     'DisplayName', 'Torque Hyperbolas (100, 180, 258.2 N\cdot m)');
    else
        plot(ax1, id_h(mask), iq_h(mask), '-', 'Color', [0.85 0.65 0.65], 'LineWidth', 1.1, 'HandleVisibility', 'off');
    end
end

% 5. Active Operating Trajectory
% Region I: MTPA (Origin -> Point A)
iq_mtpa_line = linspace(0, iq_A, 200);
id_mtpa_line = (PsiPM / (4.0 * dL)) - sqrt((PsiPM^2 / (16.0 * dL^2)) + (iq_mtpa_line.^2 / 2.0));
h_reg1 = plot(ax1, id_mtpa_line, iq_mtpa_line, 'r-', 'LineWidth', 3.2, ...
              'DisplayName', 'Region I: MTPA Locus (0 \rightarrow 4000 RPM)');

% Region II: Field-Weakening Arc along Current Limit Circle (Point A -> Point B)
th_fw_arc = linspace(atan2(iq_A, id_A), atan2(iq_B, id_B), 150);
h_reg2 = plot(ax1, Imax*cos(th_fw_arc), Imax*sin(th_fw_arc), '-', 'Color', [0.0 0.35 0.85], 'LineWidth', 3.2, ...
              'DisplayName', 'Region II: FW Arc on I_{max} (4000 \rightarrow 8500 RPM)');

% Region III: MTPV Locus (Point B -> Point C)
iq_v_line = linspace(iq_B, iq_C, 150);
term_v_line = sqrt((Lq * PsiPM)^2 + 8.0 * (dL^2) * (Lq * iq_v_line).^2);
id_v_line = id_ctr + (-Lq * PsiPM + term_v_line) ./ (4.0 * dL * Ld);
h_reg3 = plot(ax1, id_v_line, iq_v_line, '--', 'Color', [0.55 0.0 0.75], 'LineWidth', 2.8, ...
              'DisplayName', 'Region III: MTPV Locus (8500 \rightarrow 11400 RPM)');

% 6. Distinct Key Operating Points
plot(ax1, id_A, iq_A, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
plot(ax1, id_B, iq_B, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', [0.0 0.35 0.85], 'HandleVisibility', 'off');
plot(ax1, id_C, iq_C, 'mo', 'MarkerSize', 8, 'MarkerFaceColor', [0.55 0 0.75], 'HandleVisibility', 'off');
plot(ax1, id_ctr, 0, 'k+', 'MarkerSize', 8, 'LineWidth', 1.8, 'HandleVisibility', 'off');

text(ax1, id_A + 12, iq_A + 8, '\bf Point A (4000 RPM, Rated)', 'FontSize', 9.5, 'Color', [0.8 0 0]);
text(ax1, id_B - 20, iq_B + 16, '\bf Point B (8500 RPM, MTPV)', 'FontSize', 9.5, 'Color', [0.0 0.25 0.80], 'HorizontalAlignment', 'right');
text(ax1, id_C - 20, iq_C + 16, '\bf Point C (11400 RPM, Max)', 'FontSize', 9.5, 'Color', [0.5 0 0.7], 'HorizontalAlignment', 'right');
text(ax1, id_ctr, 20, '-\Psi_{PM}/L_d', 'FontSize', 8.5, 'Color', [0.3 0.3 0.3], 'HorizontalAlignment', 'center');

% 7. UNCLUTTERED TOP LEGEND (Above Axes so it never covers plot data)
lgd1 = legend(ax1, [h_reg1, h_reg2, h_reg3, h_Ilim, h_Vlim, h_Trq], ...
              'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', 8.5);
set(lgd1, 'Color', 'w', 'EdgeColor', [0.75 0.75 0.75], 'LineWidth', 0.9);

%% =========================================================================
%% PANEL 2 (RIGHT): TORQUE, POWER & OPTIMAL CURRENTS VS. SPEED
%% =========================================================================

% Top Right: Torque & Power vs Speed (Constant Torque -> Constant Power -> MTPV Roll-Off)
ax2_top = subplot(2, 2, 2);
hold(ax2_top, 'on'); grid(ax2_top, 'on'); box(ax2_top, 'on');
set(ax2_top, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
             'GridColor', [0.85 0.85 0.85], 'GridAlpha', 1.0, ...
             'FontSize', 10, 'FontWeight', 'bold', 'LineWidth', 1.1);

% Left axis: Torque
yyaxis(ax2_top, 'left');
plot(ax2_top, n_vec, Te_vec, 'r-', 'LineWidth', 2.6);
ylabel(ax2_top, 'Torque T_e (N\cdot m)', 'Color', [0.8 0 0], 'FontWeight', 'bold');
ylim(ax2_top, [0, 300]);
ax2_top.YColor = [0.8 0 0];

% Right axis: Mechanical Power
yyaxis(ax2_top, 'right');
plot(ax2_top, n_vec, P_vec, 'Color', [0.0 0.45 0.85], 'LineWidth', 2.4);
ylabel(ax2_top, 'Power P_{mech} (kW)', 'Color', [0.0 0.45 0.85], 'FontWeight', 'bold');
ylim(ax2_top, [0, 140]);
ax2_top.YColor = [0.0 0.45 0.85];

xlim(ax2_top, [0, 11400]);
title(ax2_top, 'Torque & Power Envelopes vs. Speed', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.05 0.15 0.4]);

% Vertical region dividers
xline(ax2_top, 4000, 'k--', 'LineWidth', 1.2, 'Alpha', 0.7);
xline(ax2_top, 8500, 'k--', 'LineWidth', 1.2, 'Alpha', 0.7);

text(ax2_top, 2000, 275, '\bf Region I\rm (MTPA)', 'FontSize', 8.5, 'HorizontalAlignment', 'center');
text(ax2_top, 6250, 275, '\bf Region II\rm (Const. Power FW)', 'FontSize', 8.5, 'HorizontalAlignment', 'center');
text(ax2_top, 10000, 275, '\bf Region III\rm (MTPV)', 'FontSize', 8.5, 'HorizontalAlignment', 'center');

% Bottom Right: Optimal Current Allocations (id, iq, Is) vs Speed
ax2_bot = subplot(2, 2, 4);
hold(ax2_bot, 'on'); grid(ax2_bot, 'on'); box(ax2_bot, 'on');
set(ax2_bot, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
             'GridColor', [0.85 0.85 0.85], 'GridAlpha', 1.0, ...
             'FontSize', 10, 'FontWeight', 'bold', 'LineWidth', 1.1);

plot(ax2_bot, n_vec, id_vec, 'b-', 'LineWidth', 2.2, 'DisplayName', 'd-axis Current i_{d,opt}');
plot(ax2_bot, n_vec, iq_vec, 'r-', 'LineWidth', 2.2, 'DisplayName', 'q-axis Current i_{q,opt}');
plot(ax2_bot, n_vec, sqrt(id_vec.^2 + iq_vec.^2), 'k--', 'LineWidth', 1.6, 'DisplayName', 'Stator Current I_s');

xline(ax2_bot, 4000, 'k--', 'LineWidth', 1.2, 'Alpha', 0.7, 'HandleVisibility', 'off');
xline(ax2_bot, 8500, 'k--', 'LineWidth', 1.2, 'Alpha', 0.7, 'HandleVisibility', 'off');

xlabel(ax2_bot, 'Rotor Speed, n (RPM)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel(ax2_bot, 'Current (A)', 'FontSize', 11, 'FontWeight', 'bold');
title(ax2_bot, 'Optimal Current Allocations (i_d, i_q, I_s) vs. Speed', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.05 0.15 0.4]);
xlim(ax2_bot, [0, 11400]);
ylim(ax2_bot, [-560, 600]);

lgd2 = legend(ax2_bot, 'Location', 'east', 'FontSize', 8.5);
set(lgd2, 'Color', 'w', 'EdgeColor', [0.75 0.75 0.75]);

%% =========================================================================
%% 5. Save Clean Figures
%% =========================================================================
exportgraphics(f, 'MTPA_MTPV_Vector_Plane_Envelope.png', 'BackgroundColor', 'white', 'Resolution', 300);
exportgraphics(f, fullfile('images', 'MTPA_MTPV_Vector_Plane_Envelope.png'), 'BackgroundColor', 'white', 'Resolution', 300);
exportgraphics(f, fullfile('HIL', 'MTPA_MTPV_Vector_Plane_Envelope.png'), 'BackgroundColor', 'white', 'Resolution', 300);
exportgraphics(f, fullfile('HIL', 'images', 'MTPA_MTPV_Vector_Plane_Envelope.png'), 'BackgroundColor', 'white', 'Resolution', 300);

savefig(f, 'MTPA_MTPV_Vector_Plane_Envelope.fig');
savefig(f, fullfile('images', 'MTPA_MTPV_Vector_Plane_Envelope.fig'));
savefig(f, fullfile('HIL', 'MTPA_MTPV_Vector_Plane_Envelope.fig'));

disp('=== PERFECTED FLAT CONSTANT POWER 2-PANEL FIGURE GENERATED ===');
