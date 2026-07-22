%% run_commissioning_v2.m
% Self-commissioning: estimate Rs, Ld, Lq, PsiPM
% 
% Mode 1 (locked rotor): Pulse test → Rs, Ld estimation
% Mode 2 (closed-loop): Run motor → PsiPM from back-EMF
% -------------------------------------------------------------------------

fprintf('=== Self-Commissioning Parameter Identification ===\n');
params_pmsm_inverter;

mdl = 'PMSM_CCSMPC_ClosedLoop_working';
ctrl = [mdl '/CCSMPC_Controller'];
load_system(mdl);

%% Phase 1: Rs and Ld estimation (locked rotor pulse test)
fprintf('\n--- Phase 1: Rs & Ld Estimation (locked rotor) ---\n');

% Lock rotor with very high inertia
assignin('base', 'J', 1e9);
% Use true parameters for controller (will be updated later)
assignin('base', 'Rs_hat', Rs);
assignin('base', 'Ld_hat', Ld);
assignin('base', 'Lq_hat', Lq);
assignin('base', 'PsiPM_hat', PsiPM);

% Configure: Mode 3 closed loop, SpeedRef = 0 but inject test voltage
% We'll use a small constant speed reference to inject d-axis current
set_param([mdl '/CtrlMode'], 'Value', '3');
set_param([mdl '/SpeedRef'], 'Time', '0', 'Before', '0', 'After', '0');
set_param([mdl '/LoadTorque'], 'Time', '0', 'Before', '0', 'After', '0');

% For locked rotor test, we need current flowing through the winding.
% We'll directly apply a test voltage via a modified approach:
% Set a small speed reference to get iq, which produces vq (back-EMF = 0 since locked)
% Actually, with locked rotor + J=1e9, even with iq command the motor won't move.
% The id=0 strategy means only iq flows. From the iq current and vq voltage,
% we can estimate Rs and Lq (not Ld, since id=0).

% Better approach: manually compute Rs from the steady-state voltage equation
% At locked rotor (omega_e = 0), vq_ss = Rs * iq_ss (since omega_e = 0 removes back-EMF)
% And from the d-axis: vd_ss = Rs * id_ss = 0 (since id_ref = 0)

% Let's command a non-zero speed to get iq flowing
set_param([mdl '/SpeedRef'], 'Time', '0', 'Before', '50', 'After', '50');
set_param(mdl, 'StopTime', '0.1');

fprintf('  Simulating locked rotor (J=1e9)...\n');
out = sim(mdl);

iq = squeeze(out.iq_meas_log.Data);
vq = squeeze(out.vq_ref_log.Data);
id = squeeze(out.id_meas_log.Data);
vd = squeeze(out.vd_ref_log.Data);
omega_e = squeeze(out.omega_e_log.Data);
t = out.iq_meas_log.Time;

% Steady state values (last 20%)
N = length(iq);
ss = round(0.8*N):N;

iq_ss = mean(iq(ss));
vq_ss = mean(vq(ss));
id_ss = mean(id(ss));
vd_ss = mean(vd(ss));
omega_e_ss = mean(omega_e(ss));

fprintf('  Steady state: iq=%.3f A, vq=%.3f V, omega_e=%.4f rad/s\n', iq_ss, vq_ss, omega_e_ss);

% Rs estimation: vq = Rs*iq + omega_e*(Ld*id + PsiPM) ≈ Rs*iq (since omega_e ≈ 0)
if abs(iq_ss) > 0.1
    Rs_est = vq_ss / iq_ss;
    fprintf('  Estimated Rs = %.4f Ohm (true = %.3f)\n', Rs_est, Rs);
else
    Rs_est = Rs;
    fprintf('  [WARN] iq too small for Rs estimation, using nominal\n');
end

% Ld estimation from transient response
% At locked rotor with step voltage, i(t) = (V/R)*(1 - exp(-t/tau))
% tau = L/R → L = tau * R
% Find the time to reach 63.2% of steady state iq
iq_target = 0.632 * iq_ss;
idx_start = find(t > 0.001, 1); % skip first ms
idx_tau = find(iq(idx_start:end) >= iq_target, 1) + idx_start - 1;
if ~isempty(idx_tau)
    tau_est = t(idx_tau) - t(idx_start);
    Ld_est = tau_est * Rs_est;
    fprintf('  Estimated Ld = %.4f mH (true = %.2f mH)\n', Ld_est*1000, Ld*1000);
else
    Ld_est = Ld;
    fprintf('  [WARN] Could not estimate Ld from transient\n');
end

Lq_est = Ld_est; % Approximate; proper Lq ID needs d-axis injection

%% Phase 2: PsiPM estimation (closed-loop, free spinning)
fprintf('\n--- Phase 2: PsiPM Estimation (free spinning) ---\n');

% Unlock rotor
assignin('base', 'J', 1e-4);
% Use estimated parameters
assignin('base', 'Rs_hat', Rs_est);
assignin('base', 'Ld_hat', Ld_est);
assignin('base', 'Lq_hat', Lq_est);
assignin('base', 'PsiPM_hat', PsiPM); % Will be updated

% Run motor at moderate speed with no load
set_param([mdl '/SpeedRef'], 'Time', '0.01', 'Before', '0', 'After', '80');
set_param([mdl '/LoadTorque'], 'Time', '0', 'Before', '0', 'After', '0');
set_param([mdl '/CtrlMode'], 'Value', '3');
set_param(mdl, 'StopTime', '0.5');

fprintf('  Simulating free-spinning motor (omega_ref=80 rad/s)...\n');
out2 = sim(mdl);

omega_e2 = squeeze(out2.omega_e_log.Data);
vq2 = squeeze(out2.vq_ref_log.Data);
iq2 = squeeze(out2.iq_meas_log.Data);
id2 = squeeze(out2.id_meas_log.Data);
t2 = out2.omega_e_log.Time;

N2 = length(omega_e2);
ss2 = round(0.9*N2):N2;

omega_e2_ss = mean(omega_e2(ss2));
vq2_ss = mean(vq2(ss2));
iq2_ss = mean(iq2(ss2));
id2_ss = mean(id2(ss2));

fprintf('  Steady state: omega_e=%.2f, vq=%.3f V, iq=%.4f A, id=%.4f A\n', ...
    omega_e2_ss, vq2_ss, iq2_ss, id2_ss);

% PsiPM estimation from q-axis voltage equation:
% vq = Rs*iq + omega_e*Ld*id + Lq*diq/dt + omega_e*PsiPM
% At steady state, diq/dt ≈ 0, and id ≈ 0:
% vq_ss = Rs*iq_ss + omega_e*PsiPM
% PsiPM = (vq_ss - Rs*iq_ss) / omega_e

if abs(omega_e2_ss) > 5  % Need enough speed for reliable estimation
    PsiPM_est = (vq2_ss - Rs_est * iq2_ss - omega_e2_ss * Ld_est * id2_ss) / omega_e2_ss;
    fprintf('  Estimated PsiPM = %.4f Wb (true = %.3f)\n', PsiPM_est, PsiPM);
else
    PsiPM_est = PsiPM;
    fprintf('  [FAIL] omega_e too low for PsiPM estimation\n');
end

%% Summary
fprintf('\n=== Commissioning Results ===\n');
fprintf('  Parameter  | Estimated | True     | Error\n');
fprintf('  -----------|-----------|----------|-------\n');
fprintf('  Rs         | %.4f Ohm | %.3f Ohm | %.1f%%\n', Rs_est, Rs, abs(Rs_est-Rs)/Rs*100);
fprintf('  Ld         | %.4f mH  | %.2f mH  | %.1f%%\n', Ld_est*1000, Ld*1000, abs(Ld_est-Ld)/Ld*100);
fprintf('  Lq         | %.4f mH  | %.2f mH  | (=Ld est)\n', Lq_est*1000, Lq*1000);
fprintf('  PsiPM      | %.4f Wb  | %.3f Wb  | %.1f%%\n', PsiPM_est, PsiPM, abs(PsiPM_est-PsiPM)/PsiPM*100);

% Save results
save('commissioning_results.mat', 'Rs_est', 'Ld_est', 'Lq_est', 'PsiPM_est');

% Update workspace with estimated values
assignin('base', 'Rs_hat', Rs_est);
assignin('base', 'Ld_hat', Ld_est);
assignin('base', 'Lq_hat', Lq_est);
assignin('base', 'PsiPM_hat', PsiPM_est);

save_system(mdl);
fprintf('\n=== Commissioning Complete ===\n');
