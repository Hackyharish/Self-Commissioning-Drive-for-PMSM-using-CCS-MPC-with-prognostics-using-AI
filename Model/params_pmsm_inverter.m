% params_pmsm_inverter.m
% Parameters for PMSM Drive Simulation
% 
% Component Datasheets:
% PMSM: Teknic Hudson M-2310P (Typical 48V, 4-pole pair industrial servo)
% MOSFET: Texas Instruments CSD18540Q5B (60V, 1.8 mOhm, 100A+)

%% 1. Inverter and Power Supply Parameters
Vdc = 48.0;          % Nominal DC Bus Voltage (V)
C_bus = 1000e-6;     % DC Bus Capacitor (F)
Ron = 0.0018;        % MOSFET Rds(on) for TI CSD18540Q5B (Ohms)
T_dead = 1e-6;       % Inverter Deadtime (s) (1 us is standard for these drives)
f_sw = 10e3;         % Switching Frequency (Hz)

%% 2. PMSM Parameters (Teknic Hudson M-2310P approximate values)
Rs = 0.5;            % Stator Resistance per phase (Ohm)
Ld = 0.0008;         % D-axis Inductance (H) (0.8 mH)
Lq = 0.0008;         % Q-axis Inductance (H)
PsiPM = 0.02;        % Permanent Magnet Flux Linkage (Wb)
pp = 4;              % Number of Pole Pairs
J = 1e-5;
B_fric = 1e-6;       % Viscous Friction (N.m.s/rad)
Kt = 1.5 * pp * PsiPM; % Torque Constant (N.m/A)
Rs_hat = Rs;
Ld_hat = Ld;
Lq_hat = Lq;
PsiPM_hat = PsiPM;
%% 3. Control Loop Parameters
Ts_ctrl = 100e-6;    % Control Sample Time (10 kHz)
Ts_sim = 1e-6;       % Simulation Time Step (1 MHz for switching dynamics)

% Speed Control Loop (Outer Loop)
Kp_spd = 0.01;
Ki_spd = 1.5;
Iq_max = 10.0;       % Maximum Q-axis Current limit (A) (Nominal for this motor)
Id_max = 2.0;        % Maximum D-axis Current limit (A) (for testing)

% Current Control Loop (For FOC variant)
% Tuning based on L and Rs: Kp = L*omega_c, Ki = Rs*omega_c
omega_c_curr = 2*pi*1000; % 1 kHz bandwidth
Kp_curr = Ld * omega_c_curr;
Ki_curr = Rs * omega_c_curr;

% CCS-MPC Parameters
Q_id = 1.0;          % Weight for id tracking error
Q_iq = 1.0;          % Weight for iq tracking error
R_sw = 0.01;         % Weight for switching effort (penalty on voltage mag)
Vph_max = Vdc / sqrt(3); % Maximum phase voltage magnitude (SVPWM linear region limit)

disp('=== PMSM Parameters Loaded (Datasheet Values) ===');
fprintf('  Rs=%.3f Ohm, Ld=%.2f mH, Lq=%.2f mH, PsiPM=%.3f Wb\n', Rs, Ld*1000, Lq*1000, PsiPM);
fprintf('  pp=%d, J=%.2e kg.m^2, Kt=%.3f N.m/A\n', pp, J, Kt);
fprintf('  Ts_ctrl=%.0f us, Ts_sim=%.0f us\n', Ts_ctrl*1e6, Ts_sim*1e6);
fprintf('  MOSFET Ron=%.4f Ohm, Vdc=%.1f V\n', Ron, Vdc);


