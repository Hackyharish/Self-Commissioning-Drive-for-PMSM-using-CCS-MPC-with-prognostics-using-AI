% params_pmsm_inverter.m
% Parameters for PMSM Drive Simulation
% BMW i3 Traction Motor (IPMSM) validation

%% 1. Inverter and Power Supply Parameters
Vdc = 400;         % Nominal DC Bus Voltage (V) - derived from 159.2 V fundamental peak phase voltage
C_bus = 2200e-6;     % DC Bus Capacitor (F)
Ron = 0.001;         % MOSFET/IGBT Rds(on) (Ohms)
T_dead = 2e-6;       % Inverter Deadtime (s)
f_sw = 10e3;         % Switching Frequency (Hz)

%% 2. PMSM Parameters (BMW i3 2013-2022)
% Known ground truth values for the plant model:
Rs = 0.0053;         % Stator Resistance per phase (Ohm)
Ld = 71.2e-6;        % D-axis Inductance (H)
Lq = 141.3e-6;       % Q-axis Inductance (H)
PsiPM = 0.0436;      % Permanent Magnet Flux Linkage (Wb)
pp = 6;              % Number of Pole Pairs

% Mechanical parameters
J = 0.0867;          % Rotor Inertia (kg.m^2) (approximated from cylinder: 14.2 kg, 178.6mm/130.3mm)
J_load = 0.5;        % Load Inertia (kg.m^2)
B_fric = 0.001;      % Viscous Friction (N.m.s/rad) (assumed)

Kt = 1.5 * pp * PsiPM; % Magnet torque constant

% Estimated parameters (initial guesses before auto-tuning)
% We initialize with bad values to ensure the auto-tuner estimates them!
Rs_hat = 0.5;
Ld_hat = 0.001;
Lq_hat = 0.001;
PsiPM_hat = 0.0;

%% 3. Motor Ratings (for reference and limit calculations)
P_rated = 125e3;     % Rated Power (W) (approximate)
T_rated = 258.2;     % Peak Torque (Nm) - from nonlinear FEM model
n_rated = 4000;      % Rated Speed (rpm)
n_max = 11400;       % Maximum Speed (rpm)
I_peak = 565.7;      % Peak Phase Current (A) - Max current rating
I_cont = 300;        % Continuous Phase Current (A)

%% 4. Control Loop Parameters
Ts_ctrl = 100e-6;    % Control Sample Time (10 kHz)
Ts_sim = 5e-6;       % Simulation Time Step (200 kHz)

Kp_spd = 2.0;       
Ki_spd = 10.0;      

Iq_max = 565.7;      % Maximum Q-axis Current limit (A)
Id_max = 565.7;      % Maximum D-axis Current limit (A)

omega_c_curr = 2*pi*1000;   % 1 kHz current loop bandwidth
Kp_curr = Ld * omega_c_curr;
Ki_curr = Rs * omega_c_curr;

Q_id = 1.0;          % Weight for id tracking error
Q_iq = 1.0;          % Weight for iq tracking error
R_sw = 0.001;        % Weight for switching effort
Vph_max = 159.2;     % Maximum fundamental phase voltage

%% 5. MTPA Parameters
MTPA_enable = true;  

%% 6. Field Weakening Parameters
omega_base_e = Vph_max / PsiPM;  
omega_base_m = omega_base_e / pp;  
n_base = omega_base_m * 60 / (2*pi);  

disp('=== PMSM Parameters Loaded (BMW i3 IPMSM) ===');
