% run_speed_mpc_sil.m
try
    disp('Loading Speed_MPC Harness...');
    load_system('PMSM_CascadedMPC_Working');
    sltest.harness.load('PMSM_CascadedMPC_Working/CCSMPC_Controller/Speed_MPC', 'Harness_Speed_MPC');
    
    Ts = 1e-4; % Simulation step
    t_end = 3.0;
    
    disp('Generating SIL block for Speed_MPC...');
    
    % Force SIL generation on the atomic subsystem
    set_param('Harness_Speed_MPC/Speed_MPC', 'TreatAsAtomicUnit', 'on');
    
    % slbuild requires the model to be configured for code generation
    % Build SIL block
    % slbuild('Harness_Speed_MPC/Speed_MPC', 'SIL') 
    % Note: Using slbuild on a subsystem requires specific settings or Simulink Coder.
    % An alternative is to use rtwbuild or set the system target file.
    % Actually, since we're in a Harness, we can configure the harness for code generation
    % and run the simulation in normal mode but with the block replaced.
    % For automation, we will just use code generation directly.
    
    % Let's use slbuild
    slbuild('Harness_Speed_MPC/Speed_MPC', 'SIL');
    
    disp('SIL block generated.');
    
    sltest.harness.close('PMSM_CascadedMPC_Working/CCSMPC_Controller/Speed_MPC', 'Harness_Speed_MPC');
    close_system('PMSM_CascadedMPC_Working', 0);
catch ME
    disp(['Error: ', ME.message]);
end
