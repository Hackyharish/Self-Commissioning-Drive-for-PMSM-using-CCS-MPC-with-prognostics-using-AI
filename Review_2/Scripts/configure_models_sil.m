try
    mdls = {'Standalone_AutoTuner', 'Standalone_Speed_MPC', 'Standalone_CCSMPC_Controller'};
    for m = 1:length(mdls)
        mdl = mdls{m};
        load_system(mdl);
        
        set_param(mdl, 'ProdEqTarget', 'on');
        set_param(mdl, 'PortableWordSizes', 'off');
        set_param(mdl, 'SystemTargetFile', 'ert.tlc');
        set_param(mdl, 'GenerateReport', 'on');
        set_param(mdl, 'LaunchReport', 'off');
        set_param(mdl, 'SaveOutput', 'on');
        set_param(mdl, 'SaveTime', 'on');
        set_param(mdl, 'SolverType', 'Fixed-step');
        set_param(mdl, 'Solver', 'FixedStepDiscrete');
        set_param(mdl, 'FixedStep', '1e-4');
        
        save_system(mdl);
        close_system(mdl, 0);
        disp(['Successfully configured ', mdl]);
    end
catch ME
    disp(ME.getReport());
end
exit;
