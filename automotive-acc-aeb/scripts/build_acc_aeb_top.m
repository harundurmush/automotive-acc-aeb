% Build the Simulink top model programmatically
% Place this file under: scripts/build_acc_aeb_top.m

%% Resolve project paths (robust to current folder)
thisFile = mfilename('fullpath');
thisDir  = fileparts(thisFile);           % .../automotive-acc-aeb/scripts
rootDir  = fileparts(thisDir);            % .../automotive-acc-aeb
modelsDir = fullfile(rootDir, 'models');

%% Ensure params are loaded (run with absolute path)
if ~evalin('base','exist(''P'',''var'')')
    run(fullfile(thisDir,'acc_aeb_params.m'));
end
P = evalin('base','P');

%% Inject defaults if missing
if ~isfield(P,'controller') || ~isfield(P.controller,'AEB')
    P.controller.AEB.eps        = 0.1;
    P.controller.AEB.TTC_warn   = 6.0;   % ↑ easier to visualize
    P.controller.AEB.TTC_brake  = 3.0;   % ↑ easier to visualize
end
if ~isfield(P,'limits')
    P.limits.a_comfort_pos        = 2.5;
    P.limits.a_comfort_neg        = 4.0;
    P.limits.jerk_limit           = 5.0;
    P.limits.rate_throttle_up     = 0.6;
    P.limits.rate_throttle_down   = 1.2;
    P.limits.rate_brake_up        = 2.0;
    P.limits.rate_brake_down      = 3.0;
end
assignin('base','P',P);  % persist

%% Ensure folders exist
if ~exist(modelsDir, 'dir');  mkdir(modelsDir);  end

mdl = 'acc_aeb_top';
if bdIsLoaded(mdl); close_system(mdl,0); end
new_system(mdl);
open_system(mdl);

% Basic model configuration
set_param(mdl, 'StopTime', num2str(P.sim.t_end));
set_param(mdl, 'Solver', 'ode23t');
set_param(mdl, 'FixedStep', 'auto');
set_param(mdl, 'AutoInsertRateTranBlk', 'off');

% Helper for quick positioning
px = @(x,y,w,h) [x y x+w y+h];

%% Top-level subsystems
add_block('built-in/Subsystem', [mdl '/Scenarios'],     'Position', px(50,  60, 120, 90));
add_block('built-in/Subsystem', [mdl '/Sensors'],       'Position', px(260, 60, 120, 90));
add_block('built-in/Subsystem', [mdl '/ACC_Controller'],'Position', px(260,190, 180,150));
add_block('built-in/Subsystem', [mdl '/AEB_Manager'],   'Position', px(260,360, 180,170));
add_block('built-in/Subsystem', [mdl '/Plant'],         'Position', px(470,200, 180,140));
add_block('built-in/Subsystem', [mdl '/Unreal_Bridge'], 'Position', px(700,360, 260,150));

% Throttle/Brake arbitration (Max for brakes)
add_block('simulink/Math Operations/MinMax', [mdl '/MaxBrake'], ...
    'Function','max', 'Inputs','2', 'Position', px(460, 360, 50, 46));

% Scopes
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Speeds'], 'Inputs','2', 'Position', px(700, 60, 40, 40));
add_block('simulink/Sinks/Scope',        [mdl '/Scope_Speeds'], 'Position', px(760, 50, 110, 90));
add_block('simulink/Sinks/Scope',        [mdl '/Scope_RelDistance'], 'Position', px(760, 160, 130, 90));
add_block('simulink/Sinks/Scope',        [mdl '/Scope_AEB'], 'Position', px(760, 280, 130, 90));
add_block('simulink/Sinks/Scope',        [mdl '/Scope_TTC'], 'Position', px(760, 410, 170, 110));   % debug

%% --- Scenarios ---
blk = [mdl '/Scenarios']; open_system(blk);
add_block('simulink/Sources/From Workspace', [blk '/FromLeadSpeed'], ...
    'VariableName','lead_speed_ts', 'Position', px(40, 40, 140, 40));
add_block('simulink/Sources/From Workspace', [blk '/FromDriverSet'], ...
    'VariableName','driver_set_speed_ts', 'Position', px(40, 110, 140, 40));
add_block('simulink/Continuous/Integrator', [blk '/Int_LeadPos'], ...
    'InitialCondition', 'P.scenario.lead_x0', 'Position', px(220, 40, 40, 40));
add_block('simulink/Sinks/Out1', [blk '/LeadSpeed'],      'Position', px(200, 100, 30, 14));
add_block('simulink/Sinks/Out1', [blk '/LeadPosition'],   'Position', px(300, 50, 30, 14));
add_block('simulink/Sinks/Out1', [blk '/DriverSetSpeed'], 'Position', px(200, 160, 30, 14));
add_line(blk, 'FromLeadSpeed/1', 'Int_LeadPos/1');
add_line(blk, 'FromLeadSpeed/1', 'LeadSpeed/1');
add_line(blk, 'Int_LeadPos/1',   'LeadPosition/1');
add_line(blk, 'FromDriverSet/1', 'DriverSetSpeed/1');
close_system(blk);

%% --- Sensors (pass-through relatives) ---
blk = [mdl '/Sensors']; open_system(blk);
add_block('simulink/Sources/In1', [blk '/LeadSpeed'],    'Position', px(20, 30, 30, 14));
add_block('simulink/Sources/In1', [blk '/LeadPosition'], 'Position', px(20, 70, 30, 14));
add_block('simulink/Sources/In1', [blk '/EgoSpeed'],     'Position', px(20,110, 30, 14));
add_block('simulink/Sources/In1', [blk '/EgoPosition'],  'Position', px(20,150, 30, 14));
add_block('simulink/Math Operations/Add', [blk '/Add_RelSpeed'], 'Inputs','+-', 'Position', px(120, 40, 30, 30));
add_block('simulink/Math Operations/Add', [blk '/Add_RelDist'],  'Inputs','+-', 'Position', px(120,120, 30, 30));
add_block('simulink/Sinks/Out1', [blk '/RelSpeed'],    'Position', px(200, 40, 30, 14));
add_block('simulink/Sinks/Out1', [blk '/RelDistance'], 'Position', px(200,130, 30, 14));
add_line(blk, 'LeadSpeed/1',    'Add_RelSpeed/1');
add_line(blk, 'EgoSpeed/1',     'Add_RelSpeed/2');
add_line(blk, 'LeadPosition/1', 'Add_RelDist/1');
add_line(blk, 'EgoPosition/1',  'Add_RelDist/2');
add_line(blk, 'Add_RelSpeed/1', 'RelSpeed/1');
add_line(blk, 'Add_RelDist/1',  'RelDistance/1');
close_system(blk);

%% --- ACC_Controller ---
blk = [mdl '/ACC_Controller']; open_system(blk);
add_block('simulink/Sources/In1', [blk '/DriverSetSpeed'], 'Position', px(20,40,30,14));
add_block('simulink/Sources/In1', [blk '/EgoSpeed'],       'Position', px(20,80,30,14));
add_block('simulink/Sources/In1', [blk '/RelDistance'],    'Position', px(20,120,30,14));

add_block('simulink/Sources/Constant', [blk '/Const_d0'], 'Value','P.controller.minDistance', 'Position', px(80, 140, 60, 24));
add_block('simulink/Math Operations/Add', [blk '/Add_RelMinusD0'], 'Inputs','+-', 'Position', px(160, 120, 30, 30));
invTau = 1/max(P.controller.timeGap, 1e-3); assignin('base','invTau_ACC',invTau);
add_block('simulink/Math Operations/Gain', [blk '/Gain_InvTau'], 'Gain','invTau_ACC', 'Position', px(210, 120, 60, 30));
add_block('simulink/Discontinuities/Saturation', [blk '/Sat_vgap'], 'UpperLimit','inf','LowerLimit','0', 'Position', px(280, 120, 60, 30));
add_block('simulink/Math Operations/MinMax', [blk '/Min_vref'], 'Function','min','Inputs','2', 'Position', px(360, 95, 50, 40));
add_block('simulink/Math Operations/Add', [blk '/Add_SpeedErr'], 'Inputs','+-', 'Position', px(430,100,30,30));
add_block('simulink/Continuous/PID Controller', [blk '/PI_Speed'], ...
    'P','P.controller.PI_speed.Kp','I','P.controller.PI_speed.Ki','D','0', ...
    'InitialConditionForIntegrator','0','LimitOutput','on', ...
    'UpperSaturationLimit','P.controller.PI_speed.u_max', ...
    'LowerSaturationLimit','P.controller.PI_speed.u_min', ...
    'Position', px(480,95,80,40));

add_block('simulink/Signal Routing/Switch', [blk '/Switch_Throttle'], ...
    'Criteria','u2 >= Threshold','Threshold','0','Position', px(580,75,50,40));
add_block('simulink/Math Operations/Gain', [blk '/Negate'], 'Gain','-1', 'Position', px(580,140,40,30));
add_block('simulink/Discontinuities/Saturation', [blk '/Sat_Throttle'], ...
    'UpperLimit','1','LowerLimit','0', 'Position', px(650,75,50,40));

% *** IMPORTANT: Cap ACC brake authority so AEB can add more braking ***
add_block('simulink/Discontinuities/Saturation', [blk '/Sat_Brake'], ...
    'UpperLimit','0.3','LowerLimit','0', 'Position', px(650,140,50,40));

add_block('simulink/Discontinuities/Rate Limiter', [blk '/RL_Throttle'], ...
    'RisingSlewLimit','P.limits.rate_throttle_up', 'FallingSlewLimit','-P.limits.rate_throttle_down', ...
    'InitialCondition','0','Position', px(710,75,50,40));
add_block('simulink/Discontinuities/Rate Limiter', [blk '/RL_Brake'], ...
    'RisingSlewLimit','P.limits.rate_brake_up', 'FallingSlewLimit','-P.limits.rate_brake_down', ...
    'InitialCondition','0','Position', px(710,140,50,40));

add_block('simulink/Sinks/Out1', [blk '/ThrottleCmd_ACC'], 'Position', px(780, 85, 30, 14));
add_block('simulink/Sinks/Out1', [blk '/BrakeCmd_ACC'],    'Position', px(780,155, 30, 14));

add_line(blk, 'RelDistance/1','Add_RelMinusD0/1');
add_line(blk, 'Const_d0/1',   'Add_RelMinusD0/2');
add_line(blk, 'Add_RelMinusD0/1','Gain_InvTau/1');
add_line(blk, 'Gain_InvTau/1','Sat_vgap/1');
add_line(blk, 'DriverSetSpeed/1','Min_vref/1');
add_line(blk, 'Sat_vgap/1',      'Min_vref/2');
add_line(blk, 'Min_vref/1','Add_SpeedErr/1');
add_line(blk, 'EgoSpeed/1','Add_SpeedErr/2');
add_line(blk, 'Add_SpeedErr/1','PI_Speed/1');
add_line(blk, 'PI_Speed/1','Switch_Throttle/1');
add_line(blk, 'PI_Speed/1','Switch_Throttle/2');
add_line(blk, 'PI_Speed/1','Negate/1');
add_line(blk, 'Switch_Throttle/1','Sat_Throttle/1');
add_line(blk, 'Sat_Throttle/1','RL_Throttle/1');
add_line(blk, 'RL_Throttle/1','ThrottleCmd_ACC/1');
add_line(blk, 'Negate/1','Sat_Brake/1');
add_line(blk, 'Sat_Brake/1','RL_Brake/1');
add_line(blk, 'RL_Brake/1','BrakeCmd_ACC/1');
close_system(blk);

%% --- AEB_Manager (TTC-based) ---
blk = [mdl '/AEB_Manager']; open_system(blk);
add_block('simulink/Sources/In1', [blk '/RelDistance'], 'Position', px(20,30,30,14));
add_block('simulink/Sources/In1', [blk '/RelSpeed'],    'Position', px(20,70,30,14));
add_block('simulink/Sources/Constant', [blk '/Const_eps'],      'Value','P.controller.AEB.eps',       'Position', px(100,20,60,24));
add_block('simulink/Sources/Constant', [blk '/Const_TTCwarn'],  'Value','P.controller.AEB.TTC_warn',  'Position', px(100,50,60,24));
add_block('simulink/Sources/Constant', [blk '/Const_TTCbrake'], 'Value','P.controller.AEB.TTC_brake', 'Position', px(100,80,60,24));
add_block('simulink/Sources/Constant', [blk '/Const_Zero'],     'Value','0', 'Position', px(100,160,50,24));
add_block('simulink/Math Operations/Gain', [blk '/NegRelSpeed'], 'Gain','-1', 'Position', px(180, 65, 40, 24));
add_block('simulink/Math Operations/MinMax', [blk '/Max_Close'], 'Function','max','Inputs','2', 'Position', px(230, 60, 40, 34));
add_block('simulink/Math Operations/Divide', [blk '/Div_TTC'], 'Position', px(290, 60, 40, 34));
add_block('simulink/Math Operations/Add', [blk '/Add_Twarn_minus_TTC'], 'Inputs','+-', 'Position', px(350, 60, 40, 34));
diffWarnBrake = max(P.controller.AEB.TTC_warn - P.controller.AEB.TTC_brake, 1e-3);
assignin('base','invDiffWarnBrake', 1/diffWarnBrake);
add_block('simulink/Math Operations/Gain', [blk '/Gain_InvDiff'], 'Gain','invDiffWarnBrake', 'Position', px(400, 60, 40, 34));
add_block('simulink/Discontinuities/Saturation', [blk '/Sat_Scale01'], 'UpperLimit','1','LowerLimit','0', 'Position', px(450, 60, 40, 34));
add_block('simulink/Signal Routing/Switch', [blk '/Switch_Opening'], ...
    'Criteria','u2 >= Threshold', 'Threshold','0', 'Position', px(500, 50, 50, 40));
add_block('simulink/Sinks/Out1', [blk '/BrakeCmd_AEB'],   'Position', px(570,60,30,14));
% Debug outs
add_block('simulink/Sinks/Out1', [blk '/dbg_ClosingSpeed'],'Position', px(570,90,30,14));
add_block('simulink/Sinks/Out1', [blk '/dbg_TTC'],         'Position', px(570,120,30,14));
add_block('simulink/Sinks/Out1', [blk '/dbg_Scale'],       'Position', px(570,150,30,14));

add_line(blk, 'RelSpeed/1',     'NegRelSpeed/1');
add_line(blk, 'NegRelSpeed/1',  'Max_Close/1');
add_line(blk, 'Const_eps/1',    'Max_Close/2');
add_line(blk, 'RelDistance/1',  'Div_TTC/1');
add_line(blk, 'Max_Close/1',    'Div_TTC/2');
add_line(blk, 'Const_TTCwarn/1','Add_Twarn_minus_TTC/1');
add_line(blk, 'Div_TTC/1',      'Add_Twarn_minus_TTC/2');
add_line(blk, 'Add_Twarn_minus_TTC/1','Gain_InvDiff/1');
add_line(blk, 'Gain_InvDiff/1',       'Sat_Scale01/1');
add_line(blk, 'Sat_Scale01/1',   'Switch_Opening/3');
add_line(blk, 'RelSpeed/1',      'Switch_Opening/2');
add_line(blk, 'Const_Zero/1',    'Switch_Opening/1');
add_line(blk, 'Switch_Opening/1','BrakeCmd_AEB/1');
add_line(blk, 'Max_Close/1','dbg_ClosingSpeed/1');
add_line(blk, 'Div_TTC/1','dbg_TTC/1');
add_line(blk, 'Sat_Scale01/1','dbg_Scale/1');
close_system(blk);

%% --- Plant (nonnegative speed clamp + jerk/comfort limits) ---
blk = [mdl '/Plant']; open_system(blk);
add_block('simulink/Sources/In1', [blk '/ThrottleCmd'], 'Position', px(20,40,30,14));
add_block('simulink/Sources/In1', [blk '/BrakeCmd'],    'Position', px(20,90,30,14));

% First-order actuator lags
add_block('simulink/Continuous/Transfer Fcn', [blk '/TF_Throttle'], ...
    'Numerator','1', 'Denominator','[P.actuator.tau_throttle 1]', 'Position', px(90, 35, 70, 30));
add_block('simulink/Continuous/Transfer Fcn', [blk '/TF_Brake'], ...
    'Numerator','1', 'Denominator','[P.actuator.tau_brake 1]', 'Position', px(90, 85, 70, 30));

% Command → accelerations
add_block('simulink/Math Operations/Gain', [blk '/Gain_a_trac'],  'Gain','P.actuator.a_max',       'Position', px(180, 35, 60, 30));
add_block('simulink/Math Operations/Gain', [blk '/Gain_a_brake'], 'Gain','P.actuator.a_brake_max', 'Position', px(180, 85, 60, 30));

% Aerodynamic + rolling resistance
k_drag = 0.5 * P.vehicle.rho * P.vehicle.Cd * P.vehicle.A / P.vehicle.m; assignin('base','k_drag',k_drag);
add_block('simulink/Sources/Constant', [blk '/Const_gCr'], 'Value','P.vehicle.g*P.vehicle.Cr', 'Position', px(180, 140, 60, 30));

% Net accel: a_trac - a_brake - a_resist
add_block('simulink/Math Operations/Product', [blk '/Prod_v2'], 'Inputs','**', 'Position', px(260, 180, 40, 40));
add_block('simulink/Math Operations/Gain',    [blk '/Gain_kdrag'], 'Gain','k_drag', 'Position', px(320, 185, 60, 30));
add_block('simulink/Math Operations/Add',     [blk '/Add_Resist'], 'Inputs','++', 'Position', px(400, 165, 40, 40));
add_block('simulink/Math Operations/Add',     [blk '/Add_a_net'], 'Inputs','+--', 'Position', px(270, 70, 50, 50));

% Jerk & comfort limits
add_block('simulink/Discontinuities/Rate Limiter', [blk '/RL_Jerk'], ...
    'RisingSlewLimit','P.limits.jerk_limit', 'FallingSlewLimit','-P.limits.jerk_limit', ...
    'InitialCondition','0', 'Position', px(330, 70, 50, 40));
add_block('simulink/Discontinuities/Saturation', [blk '/Sat_a_comfort'], ...
    'UpperLimit','P.limits.a_comfort_pos', 'LowerLimit','-P.limits.a_comfort_neg', ...
    'Position', px(395, 70, 60, 40));

% Integrators
add_block('simulink/Continuous/Integrator', [blk '/Int_Speed'], ...
    'InitialCondition','P.scenario.ego_v0', 'Position', px(470, 60, 40, 40));

% *** NEW: clamp speed to be nonnegative, and use clamped speed everywhere ***
add_block('simulink/Discontinuities/Saturation', [blk '/Sat_SpeedNonNeg'], ...
    'LowerLimit','0','UpperLimit','inf','Position', px(520, 60, 60, 40));

add_block('simulink/Continuous/Integrator', [blk '/Int_Position'], ...
    'InitialCondition','P.scenario.ego_x0', 'Position', px(600, 60, 40, 40));

% Outports
add_block('simulink/Sinks/Out1', [blk '/EgoAccel'],   'Position', px(680, 40, 30, 14));
add_block('simulink/Sinks/Out1', [blk '/EgoSpeed'],   'Position', px(680, 70, 30, 14));
add_block('simulink/Sinks/Out1', [blk '/EgoPosition'],'Position', px(680,100, 30, 14));

% Wiring
add_line(blk, 'ThrottleCmd/1','TF_Throttle/1');
add_line(blk, 'BrakeCmd/1',   'TF_Brake/1');
add_line(blk, 'TF_Throttle/1','Gain_a_trac/1');
add_line(blk, 'TF_Brake/1',   'Gain_a_brake/1');

add_line(blk, 'Gain_a_trac/1',  'Add_a_net/1'); % +
add_line(blk, 'Gain_a_brake/1', 'Add_a_net/2'); % -

% a_resist from *clamped* speed
add_line(blk, 'Sat_SpeedNonNeg/1', 'Prod_v2/1');
add_line(blk, 'Sat_SpeedNonNeg/1', 'Prod_v2/2');
add_line(blk, 'Prod_v2/1',   'Gain_kdrag/1');
add_line(blk, 'Gain_kdrag/1','Add_Resist/1');
add_line(blk, 'Const_gCr/1', 'Add_Resist/2');
add_line(blk, 'Add_Resist/1','Add_a_net/3'); % -

% a -> jerk limiter -> comfort -> integrate v
add_line(blk, 'Add_a_net/1',    'RL_Jerk/1');
add_line(blk, 'RL_Jerk/1',      'Sat_a_comfort/1');
add_line(blk, 'Sat_a_comfort/1','Int_Speed/1');

% *** feed clamped speed everywhere ***
add_line(blk, 'Int_Speed/1',      'Sat_SpeedNonNeg/1');
add_line(blk, 'Sat_SpeedNonNeg/1','Int_Position/1');
add_line(blk, 'Sat_SpeedNonNeg/1','EgoSpeed/1');

% outputs
add_line(blk, 'Sat_a_comfort/1','EgoAccel/1');
add_line(blk, 'Int_Position/1','EgoPosition/1');
close_system(blk);


%% --- Unreal_Bridge (vector + scalar X outputs) ---
blk = [mdl '/Unreal_Bridge']; open_system(blk);
add_block('simulink/Sources/In1', [blk '/EgoX'],  'Position', px(20, 40, 30, 14));
add_block('simulink/Sources/In1', [blk '/LeadX'], 'Position', px(20, 90, 30, 14));
add_block('simulink/Sources/Constant', [blk '/Zero'],  'Value','0',       'Position', px(80, 20, 40, 24));
add_block('simulink/Sources/Constant', [blk '/Zero3'], 'Value','[0;0;0]', 'Position', px(80,130, 60, 24));
add_block('simulink/Signal Routing/Mux', [blk '/Mux_EgoTrans'],  'Inputs','3', 'Position', px(150, 35, 40, 30));
add_block('simulink/Signal Routing/Mux', [blk '/Mux_LeadTrans'], 'Inputs','3', 'Position', px(150, 85, 40, 30));
add_block('simulink/Sinks/Out1', [blk '/EgoTranslation'],  'Position', px(210, 35, 30, 14));
add_block('simulink/Sinks/Out1', [blk '/EgoRotation'],     'Position', px(210, 65, 30, 14));
add_block('simulink/Sinks/Out1', [blk '/LeadTranslation'], 'Position', px(210, 95, 30, 14));
add_block('simulink/Sinks/Out1', [blk '/LeadRotation'],    'Position', px(210,125, 30, 14));
add_block('simulink/Sinks/Out1', [blk '/EgoX_scalar'],     'Position', px(260, 35, 40, 14));
add_block('simulink/Sinks/Out1', [blk '/LeadX_scalar'],    'Position', px(260, 95, 40, 14));
add_line(blk, 'EgoX/1','Mux_EgoTrans/1');  add_line(blk, 'Zero/1','Mux_EgoTrans/2');  add_line(blk, 'Zero/1','Mux_EgoTrans/3');
add_line(blk, 'LeadX/1','Mux_LeadTrans/1');add_line(blk, 'Zero/1','Mux_LeadTrans/2');add_line(blk, 'Zero/1','Mux_LeadTrans/3');
add_line(blk, 'Mux_EgoTrans/1','EgoTranslation/1'); add_line(blk, 'Mux_LeadTrans/1','LeadTranslation/1');
add_line(blk, 'Zero3/1','EgoRotation/1');  add_line(blk, 'Zero3/1','LeadRotation/1');
add_line(blk, 'EgoX/1','EgoX_scalar/1');   add_line(blk, 'LeadX/1','LeadX_scalar/1');
close_system(blk);

%% --- Top-level wiring ---
% Scenarios -> Sensors
add_line(mdl, 'Scenarios/1', 'Sensors/1');  % LeadSpeed
add_line(mdl, 'Scenarios/2', 'Sensors/2');  % LeadPosition
% Plant -> Sensors
add_line(mdl, 'Plant/2', 'Sensors/3');      % EgoSpeed
add_line(mdl, 'Plant/3', 'Sensors/4');      % EgoPosition

% ACC inputs
add_line(mdl, 'Scenarios/3', 'ACC_Controller/1'); % DriverSetSpeed
add_line(mdl, 'Plant/2',     'ACC_Controller/2'); % EgoSpeed

% Top-level relatives (authoritative)
add_block('simulink/Math Operations/Add', [mdl '/RelSpeed_TOP'], 'Inputs','+-', 'Position', px(590, 120, 40, 30));
add_block('simulink/Math Operations/Add', [mdl '/RelDist_TOP'],  'Inputs','+-', 'Position', px(590, 170, 40, 30));
add_line(mdl, 'Scenarios/1','RelSpeed_TOP/1');  % LeadSpeed
add_line(mdl, 'Plant/2',    'RelSpeed_TOP/2');  % EgoSpeed
add_line(mdl, 'Scenarios/2','RelDist_TOP/1');   % LeadPos
add_line(mdl, 'Plant/3',    'RelDist_TOP/2');   % EgoPos

% Feed controllers from TOP-LEVEL relatives
add_line(mdl, 'RelDist_TOP/1', 'ACC_Controller/3');  % RelDistance -> ACC
add_line(mdl, 'RelDist_TOP/1', 'AEB_Manager/1');     % RelDistance -> AEB
add_line(mdl, 'RelSpeed_TOP/1','AEB_Manager/2');     % RelSpeed    -> AEB

% ACC/AEB -> Plant via MaxBrake
add_line(mdl, 'ACC_Controller/2', 'MaxBrake/1');     % BrakeCmd_ACC
add_line(mdl, 'AEB_Manager/1',    'MaxBrake/2');     % BrakeCmd_AEB
add_line(mdl, 'ACC_Controller/1', 'Plant/1');        % ThrottleCmd
add_line(mdl, 'MaxBrake/1',       'Plant/2');        % BrakeCmd

% Scopes
add_line(mdl, 'Plant/2',    'Mux_Speeds/1');         % EgoSpeed
add_line(mdl, 'Scenarios/1','Mux_Speeds/2');         % LeadSpeed
add_line(mdl, 'Mux_Speeds/1','Scope_Speeds/1');
add_line(mdl, 'RelDist_TOP/1','Scope_RelDistance/1');
add_line(mdl, 'AEB_Manager/1','Scope_AEB/1');

% TTC debug scope
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_TTC'], 'Inputs','3', 'Position', px(700, 410, 50, 50));
add_line(mdl, 'AEB_Manager/2','Mux_TTC/1');  % dbg_ClosingSpeed
add_line(mdl, 'AEB_Manager/3','Mux_TTC/2');  % dbg_TTC
add_line(mdl, 'AEB_Manager/4','Mux_TTC/3');  % dbg_Scale
add_line(mdl, 'Mux_TTC/1','Scope_TTC/1');

% Unreal_Bridge inputs
add_line(mdl, 'Plant/3',     'Unreal_Bridge/1');     % EgoPosition -> EgoX
add_line(mdl, 'Scenarios/2', 'Unreal_Bridge/2');     % LeadPosition -> LeadX

% Open scopes at start
set_param([mdl '/Scope_Speeds'],     'OpenAtSimulationStart','on');
set_param([mdl '/Scope_RelDistance'],'OpenAtSimulationStart','on');
set_param([mdl '/Scope_AEB'],        'OpenAtSimulationStart','on');
set_param([mdl '/Scope_TTC'],        'OpenAtSimulationStart','on');

% Save model
save_system(mdl, fullfile(modelsDir,[mdl '.slx']));
fprintf('Model built and saved to %s\n', fullfile(modelsDir,[mdl '.slx']));
