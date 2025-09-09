% scripts/configure_unreal_blocks.m
% Configure Simulation 3D blocks (if present) to match P and good execution order.

mdl = 'acc_aeb_top';
if ~bdIsLoaded(mdl); open_system(fullfile('models',[mdl '.slx'])); end
P = evalin('base','P');

% Find blocks by MaskType (robust to user-given names)
sceneBlk = find_system(mdl,'MatchFilter',@Simulink.match.internal.filterOutInactiveVariantSubsystemChoices, ...
                            'MaskType','Simulation 3D Scene Configuration');
vehBlks  = find_system(mdl,'MatchFilter',@Simulink.match.internal.filterOutInactiveVariantSubsystemChoices, ...
                            'MaskType','Simulation 3D Vehicle with Ground Following');

% Configure Scene (optional)
for i = 1:numel(sceneBlk)
    try
        set_param(sceneBlk{i}, 'SampleTime', num2str(P.sim.dt)); % e.g., '0.02'
        set_param(sceneBlk{i}, 'Priority', '0');
    catch ME
        warning('Scene block config skipped: %s', ME.message);
    end
end

% Configure Vehicles
for i = 1:numel(vehBlks)
    blk = vehBlks{i};
    % Guess Ego/Lead by name occurrence if possible
    isEgo  = contains(get_param(blk,'Name'),'Ego','IgnoreCase',true);
    isLead = contains(get_param(blk,'Name'),'Lead','IgnoreCase',true);

    % Initial positions
    if isEgo
        initPos = [P.scenario.ego_x0  0 0];
    elseif isLead
        initPos = [P.scenario.lead_x0 0 0];
    else
        % Fallback: put unknown vehicles at origin line
        initPos = [0 0 0];
    end

    try
        set_param(blk,'SampleTime','-1');  % inherit from Scene
    catch, end
    try
        set_param(blk,'InitialPosition', mat2str(initPos,6));
    catch, end
    try
        set_param(blk,'InitialRotation','[0 0 0]');
    catch, end
    try
        set_param(blk,'Priority','-1');
    catch, end
end

disp('✅ Unreal blocks configured (sample times, priorities, initial poses).');
