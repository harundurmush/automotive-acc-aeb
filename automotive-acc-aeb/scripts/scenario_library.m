function [lead_v, driver_v, P] = scenario_library(name, t, P)
%SCENARIO_LIBRARY Build lead vehicle speed and driver set speed profiles.
%   name : 'lead_hard_brake' | 'follow_slower_lead' | 'speed_hold'
%   t    : time vector (s)
%   P    : parameter struct (will update P.scenario.*)

% Defaults (if not set)
if ~isfield(P,'controller');             P.controller = struct(); end
if ~isfield(P.controller,'timeGap');     P.controller.timeGap = 1.4; end
if ~isfield(P.controller,'minDistance'); P.controller.minDistance = 5; end

switch lower(string(name))
    case "lead_hard_brake"
        % Ego starts at v0, Lead ahead then brakes hard at t0 with decel a_brake
        v0     = 25;      % m/s  (~90 km/h)
        t0     = 6.0;     % s
        a_br   = 6.0;     % m/s^2 (lead harsh brake)
        lead_v = v0 - a_br*max(0, t - t0);
        lead_v = max(lead_v, 0);

        driver_v = v0 * ones(size(t));

        % Initial positions/speeds
        P.scenario.ego_x0  = 0;
        P.scenario.ego_v0  = v0;
        P.scenario.lead_x0 = 35;   % 35 m headway
        P.scenario.lead_v0 = v0;

    case "follow_slower_lead"
        v_lead   = 18;     % m/s (~65 km/h)
        v_set    = 25;     % m/s (~90 km/h)
        lead_v   = v_lead * ones(size(t));
        driver_v = v_set  * ones(size(t));

        P.scenario.ego_x0  = 0;
        P.scenario.ego_v0  = v_set;
        P.scenario.lead_x0 = 30;   % initial headway
        P.scenario.lead_v0 = v_lead;

    case "speed_hold"
        % No meaningful lead influence; put lead far ahead and/or stopped
        v_set    = 25; 
        lead_v   = zeros(size(t)); % integrate to constant pos (far ahead)
        driver_v = v_set * ones(size(t));

        P.scenario.ego_x0  = 0;
        P.scenario.ego_v0  = 0;
        P.scenario.lead_x0 = 500;  % very far ahead
        P.scenario.lead_v0 = 0;

    otherwise
        error('Unknown scenario name: %s', name);
end
end
