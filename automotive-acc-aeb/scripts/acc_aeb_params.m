% ACC + AEB Project Parameters (demo where AEB must trigger)
% Place under: scripts/acc_aeb_params.m

%% Simulation (match Unreal / sample time)
P.sim.dt    = 1/50;      % 0.02 s
P.sim.t_end = 40;
t = (0:P.sim.dt:P.sim.t_end).';

%% Vehicle / Environment
P.vehicle.m   = 1500;
P.vehicle.Cd  = 0.29;
P.vehicle.A   = 2.2;
P.vehicle.rho = 1.225;
P.vehicle.Cr  = 0.012;
P.vehicle.g   = 9.81;

%% Actuators
P.actuator.tau_throttle  = 0.30;
P.actuator.tau_brake     = 0.20;
P.actuator.a_max         = 2.5;
P.actuator.a_brake_max   = 6.0;

%% ACC policy
P.controller.timeGap     = 1.6;
P.controller.minDistance = 2.0;

% PI for speed
P.controller.PI_speed.Kp    = 0.40;
P.controller.PI_speed.Ki    = 0.20;
P.controller.PI_speed.u_min = -1.0;
P.controller.PI_speed.u_max =  1.0;

%% AEB thresholds (raised for easier demo)
P.controller.AEB.eps       = 0.1;   % m/s
P.controller.AEB.TTC_warn  = 6.0;   % s  (warning starts here)
P.controller.AEB.TTC_brake = 3.0;   % s  (hard brake region)

%% Scenario (force a near-collision)
P.scenario.ego_v0     = 20/3.6;   % ~5.56 m/s
P.scenario.ego_x0     = 0.0;
P.scenario.initialGap = 15.0;     % smaller gap → tougher case

% Lead vehicle speed:
% 0–6 s: 20 km/h
% 6–12 s: ~0 km/h (actually 0.5 km/h to avoid exact zero)
% >=12 s: 15 km/h
v_lead = (20/3.6)*ones(size(t));
v_lead(t >= 6  ) = 0.5/3.6;
v_lead(t >= 12 ) = 15/3.6;

% Driver set speed (ego target): 25 km/h
v_set = (25/3.6)*ones(size(t));

% Export as timeseries
lead_speed_ts       = timeseries(v_lead, t);
driver_set_speed_ts = timeseries(v_set,   t);

% Lead initial absolute position (used by Scenarios integrator)
P.scenario.lead_x0 = P.scenario.ego_x0 + P.scenario.initialGap;

% Push to base workspace
assignin('base','P',P);
assignin('base','lead_speed_ts',       lead_speed_ts);
assignin('base','driver_set_speed_ts', driver_set_speed_ts);
