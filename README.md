# 🚗 ACC + AEB Simulation (Simulink + Unreal)

A compact **Adaptive Cruise Control (ACC)** and **Autonomous Emergency Braking (AEB)** demo built entirely by script.  
It programmatically creates a Simulink model, runs closed-loop longitudinal dynamics, and (optionally) streams ego/lead vehicle poses to the **Simulation 3D** environment (Unreal).

---

## ✨ What’s Inside

- **Programmatic model build**: one script constructs the whole Simulink model from scratch.
- **ACC (time-gap policy)**: PI speed loop with `v_ref = min(v_set, (RelDistance-d0)/tau)`.
- **AEB (TTC-based)**: computes closing speed, Time-to-Collision, and blends a 0–1 brake command.
- **Actuators & Plant**: first-order throttle/brake lags, aerodynamic/rolling resistance, jerk & comfort limits.
- **Scenario engine**: lead speed & driver set speed from workspace time series.
- **Scopes & debug**: speeds, relative distance, AEB command, and TTC internals.
- **Unreal bridge (optional)**: feeds ego/lead X-positions to *Simulation 3D Vehicle with Ground Following*.

---

## 🧩 Files

- `scripts/acc_aeb_params.m` — Defines `P` (all params), builds time series for scenarios, pushes to base workspace.
- `scripts/build_acc_aeb_top.m` — Builds the full Simulink model programmatically and wires everything.
- `scripts/configure_unreal_blocks.m` — (Optional) Auto-configures Simulation 3D blocks (sample time, initial poses, priorities).
- `scripts/scenario_library.m` — (Optional helper) Prebaked scenario profiles (hard brake / slower lead / speed hold).

---

## 📦 Requirements

- MATLAB + Simulink  
- Simulink 3D Animation / Vehicle Dynamics (for Simulation 3D blocks) — optional but supported  
- Tested with variable-step solver (`ode23t`) and scene sample time **0.02 s** (50 Hz)

---

## 🚀 Quick Start

```matlab
% From the repo root:
run('scripts/acc_aeb_params.m');     % defines P and time series
run('scripts/build_acc_aeb_top.m');   % creates models/acc_aeb_top.slx
set_param('acc_aeb_top','SimulationCommand','start');  % run

% (Optional) If you have Simulation 3D blocks in the model:
run('scripts/configure_unreal_blocks.m');  % align sample times & initial poses
```

---

## Where to Look

- **Scopes open automatically:** Scope_Speeds, Scope_RelDistance, Scope_AEB, Scope_TTC.
- **In Unreal:** add Simulation 3D Scene Configuration + two Simulation 3D Vehicle with Ground Following blocks.
- **Connect:**
- Unreal_Bridge/EgoX_scalar -> EgoVehicle/X
- Unreal_Bridge/LeadX_scalar -> LeadVehicle/X
- Leave Y/Yaw (and rotations) at zero constants for a simple 1-lane longitudinal demo.

---

## 🗺️ Project Structure

```bash
.
├─ models/                      # Generated Simulink models (.slx)
└─ scripts/
   ├─ acc_aeb_params.m          # Parameters (P), scenarios → timeseries
   ├─ build_acc_aeb_top.m       # Programmatic model builder
   ├─ configure_unreal_blocks.m # Auto-tuning for Simulation 3D blocks (optional)
   └─ scenario_library.m        # Scenario helper (optional)
```

---

## 🛠️ How It Works

**Scenarios**
- lead_speed_ts and driver_set_speed_ts are timeseries(t); lead position is integrated from speed with P.scenario.lead_x0.
**Sensors (top level)**
- Compute RelSpeed = v_lead - v_ego, RelDistance = x_lead - x_ego.
**ACC**
- v_gap = max((RelDistance - d0)/tau, 0), then v_ref = min(v_set, v_gap).
- PI speed controller → throttle/brake split → saturation (0…1) → rate limiters.
- Tip: ACC brake authority can be capped (e.g. 0.3) so AEB can override harder.
**AEB**
- Closing speed = max(-RelSpeed, eps); TTC = RelDistance / closingSpeed.
- Scale = (TTC_warn - TTC)/(TTC_warn - TTC_brake) → saturate 0–1.
- Gate to zero when opening (RelSpeed >= 0).
**Plant**
- a_net = a_trac - a_brake - (k_drag*v^2 + g*Cr) → jerk limiter → comfort sat → integrate to v and x.

---

## ⚙️ Key Parameters (edit in acc_aeb_params.m)

- **Simulation:** P.sim.dt = 1/50, P.sim.t_end = 40
- **Vehicle:** mass, Cd, A, rho, Cr, g
- **Actuators:** tau_throttle, tau_brake, a_max, a_brake_max
- **ACC:** timeGap (tau), minDistance (d0), PI gains & output limits
- **AEB:** eps, TTC_warn, TTC_brake
- **Scenario:** ego_v0, ego_x0, initialGap, piecewise v_lead, constant v_set

---

## 🧪 Demo Scenario (AEB should trigger)

In the provided parameters, the lead slows sharply (to near-stop) while ego tries to hold a higher set speed.

You should see:
- Ego close in (ACC reduces speed to maintain gap),
- Lead hard decel region → AEB ramps brake command toward 1.0,
- Vehicles avoid collision, then speed up and re-establish time-gap.

---

## 🧰 Troubleshooting

**No scopes opening**
- Ensure build_acc_aeb_top.m ran to completion; it sets OpenAtSimulationStart=on.
**AEB never fires**
- Increase P.controller.AEB.TTC_warn/TTC_brake or reduce initialGap.
- Cap ACC brake (e.g., Sat_Brake.UpperLimit = 0.3) so AEB has headroom.
**Unreal vehicles not moving**
- Verify block mask names: Simulation 3D Vehicle with Ground Following.
- Connect EgoX_scalar/LeadX_scalar to each vehicle’s X input.
- Run configure_unreal_blocks.m to set sample times and initial poses.

---

## 🙌 Notes

- The entire model is generated; delete models/acc_aeb_top.slx and re-run the build to regenerate.
