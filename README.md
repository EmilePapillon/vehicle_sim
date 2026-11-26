# Vehicle Dynamics Simulator

A vehicle dynamics simulator with a C++20 physics core, a headless batch CLI, and a native
macOS SwiftUI + Metal front-end.

<img width="1909" height="958" alt="image" src="https://github.com/user-attachments/assets/32100e5c-f8ce-45c4-91fb-703e21a1ec0a" />

```
core/      VehicleCore — C++20 physics library, no dependencies
capi/      vd_capi.h — stable C API (used by the app; the integration point for Unreal, Python, …)
cli/       vdsim — headless batch runner (scenarios, parameter sweeps, CSV + metrics)
tests/     vdtests — 35 unit + physics validation tests
app/       VehicleSimApp — SwiftUI panels, Swift Charts plots, Metal 3D view, top-down view
scenarios/ example scenario files      vehicles/ sedan.json, sports.json
```

## Build & run

Everything builds with the Xcode toolchain; CMake is only needed for the CMake build.

```sh
swift run -c release VehicleSimApp        # GUI
swift run -c release vdsim run scenarios/*.json -o out
swift run -c release vdtests              # tests
./scripts/make_app.sh                     # dist/VehicleSim.app + dist/vdsim

cmake -B build && cmake --build build -j && (cd build && ctest)   # core, CLI and tests without Swift
```

## Physics model

28 states, integrated at a fixed step (default 0.5 ms, RK4; Heun/Euler selectable):

| Subsystem | Model |
|---|---|
| Body | Planar x, y, yaw + sprung-mass heave, roll and pitch on the suspension |
| Load transfer | Elastic (through springs/dampers/ARBs), geometric (roll/pitch centres), unsprung mass; wheel lift-off |
| Tires | `Linear` (friction-circle capped), `Brush` (combined slip), `Pacejka` (Magic Formula, load-dependent μ and stiffness, camber thrust, combined slip). Transient slip via relaxation length; low-speed damping |
| Suspension | `Linear` / `Progressive` spring with motion ratio, bump stops, digressive bump/rebound dampers, ARBs, static camber, camber-with-roll, toe |
| Drivetrain | Engine torque curve, idle governor, rev limiter, engine braking; automatic launch clutch; 1–8 speed gearbox (automatic or manual); `Open` / `Locked` (torsional spool) / `Clutch LSD` / `Viscous` diffs; RWD / FWD / AWD with centre diff |
| Brakes & aids | Pedal torque with front bias, handbrake, optional ABS and traction control |
| Aero | Drag, downforce with front/rear balance (acts through the suspension) |
| Steering | Ratio, lock, partial Ackermann |

Conventions: ISO 8855 (x forward, y left, z up), SI units, radians in the API. Roll positive = left side up,
pitch positive = nose down. Corners are FL, FR, RL, RR.

### Validation (`vdtests`)
These are checked against closed-form results, not just "it runs":

- Linear bicycle model: steady-state yaw rate within 1%.
- Understeer gradient from a ramp steer.
- Lateral load-transfer moment balance and roll angle vs roll stiffness.
- Longitudinal load transfer under braking.
- Coast-down vs drag and rolling resistance.
- Energy conservation with losses off.
- Static equilibrium.
- Braking distance with and without ABS; ABS must help.
- Gear shifts and 0–100 km/h.
- TCS slip limiting.
- A stiffer front ARB gives more understeer.
- A locked diff equalises wheel speeds.
- Path tracking: constant radius and double lane change.
- RK4 convergence and bit-exact determinism.
- Wheel lift.
- Fuzzed inputs on every tire × layout × diff combination.
- Parameter changes mid-run.
- Parallel batch results equal serial ones.

## Headless batch mode

```sh
vdsim run scenarios/arb_sweep.json -o out -j 8          # 12 runs in parallel
vdsim run scenarios/step_steer.json --set tire.front.mu=0.9 --vehicle vehicles/sports.json
vdsim params                                             # every parameter: key, value, range, unit
vdsim export-vehicle my_car.json                         # editable vehicle file
vdsim tire --axle rear --fz 3500                         # tire Fy(alpha) curve as CSV
```

Each run writes a telemetry CSV: body states, inputs, engine, and per-wheel Fz/Fx/Fy/slip/compression. All runs together also produce `out/summary.csv`, with one row per run giving the sweep values and metrics: steady-state ay and yaw rate, understeer gradient, yaw response time and overshoot, roll gradient, stopping distance, 0–100 km/h, load-transfer ratio, and more.

Scenario format (paths are relative to the scenario file):

```json
{
  "name": "step_steer",
  "vehicle": "../vehicles/sedan.json",
  "params": {"suspension.front.arb_rate": 15000},
  "duration": 6, "output_rate": 100, "initial_speed": 22.22,
  "maneuver": {
    "steer": {"type": "step", "angle_deg": 40, "t_start": 1.0, "ramp_time": 0.15},
    "speed": {"type": "hold", "target": 22.22}
  },
  "sweep": [{"param": "suspension.front.arb_rate", "values": [0, 12000, 25000]},
            {"param": "tire.front.model", "from": 0, "to": 2, "steps": 3}]
}
```

Steer types:
- `none`, `constant`, `step`, `ramp`
- `sine`, `chirp`
- `circle`: pure-pursuit path follower
- `lane_change`: ISO 3888-style double lane change
- `path`: arbitrary `[[x, y], …]`

Speed types:
- `none`
- `hold`: PI cruise control with an optional `accel` ramp
- `throttle`, `brake`: pedal steps with `t_start`/`t_end`, and optionally `hold_before`

## GUI

- **Sidebar**: built automatically from the core's parameter registry, so every parameter gets a slider, picker, toggle or stepper. It has a filter box, a "changed" count per group, and right-click to reset a parameter or copy its key. Vehicle presets are in the menu, along with open/save of vehicle JSON.
- **Viewport**:
  - Metal 3D view with chase, orbit (drag / scroll) and top camera modes.
  - Body roll and pitch, with an exaggeration setting.
  - Spinning, steering wheels, plus per-wheel tire force vectors and wheel-load bars.
  - Driven trail and the maneuver's reference path.
  - Top-down Canvas view with load circles, force arrows and the velocity vector; also available in split view.
- **Plots**: slip angles, wheel loads (load transfer), ax/ay, and the tire Fy–α curves for each axle with live wheel operating points. Hover over a plot for a crosshair readout.
- **Driving**:
  - Keyboard: W/S/A/D or arrow keys, Space for handbrake, R to reset, P to pause. Steering is speed-sensitive.
  - Game controllers work through GameController.
  - Or pick a scripted maneuver, or open any `vdsim` scenario file. Both use the same core driver as batch mode, so results match.
- Toolbar: run/pause, single step, restart, time scale from 0.1× to 2×.
- Launch options:
  - `--maneuver step|circle|sine|dlc|accel|brake` starts with that maneuver.
  - `--snapshot <dir> [maneuver] [seconds]` writes PNGs of every view and quits. Use it for automated visual checks.

## Extending

- **New tire / suspension / differential model**: subclass `TireModel`, `SuspensionModel` or `Differential`, then add it to the factory and the choices string (`kTireModelChoices`, …). It then shows up in the GUI picker, JSON, CLI and sweeps.
- **New parameter**: add a field to the params struct and one `b.real(...)` line in `params.cpp`. The registry exposes it everywhere.
- **New maneuver**: implement `SteerChannel` / `SpeedChannel` and add a branch in `makeManeuver`.
- **Other hosts**: the C API (`capi/include/vd_capi.h`) is plain C with POD structs. Python can load `build/libvehiclecore.dylib` with `ctypes`.

### Unreal Engine integration (outline)
1. Build the static library for the target platform:
   `cmake -B build-ue -DVD_BUILD_SHARED=OFF -DVD_BUILD_TESTS=OFF -DVD_BUILD_TOOLS=OFF && cmake --build build-ue --config Release`
   (on Windows, use the MSVC generator).
2. Create a plugin with a ThirdParty module:
   - Add `capi/include` to `PublicIncludePaths`.
   - Add `libvehiclecore.a` / `vehiclecore.lib` to `PublicAdditionalLibraries`.
3. Write a `UActorComponent` that owns a `vd_sim*`:
   - In `TickComponent`: `vd_set_inputs`, then `vd_advance(sim, DeltaTime)` (fixed internal substeps, deterministic), then `vd_get_telemetry`.
   - Drive the actor transform and the wheel bones from the telemetry.
4. Convert coordinates:
   - Unreal is left-handed (X forward, Y right, Z up) and uses cm, so use `UE = (x, -y, z) * 100`, and negate yaw and roll.
   - Keep one `vd_sim` per vehicle per thread.
5. The parameter registry (`vd_param_count` / `vd_param_info_at`) can generate editor details panels the same way the SwiftUI sidebar does.

Ideas for later: road height / friction maps (the tire `TireInput` already takes load and slip; a per-wheel `mu_scale` and contact height would plug into `Vehicle::derivatives`), a kinematic suspension (K&C tables), and steering compliance.
