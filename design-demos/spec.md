# Spec — EMIO-Labs Corset effector controller, interface redesign

Common input for all three directions. It says *what* the interface must do and say, not what it looks like
(that is each direction card in `directions.md`).

Source of truth: `motor_gui/app/static/index.html`, `scene3d.js`, `motor_gui/app/server.py`,
`motor_gui/app/tipctl.py`, `motor_gui/app/motors.py`. Prototypes are standalone and fully simulated
(`shared/sim.js`); they never touch the backend or hardware.

## Who and where

- Disaster first responders operating a cable-driven soft arm (4 Dynamixel XM430-W210 motors) through a
  laptop or tablet, often gloved, standing, in bright daylight or at night, under stress and interruption.
- They need: where is the tip, is it safe to move, how do I move it, how do I stop it, what is wrong.
- Engineers use the same screen for setup and diagnosis, less often.

## Output format (identical for all three directions)

- One standalone HTML page per direction: `dir-1.html`, `dir-2.html`, `dir-3.html`, sharing
  `shared/sim.js` (simulated backend + alert derivation) and `shared/views.js` (workspace drawings).
- Must work at desktop 1440×900 and tablet 1024×768 (landscape) and 834×1112 (portrait).
- Day (bright) and night (low-light) themes, switchable; both high contrast.
- A clearly separated **Simulation** harness (`shared/harness.js`) to switch scenarios and inject
  conditions. It is not part of the design; it is labelled SIMULATION.

## Functional inventory (preserve all)

### Global, always reachable
| Item | Source behaviour |
|---|---|
| Connection state | `Connected · <port>` (+ `(simulated)` when mock), `Disconnected`, `Server unreachable — retrying` (WebSocket closed) |
| Simulated operation indicator | `state.mock` |
| Active control mode + movement status | tip status `state.tip.status.state` (idle / moving / done / limited / failed / stopped) + message; joystick `state.tip.joy.active/enable/message` |
| Speed | slider 5–200 (profile velocity units), shown as rpm = value × 0.229; POST `/api/speed` on change |
| Hold all | POST `/api/hold`: stops tip control and sets every powered motor's goal to its current position |
| TORQUE OFF ALL | POST `/api/torque_off_all` (also stops tip control). `Esc` key and gamepad **B / Circle** do the same. Not an emergency stop: motors go limp, nothing holds position. |
| Fault summary | count + access to details (see Alerts) |

Secondary (setup): port select (`auto (first FTDI)` + listed ports, `· EMIO` marks the robot), Connect,
Disconnect, **Calibrate all** (confirm: "Set the current position of every motor as 0?").

Four different stops, never merged: **Hold all** (freeze where it is, torque stays), **tip Stop**
(`/api/tip/stop`: ends a tip move, holds), **joystick Stop** (same endpoint, ends joystick control),
**TORQUE OFF ALL** (release everything).

### Operate (main workspace)
- On-screen joystick: drag = moving (dead-man); release recentres and the arm holds.
- Gamepad state: none / `<name>` / non-standard layout ("hold RB to move"); enable state: released /
  "trigger held: moving" / "dragging: moving". Hold **RT/R2** (or RB/R1) to move; B/Circle = torque off all.
- Start joystick control / joystick Stop. Mapping: **tip** (top-view direction; needs tip model) or
  **motor pairs** (push toward a motor to pull it). Control: **jog** (deflection = speed) or **position**
  (deflection = where). Mapping and control are locked while joystick control is active.
- Target X / Z (mm), **Move tip**, tip **Stop**, **Correct with camera (closed loop)** checkbox,
  **Move when I click the top view** checkbox.
- Movement feedback: state, message, error mm, iteration.
- Positions: tip (camera) x, z; tip (model, from motor angles) x, z; height y (camera, or model labelled
  "(model)"); bend dA / dB.
- Views: live camera (with rod outline), top view (x, z: reachable area, camera tip, model tip, target,
  gamepad target, rod + near/hit shading, dashed planned path), side view (bending plane, platform,
  attachment, bend angle), 3D view (point cloud, model arm, camera arm, rod + clearance zone, tip surface,
  planned path; double-click surface = preview, never moves).
- Clear distinction: **preview** (planned, not moving) vs **moving**; **camera** vs **model** position.

### Motors (0–3; physical layout from above: 1 top-left, 0 top-right, 2 bottom-left, 3 bottom-right; pairs A = 0/2, B = 1/3)
Overview per motor: online/offline ("no response" when connected), torque, moving, angle (relative, °),
goal (°), current (mA if `ma_per_unit`, else raw `u`), temperature °C, HW error, override, uncalibrated.
Detail: slider (soft limits lo–hi relative to zero; disabled unless online + torque), jog −5/−1/+1/+5°,
numeric target + Go, torque toggle (disabled when offline, or HW error with torque off), **Reboot motor**
(only with HW error; confirm "Reboot motor N? Release cable tension first. It restarts with torque off."),
**Calibrate: set 0 here**, telemetry: pulses, angle, radians, cable mm, goal, velocity rpm, current,
absolute angle, zero-at, model, temperature. Settings: drum radius mm, invert cable sign, soft min/max
(° absolute), Save, Clear calibration. Clamp note "at limit: command clamped" (2.5 s after a clamped goal).
Goals are refused while tip control moves the arm ("Tip control is moving the arm; press Stop first").

### Setup & diagnostics
Serial connection; individual + global calibration; per-motor settings; calibration reset; tip model
(samples, median error, fitted date) or "no model — run fit_tip_model.py"; obstacle: avoid on/off,
clearance mm (0–100), Pick rod colour (then click rod in camera view), Mark rod ends (click two ends),
Forget rod; joystick invert x / y; 3D display options (cloud + crop mm, tip surface, clearance zone,
camera arm, planned path, Front/Top/Camera's view, Play path); diagnostics and engineering notes
(emioapi angle convention, pulse 2048 = 0°, cable = (angle − zero) × drum radius, dev commands).

## Conditions to represent (all must be demonstrable)
| Condition | Data | Severity |
|---|---|---|
| HW error on motor N | `hw_error` bits: 1 input voltage, 4 overheating, 8 encoder, 16 electrical shock, 32 overload; other = `code N` | Fault, motor + global; action Reboot (confirm) |
| Motor not responding | connected but `online:false` | Fault |
| Overridden by another program | `overridden` | Warning ("close Emio Labs / SOFA scene") |
| Uncalibrated | `calibrated:false` | Caution |
| Command clamped | goal response `clamped:true` | Notice, transient, beside motor |
| Disconnected | `connected:false` (+ `error`, e.g. "Lost connection: No motor answered") | Fault; telemetry becomes unknown |
| Server unreachable | WebSocket closed | Fault; everything stale |
| Missing tip model | `tip.model:false` | Caution; tip mode, Move tip and planning unavailable |
| Camera feed off / stale | `tip.feed` null / `age_s ≥ 0.5` | Caution / Warning (closed loop degraded) |
| Tip marker not detected | feed live, `markers:0` | Warning |
| Obstacle tracking | feed off → can't pick/mark; rod not seen (using last known); pick/mark error | Caution / Warning |
| Insufficient modelled clearance | `clear_model_mm < clearance_mm` (touching when < 0) | Warning / Fault |
| Backend current limit | tip `limited` "force limit (motor N at X mA); stopped short"; joystick wall "force limit on motor N: ease off…" | Warning |
| Hard current limit / movement error | tip `failed` with message (e.g. "motor 2 hardware error 32; released motors to rest") | Fault |

Rules: measured current, the HW overload flag and backend limit messages are three different things and are
labelled separately; no invented thresholds. Acknowledging an alert does not clear it; it stays listed as
"acknowledged, still active" until the condition is no longer reported. Unknown/stale data is shown as
unknown, never as healthy. Raw codes are available on expansion.

## Control semantics (unchanged)
No automatic motion, torque, calibration, fault reset or retry on panel change. Torque off does not hold.
Modelled clearance is a model estimate, not a guarantee. Confirmations preserved (Calibrate all, Reboot).
