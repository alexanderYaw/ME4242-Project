# EMIO motor GUI

Localhost web GUI to drive the EMIO's four Dynamixel motors independently, with live encoder readout
(pulses, angle, cable displacement, velocity, current, temperature).

## Setup on a new machine (Python only)

### 1. Prerequisites (once per machine)

- **Python 3.10 or newer**: check with `python3 --version` (Windows: `py --version`).
  - Ubuntu/Debian also need the venv module: `sudo apt install python3-venv`
  - Windows: install from python.org and tick "Add Python to PATH".
- **Git** to clone the repo (or download the repo as a ZIP).
- **Linux only**, serial port permission: `sudo usermod -aG dialout $USER`, then log out and back in.
- The EMIO's USB adapter (FTDI) is normally supported out of the box on Windows and macOS; on Windows the
  driver may install automatically the first time the EMIO is plugged in.

### 2. Get the code

```bash
git clone https://github.com/<you>/<repo-name>.git
cd <repo-name>
```

### 3. Connect the robot

Plug the EMIO into USB and power it on. **Quit Emio Labs completely** if it is installed: a running scene
overrides the motors, and only one program can use the motor port at a time.

### 4. Start the GUI

- **Linux / macOS:** `cd motor_gui && ./run.sh`
- **Windows:** double-click `motor_gui\run.bat` (or run it from a terminal in the `motor_gui` folder)

The first run creates `.venv` and installs the dependencies (needs internet, takes about a minute); later
runs start in a couple of seconds. Keep the terminal open while using the GUI; Ctrl+C stops it.

### 5. Open http://localhost:8000

The header should say **Connected · /dev/ttyUSB0** (or `COM3` etc. on Windows) and all four cards should
show a motor model. If it says no EMIO was found, check the USB cable and power, then click **Connect**.
To force a specific port: `EMIO_PORT=/dev/ttyUSB1 ./run.sh` (Windows: `set EMIO_PORT=COM5` before
`run.bat`).

### 6. First use

1. Turn **Torque** on for a motor; it holds its current position.
2. Jog with **+1° / +5°** to check it moves. From 0° only the positive direction works (0–180° limit).
3. Optional: move each motor to its reference position and press **Calibrate: set 0 here**. This creates
   `data/settings.json`, which also stores drum radius and limits (each card's Settings).
4. **TORQUE OFF ALL** (or `Esc`) releases everything.

No settings file is needed beforehand: the app starts with defaults (speed 30 ≈ 7 rpm, 20 mm drum radius,
0–180° soft limits, uncalibrated) and creates `data/settings.json` the first time something is changed.
Calibration depends on how each robot's cables/legs are mounted, so calibrate on each robot.

### Options

Environment variables, e.g. `EMIO_MOCK=1 ./run.sh` (Windows: `set EMIO_MOCK=1` then `run.bat`):

| Variable | Effect |
|---|---|
| `EMIO_MOCK=1` | Simulated motors, no hardware needed |
| `EMIO_PORT=/dev/ttyUSB1` / `COM5` | Use this serial port instead of the first FTDI port |
| `PORT=8001` | Serve the GUI on another port |

### Manual start (instead of the scripts)

```bash
cd motor_gui
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt      # Windows: .venv\Scripts\pip install -r requirements.txt
cd app && ../.venv/bin/python -m uvicorn server:app --host 127.0.0.1 --port 8000
```

<details><summary>Docker (Linux only; the container needs the USB serial device)</summary>

```bash
docker compose up -d --build          # -> http://localhost:8000
EMIO_MOCK=1 docker compose up -d      # simulated motors
docker compose down                   # stop
```
</details>

## Tip control (cable-driven arm)

The **Tip control** panel (below the motor cards) moves the arm tip to a target position (x, z) in the
calibrated camera frame, shows the tip live from above and the arm's bend from the side.

**Setup, once per robot** (scripts in the repo root use the EMIO Python, `~/emio-venv/bin/python`):

1. In the GUI, set each motor's taut zero (**Calibrate: set 0 here** where the cable just goes taut) and its
   max pull (Settings → Soft max, absolute degrees).
2. Calibrate the camera with the ArUco plate: `~/emio-venv/bin/python -m emioapi calibrate`, then check it
   with `~/emio-venv/bin/python check_camera_calibration.py` (2–4 mm error expected).
3. Put one green marker on the tip, set `MAX_SAFE_DEG` in `collect_tip_data.py` to each motor's max pull, and
   collect data (GUI running, camera free): `~/emio-venv/bin/python collect_tip_data.py` → `data/tip_<time>.csv`.
4. Fit the model: `~/emio-venv/bin/python fit_tip_model.py data/tip_<time>.csv` → writes
   `motor_gui/data/tip_model.json` (the GUI picks up a new fit automatically).

**Use:**

1. Start the GUI, and in a second terminal the camera feed: `~/emio-venv/bin/python tip_feed.py`
   (`--show` for the camera windows). The panel shows "camera: tip seen" when it's live.
2. Click the top view (or type X/Z and press **Move tip**). With **Correct with camera** on, it moves using
   the model, then corrects from the camera until the tip is within 3 mm (friction makes the model alone
   about ±9 mm). Targets outside the reachable area go to the nearest reachable point.

**Joystick:** press **Start joystick control** in the panel, then either drag the **on-screen stick** with
the mouse or a finger (let go to hold; it recentres), or use a USB **gamepad** (Xbox / PlayStation style; press
any button on it first so the browser sees it): the left stick steers, hold **RT / R2** (or RB / R1) to move and
let go to hold, **B / Circle** = torque off all. The on-screen stick takes over while it is being dragged.
**Stick control:** *jog* (default) — the stick sets how fast the target moves (squared response, so small
pushes are fine adjustments); let go and it stays where it is, the next push continues from there. *Position* —
the stick points at a spot (full deflection = edge of reach / full bend); letting go holds where the arm is.
**Motor-pair mode is a direct mapping with no combined limit:** stick x drives motors 0/2 and y drives 1/3,
each pair independently up to its motor's full pull (soft max). The stick's round range is stretched onto a square,
so a full diagonal pulls both neighbouring motors fully. Per-motor soft limits, the 1000 mA force wall, the
1100 mA cut-off and the motors' own overload protection still apply. (Tip mode keeps the combined circle limit,
which is what its model covers.)
**Motor-pair mode keeps the cables taut:** on the first trigger press all four cables are wound in to their taut
zero (so at the centre all four motors are active, just taut). When a pair bends, one motor winds in and its
opposite unwinds by the same angle at the same time, down to slack, so crossing the centre is continuous (no
slack to take up). Opposite motors never both pull. If a paying-out motor's current rises above 300 mA (its
cable going tight), it unwinds further by itself. Tip mode, Move tip and data collection keep the original
behaviour (the non-pulling cable rests slack), which is what the tip model was fitted with. "Stick moves the tip" maps the stick to a direction in the
top view (full deflection = edge of the reachable area, needs the model); "the motor pairs" maps x to motors
0/2 and y to motors 1/3. The arm follows the stick at the speed limit and holds if the trigger is released,
the tab is hidden or the gamepad disconnects. A force limit acts as a wall until you ease the stick back.

Moves follow the pair rules (motors 0/2 and 1/3 oppose; only one of each pair pulls; a cable is released
before its opposite pulls), back off above 1000 mA and release everything above 1100 mA. **Stop**, `Esc` or
**TORQUE OFF ALL** interrupt a move; manual motor commands are refused while it moves.
API: `POST /api/tip/goto {"x":40,"z":-30,"closed_loop":true}`, `POST /api/tip/move_bend {"A":0.5,"B":0}`,
`POST /api/tip/joy/start`, `POST /api/tip/joy {"mode":"tip","x":0,"y":1,"enable":true}`, `POST /api/tip/stop`;
the camera feed posts to `POST /api/tip`.

## Notes

- Close Emio Labs / SOFA first. A running Emio Labs scene with Robot connection on sends every motor a goal
  ~100x/s (0 deg for motors not in the scene) and overrides this GUI. The GUI opens the port exclusively and
  flags any motor whose goal is changed by another program ("overridden by another program").
- Connecting never moves anything. Turning a motor's torque on holds its current position.
- Goals are clamped to the soft limits (default 0-180 deg, EMIO's range) and the motor's firmware limits.
- `Esc` or **TORQUE OFF ALL** disables every motor.
- Angles follow emioapi: `angle_rad = (2048 - pulse) * 2pi / 4096`.
- **Calibrate: set 0 here** (per motor) / **Calibrate all** makes the current position 0. Angle, goal, slider
  and jog are then relative to it, and cable displacement = `(angle - zero) * drum radius`. The zero is a
  software offset saved in `data/settings.json` (with drum radius and limits); the motors themselves are not
  changed, so the raw encoder value and Emio Labs are unaffected. Soft limits stay in absolute angles.
- REST API (same as the GUI uses): `GET /api/state`, `POST /api/motor/{id}/torque {"on":true}`,
  `POST /api/motor/{id}/goal {"angle_rad":0.5,"relative":true}`, `POST /api/motor/{id}/zero`,
  `POST /api/zero_all`, `POST /api/torque_off_all`.
