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
git clone https://github.com/alexanderYaw/ME4242-Project.git
cd ME4242-Project
```

### 3. Connect the robot

Plug the EMIO into USB and power it on. **Quit Emio Labs completely** if it is installed: a running scene
overrides the motors, and only one program can use the motor port at a time.

### 4. Start the GUI

- **Linux / macOS:** `cd motor_gui && ./run.sh`
- **Windows:** double-click `motor_gui\run.bat` (or run it from a terminal in the `motor_gui` folder)

The first run creates `.venv` and installs the dependencies (needs internet, takes about a minute); later
runs start in a couple of seconds. Keep the terminal open while using the GUI; Ctrl+C stops it. Only one GUI can
use port 8000: if the Docker version below is running, stop it first (`docker compose down`) or use `PORT=8001`.

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

### Docker (Linux only, instead of the scripts)

The container gets the USB serial device from the host and keeps running in the background. Run these from the
`motor_gui` folder:

```bash
cd motor_gui
docker compose up -d --build          # build and start -> http://localhost:8000
EMIO_MOCK=1 docker compose up -d      # simulated motors
docker compose logs -f                # server output
docker compose down                   # stop
```

The code is copied into the image, so after changing anything in `motor_gui/app/` run
`docker compose up -d --build` again (rebuilding never moves the motors). Settings, the tip model and the obstacle
are kept in `motor_gui/data/`, which is mounted into the container.

## Tip control (cable-driven arm)

The **Tip control** panel (below the motor cards) moves the arm tip to a target position (x, z) in the
calibrated camera frame, shows the tip live from above and the arm's bend from the side.

**EMIO Python, once per machine.** The scripts in the repo root (camera feed, calibration, data collection,
fitting) use the camera through `emioapi` and run from a separate venv, `~/emio-venv`:

```bash
python3 -m venv ~/emio-venv
~/emio-venv/bin/pip install emioapi==0.2.0     # also installs numpy, opencv-python, pyrealsense2
```

(Windows: `py -3 -m venv %USERPROFILE%\emio-venv`, then use `%USERPROFILE%\emio-venv\Scripts\python` wherever
this README says `~/emio-venv/bin/python`.) Run the scripts from the repo root.

**Setup, once per robot:**

1. In the GUI, set each motor's taut zero (**Calibrate: set 0 here** where the cable just goes taut) and its
   max pull (Settings → Soft max, absolute degrees).
2. Calibrate the camera with the ArUco plate: `~/emio-venv/bin/python -m emioapi calibrate`, then check it
   with `~/emio-venv/bin/python check_camera_calibration.py` (2–4 mm error expected).
3. Put one green marker on the tip and collect data (GUI running and connected, camera free, so `tip_feed.py`
   stopped): `~/emio-venv/bin/python collect_tip_data.py` → `data/tip_<time>.csv`. Each motor's max pull is read
   from its soft max in the GUI (step 1).
4. Fit the model: `~/emio-venv/bin/python fit_tip_model.py data/tip_<time>.csv` → writes
   `motor_gui/data/tip_model.json` (the GUI picks up a new fit automatically). The default model is physical: the arm
   bends as a circular arc, each motor adds bend toward its own direction, so reach saturates like the real arm's
   (two adjacent motors pulled fully reach only a little further out than one; the tip mostly rises). Tip control
   stays below 120° of bend, where the tip would start curling back. If some cables weren't connected during
   collection, `--mirror 1,2` estimates those motors from their opposite ones. `motor_gui/data/` is not in git, so
   a fresh clone has no tip model: tip control (Move tip, tip-mode joystick, obstacle planning) needs this fit
   first; the motor cards and motor-pair joystick work without it.

**Use:**

1. Start the GUI, and in a second terminal the camera feed: `~/emio-venv/bin/python tip_feed.py`
   (`--show` for the camera windows). The panel shows "camera: tip seen" when it's live.
2. Click the top view (or type X/Z and press **Move tip**). With **Correct with camera** on, it moves using
   the model, then corrects from the camera until the tip is within 3 mm (friction makes the model alone
   about ±9 mm). Targets outside the reachable area go to the nearest reachable point.

**Joystick:** press **Start joystick control** in the panel, then either drag the **on-screen stick** with
the mouse or a finger (let go to hold; it recentres), or use a USB **gamepad** (Xbox / PlayStation style; press
any button on it first so the browser sees it): the left stick steers, hold **RT / R2** (or RB / R1) to move and
let go to hold, **B / Circle** = torque off all. **Bluetooth** works too: unplug the cable, put the controller in
pairing mode and run `./pair_controller.sh` once (Linux); afterwards pressing the Xbox button reconnects it. If the
browser reports a non-standard button layout (the panel then says "hold RB to move"), use **RB** as the dead-man. The on-screen stick takes over while it is being dragged.
**Stick control:** *jog* (default) — the stick sets how fast the target moves (squared response, so small
pushes are fine adjustments); let go and it stays where it is, the next push continues from there. *Position* —
the stick points at a spot (full deflection = edge of reach / full bend); letting go holds where the arm is.
**Orientation:** the top view and the stick follow the robot's physical layout: motor 1 top left, motor 0 top right,
motor 2 bottom left, motor 3 bottom right (`LAYOUT_DEG` in `index.html`). The top view is turned to match using the
directions the model says each motor bends the tip; small +x / +z arrows show the camera axes for the X / Z boxes.
**Motor-pair mode is a direct mapping with no combined limit:** push the stick toward a motor to pull it (towards the
top-right corner = motor 0); straight up, between motors 1 and 0, pulls both. Each pair (0/2, 1/3) goes independently
up to its motor's full pull (soft max); the stick's round range is stretched so that full deflection straight between
two motors pulls both fully. Per-motor soft limits, the 1000 mA force wall, the
1100 mA cut-off and the motors' own overload protection still apply. Tip mode, Move tip and the shaded reachable
area use the same square range (the tip model is fitted on the square grid), so the tip reaches the corners too.
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

### Obstacle avoidance (rod)

`tip_feed.py` also looks for an obstacle rod: by default an **orange** one (`--rod-color red|orange|yellow|pink|purple`,
or **Pick rod colour** in the panel and click the rod in the camera view; `--no-rod` turns it off). It fits the rod's
coloured pixels with a 3D line from the depth camera (`rod_detect.py`): ends, radius, and an end that runs out of the
image is taken to continue 150 mm (into the pillar). The rod is static, so detections are merged: its known length only
grows, and the arm passing in front can't hide part of it. **Mark rod ends** defines a rod of any colour (e.g. the grey
prototype) by clicking its two ends in the camera view; a marked rod stays until **Forget rod**. The live view outlines
the rod's colour in magenta and draws the rod in red.

With **Plan moves around it** on, the GUI models the **whole arm**, not just the tip: the constant-curvature fit gives
the arm's arc for any motor command, and **Move tip** follows a path in motor (bend) space that keeps every point of the
arc at least the set clearance from the rod's surface (Dijkstra over a bend grid, then straightened). Paths may curl
the arm beyond tip mode's 120° limit; that is how it gets **over the top** of a rod at mid-height (pull two adjacent
motors hard so the whole arm lifts above the rod, swing across, lower on the other side). An arm that starts inside
the clearance first backs away; a target too close to the rod goes to the nearest clear point; if no path keeps the
clearance, the move is refused with the widest gap available (lower the clearance below it to allow that path). The
joystick slides along the rod instead of pushing into it. The top view shades tip positions where the arm would touch
the rod (red) or be inside the clearance (orange) and dashes the planned path; the side view shows where the rod crosses
the bending plane. **Arm clearance** shows the model's distance (from motor angles) and the camera's (the arc through
the measured tip). The clearance is from the arm's centreline: allow for the disc radius plus the model error (±10 mm
typical, more at large bends, which were never measured). With the prototype rod beside the arm at mid-height, the way
over the top passes about 16 mm from it at best.
API: `POST /api/obstacle/settings {"avoid":true,"clearance_mm":20}`, `POST /api/obstacle/mark`, `POST /api/obstacle/clear`,
`GET /api/obstacle/map`; the feed posts to `POST /api/obstacle` (a rod can be posted by hand too:
`{"rod":{"p0":[x,y,z],"p1":[x,y,z],"radius":5}}`).

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
