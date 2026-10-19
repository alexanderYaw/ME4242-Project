# EMIO motor GUI

Localhost web GUI to drive the EMIO's four Dynamixel motors independently, with live encoder readout
(pulses, angle, cable displacement, velocity, current, temperature).

## Run it (Python 3.10+)

Plug in and power on the EMIO, then:

- **Linux / macOS:** `./run.sh`
- **Windows:** double-click `run.bat`

and open http://localhost:8000. The first run creates a `.venv` folder and installs the dependencies
(needs internet); later runs start immediately. Stop with Ctrl+C.

Options (environment variables): `EMIO_MOCK=1` simulated motors, no hardware; `EMIO_PORT=/dev/ttyUSB1`
(or `COM5`) force a serial port; `PORT=8001` use another web port.

Linux: if connecting fails with "permission denied", add yourself to the dialout group
(`sudo usermod -aG dialout $USER`) and log out and back in.

Manual start instead of the scripts:
```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt      # Windows: .venv\Scripts\pip ...
cd app && ../.venv/bin/python -m uvicorn server:app --host 127.0.0.1 --port 8000
```

<details><summary>Docker (Linux only; the container needs the USB serial device)</summary>

```bash
docker compose up -d --build          # -> http://localhost:8000
EMIO_MOCK=1 docker compose up -d      # simulated motors
docker compose down                   # stop
```
</details>

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
