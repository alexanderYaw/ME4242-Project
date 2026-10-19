"""Localhost GUI for jogging the EMIO's four motors independently, with live encoder readout."""
import asyncio
import json
import math
import os
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field

import motors

MOCK = os.environ.get("EMIO_MOCK", "0") == "1"
DATA_DIR = Path(os.environ.get("DATA_DIR", Path(__file__).resolve().parent.parent / "data"))
SETTINGS_FILE = DATA_DIR / "settings.json"
POLL_HZ = 20
DEFAULT_MOTOR_SETTINGS = {"radius_mm": 20.0, "zero_rad": None, "invert": False, "min_deg": 0.0, "max_deg": 180.0}
DEFAULT_SETTINGS = {"profile_velocity": 30, "motors": [dict(DEFAULT_MOTOR_SETTINGS) for _ in motors.IDS]}


def load_settings():
    try:
        s = json.loads(SETTINGS_FILE.read_text())
        s.setdefault("profile_velocity", DEFAULT_SETTINGS["profile_velocity"])
        s["motors"] = [{**DEFAULT_MOTOR_SETTINGS, **m} for m in s.get("motors", [])][:len(motors.IDS)]
        s["motors"] += [dict(DEFAULT_MOTOR_SETTINGS) for _ in range(len(motors.IDS) - len(s["motors"]))]
        return s
    except (OSError, ValueError):
        return json.loads(json.dumps(DEFAULT_SETTINGS))


class Controller:
    def __init__(self):
        self.bus = None
        self.static = {}
        self.state = {}
        self.error = None
        self.commanded = {}  # motor id -> (goal pulse we last sent, time sent)
        self.swap_lock = threading.Lock()
        self.settings = load_settings()
        threading.Thread(target=self._poll, daemon=True).start()

    def save_settings(self):
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        tmp = SETTINGS_FILE.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.settings, indent=2))
        tmp.replace(SETTINGS_FILE)

    def connect(self, port=None):
        self.disconnect()
        if MOCK:
            bus = motors.MockBus()
        else:
            port = port or os.environ.get("EMIO_PORT") or motors.find_emio_port()
            if not port:
                raise RuntimeError("No EMIO (FTDI) serial port found. Is the EMIO plugged in?")
            bus = motors.DynamixelBus(port)
        try:
            static = bus.static_info()
            bus.set_profile_velocity(self.settings["profile_velocity"])
        except Exception:
            bus.close()
            raise
        with self.swap_lock:
            self.bus, self.static, self.error = bus, static, None
            self.commanded = {}

    def disconnect(self):
        with self.swap_lock:
            bus, self.bus, self.state = self.bus, None, {}
        if bus:
            bus.close()

    def _poll(self):
        while True:
            t0 = time.monotonic()
            bus = self.bus
            if bus:
                try:
                    self.state = bus.read_state()
                except Exception as e:
                    self.error = f"Lost connection: {e}"
                    self.disconnect()
            time.sleep(max(0.0, 1 / POLL_HZ - (time.monotonic() - t0)))

    def require_bus(self):
        if not self.bus:
            raise HTTPException(409, "Not connected")
        return self.bus

    def limits_rad(self, i):
        """Allowed angle range: soft limits intersected with the motor's firmware limits."""
        s = self.settings["motors"][i]
        lo, hi = math.radians(s["min_deg"]), math.radians(s["max_deg"])
        st = self.static.get(i)
        if st:
            lo = max(lo, motors.to_angle(st["max_pulse"]))
            hi = min(hi, motors.to_angle(st["min_pulse"]))
        return lo, hi

    def zero_rad(self, i):
        """Calibrated zero (absolute angle); uncalibrated motors use emioapi's zero (pulse 2048)."""
        z = self.settings["motors"][i]["zero_rad"]
        return z if z is not None else 0.0

    def calibrate(self, i):
        m = self.state.get(i)
        if m is None:
            raise HTTPException(404, f"Motor {i} not available")
        self.settings["motors"][i]["zero_rad"] = motors.to_angle(m["pulse"])

    def snapshot(self):
        out = {"connected": self.bus is not None, "port": self.bus.port_name if self.bus else None,
               "mock": MOCK, "error": self.error, "profile_velocity": self.settings["profile_velocity"],
               "motors": []}
        for i in motors.IDS:
            s = self.settings["motors"][i]
            st, m = self.static.get(i), self.state.get(i)
            lo, hi = self.limits_rad(i)
            entry = {"id": i, "online": m is not None, "settings": s, "limits_rad": [lo, hi]}
            if st:
                name, ma = motors.MODELS.get(st["model"], (f"model {st['model']}", None))
                entry.update(model=name, mode=st["mode"], ma_per_unit=ma,
                             fw_limits_rad=[motors.to_angle(st["max_pulse"]), motors.to_angle(st["min_pulse"])],
                             fw_limits_pulse=[st["min_pulse"], st["max_pulse"]])
            entry["calibrated"] = s["zero_rad"] is not None
            if m:
                zero = self.zero_rad(i)
                angle = motors.to_angle(m["pulse"])
                goal = motors.to_angle(m["goal_pulse"]) if m["goal_pulse"] is not None else None
                entry.update(m, angle_rad=angle, goal_rad=goal, angle_rel_rad=angle - zero,
                             goal_rel_rad=goal - zero if goal is not None else None,
                             velocity_rpm=m["velocity_raw"] * motors.VELOCITY_UNIT_RPM)
                cmd = self.commanded.get(i)
                # Our last goal was replaced by someone else on the bus (e.g. an Emio Labs scene).
                entry["overridden"] = bool(m["torque"] and cmd and time.monotonic() - cmd[1] > 0.5
                                           and m["goal_pulse"] is not None and abs(m["goal_pulse"] - cmd[0]) > 1)
                if entry["calibrated"]:
                    entry["cable_mm"] = (angle - zero) * s["radius_mm"] * (-1 if s["invert"] else 1)
            out["motors"].append(entry)
        return out


ctl = Controller()
STATIC = Path(__file__).parent / "static"


@asynccontextmanager
async def lifespan(_app):
    try:
        ctl.connect()  # reads state only; never changes torque or moves a motor
    except Exception as e:
        ctl.error = str(e)
    yield
    ctl.disconnect()


app = FastAPI(title="EMIO motor GUI", lifespan=lifespan)


@app.get("/")
def index():
    return FileResponse(STATIC / "index.html")


@app.get("/api/state")
def get_state():
    return ctl.snapshot()


@app.get("/api/ports")
def get_ports():
    return motors.list_ports()


class ConnectReq(BaseModel):
    port: str | None = None


@app.post("/api/connect")
def connect(req: ConnectReq):
    try:
        ctl.connect(req.port)
    except Exception as e:
        ctl.error = str(e)
        raise HTTPException(500, str(e))
    return ctl.snapshot()


@app.post("/api/disconnect")
def disconnect():
    ctl.disconnect()
    return ctl.snapshot()


class TorqueReq(BaseModel):
    on: bool


@app.post("/api/motor/{mid}/torque")
def torque(mid: int, req: TorqueReq):
    bus = ctl.require_bus()
    if mid not in motors.IDS:
        raise HTTPException(404, "Unknown motor")
    try:
        bus.set_torque(mid, req.on)
    except RuntimeError as e:
        raise HTTPException(500, str(e))
    if ctl.state.get(mid):  # don't wait for the next poll before accepting goals
        ctl.state[mid] = {**ctl.state[mid], "torque": req.on}
    ctl.commanded.pop(mid, None)
    return {"ok": True}


class GoalReq(BaseModel):
    angle_rad: float
    relative: bool = False  # True: angle is relative to the calibrated zero


@app.post("/api/motor/{mid}/goal")
def goal(mid: int, req: GoalReq):
    bus = ctl.require_bus()
    m = ctl.state.get(mid)
    if mid not in motors.IDS or m is None:
        raise HTTPException(404, "Motor not available")
    if not m["torque"]:
        raise HTTPException(409, "Enable torque on this motor first")
    target = req.angle_rad + (ctl.zero_rad(mid) if req.relative else 0.0)
    lo, hi = ctl.limits_rad(mid)
    angle = min(max(target, lo), hi)
    try:
        bus.set_goal(mid, motors.to_pulse(angle))
    except RuntimeError as e:
        raise HTTPException(500, str(e))
    ctl.commanded[mid] = (motors.to_pulse(angle), time.monotonic())
    return {"angle_rad": angle, "angle_rel_rad": angle - ctl.zero_rad(mid), "clamped": angle != target}


@app.post("/api/hold")
def hold():
    """Set every powered motor's goal to where it is now."""
    bus = ctl.require_bus()
    errors = []
    for i, m in ctl.state.items():
        if m and m["torque"]:
            try:
                bus.set_goal(i, m["pulse"])
                ctl.commanded[i] = (m["pulse"], time.monotonic())
            except RuntimeError as e:
                errors.append(str(e))
    return {"ok": not errors, "errors": errors}


@app.post("/api/torque_off_all")
def torque_off_all():
    bus = ctl.require_bus()
    errors = []
    for i in motors.IDS:
        try:
            bus.set_torque(i, False)
        except RuntimeError as e:
            errors.append(str(e))
    return {"ok": not errors, "errors": errors}


@app.post("/api/motor/{mid}/zero")
def set_zero(mid: int):
    """Calibrate: the motor's current position becomes 0."""
    ctl.calibrate(mid)
    ctl.save_settings()
    return ctl.settings["motors"][mid]


@app.post("/api/zero_all")
def zero_all():
    """Calibrate every responding motor at its current position."""
    done = []
    for i in motors.IDS:
        if ctl.state.get(i) is not None:
            ctl.calibrate(i)
            done.append(i)
    if not done:
        raise HTTPException(409, "No motor available")
    ctl.save_settings()
    return {"calibrated": done}


class MotorSettingsReq(BaseModel):
    radius_mm: float | None = Field(None, gt=0)
    invert: bool | None = None
    min_deg: float | None = None
    max_deg: float | None = None
    clear_zero: bool = False


@app.post("/api/motor/{mid}/settings")
def motor_settings(mid: int, req: MotorSettingsReq):
    if mid not in motors.IDS:
        raise HTTPException(404, "Unknown motor")
    s = ctl.settings["motors"][mid]
    lo = req.min_deg if req.min_deg is not None else s["min_deg"]
    hi = req.max_deg if req.max_deg is not None else s["max_deg"]
    if lo >= hi:
        raise HTTPException(422, "Min must be below max")
    for k in ("radius_mm", "invert", "min_deg", "max_deg"):
        if getattr(req, k) is not None:
            s[k] = getattr(req, k)
    if req.clear_zero:
        s["zero_rad"] = None
    ctl.save_settings()
    return s


class SpeedReq(BaseModel):
    profile_velocity: int = Field(ge=1, le=200)


@app.post("/api/speed")
def speed(req: SpeedReq):
    ctl.settings["profile_velocity"] = req.profile_velocity
    ctl.save_settings()
    if ctl.bus:
        ctl.bus.set_profile_velocity(req.profile_velocity)
    return {"profile_velocity": req.profile_velocity}


@app.websocket("/ws")
async def ws(sock: WebSocket):
    await sock.accept()
    try:
        while True:
            await sock.send_text(json.dumps(ctl.snapshot()))
            await asyncio.sleep(1 / POLL_HZ)
    except (WebSocketDisconnect, RuntimeError):
        pass
