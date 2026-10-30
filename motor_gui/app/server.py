"""Localhost GUI for jogging the EMIO's four motors independently, with live encoder readout."""
import asyncio
import collections
import hashlib
import json
import math
import os
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

import motors
import tipctl

MOCK = os.environ.get("EMIO_MOCK", "0") == "1"
DATA_DIR = Path(os.environ.get("DATA_DIR", Path(__file__).resolve().parent.parent / "data"))
SETTINGS_FILE = DATA_DIR / "settings.json"
POLL_HZ = 20
CURRENT_FILTER_N = 3  # readings (at POLL_HZ) a current must stay above a limit before it counts: ~0.15 s
DEFAULT_MOTOR_SETTINGS = {"radius_mm": 20.0, "zero_rad": None, "invert": False, "min_deg": 0.0, "max_deg": 180.0}
PROFILE_ACCELERATION = 20  # x 214.6 rev/min^2: ~0.2 s ease-in/out at 15 rpm (0 = instant, abrupt)
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
        # Current limits act on the *sustained* current: the lowest of the last CURRENT_FILTER_N readings
        # (20 Hz), so a single brief spike can't trip them.
        self.current_hist = {i: collections.deque(maxlen=CURRENT_FILTER_N) for i in motors.IDS}
        self.sustained_raw = {}
        self.read_glitches = {i: 0 for i in motors.IDS}  # dropped register reads per motor (diagnostics)
        self.swap_lock = threading.Lock()
        self.settings = load_settings()
        threading.Thread(target=self._poll, daemon=True).start()

    def apply_motion_profile(self, bus):
        """Speed limit and acceleration ramp. Motors forget both after a reboot or power loss."""
        bus.set_profile_velocity(self.settings["profile_velocity"])
        bus.set_profile_acceleration(PROFILE_ACCELERATION)

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
            self.apply_motion_profile(bus)
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
                    state = bus.read_state()
                    prev = self.state
                    for i, m in state.items():
                        if m is None:
                            continue
                        # A dropped/garbled reply for one register block leaves those fields None: keep the
                        # last good value for this cycle instead of reporting e.g. "torque off" by mistake.
                        missing = [k for k, v in m.items() if v is None]
                        if missing:
                            self.read_glitches[i] += 1
                            if prev.get(i):
                                for k in missing:
                                    m[k] = prev[i].get(k)
                        self.current_hist[i].append(abs(m["current_raw"]))
                    self.sustained_raw = {i: min(h) for i, h in self.current_hist.items() if h}
                    self.state = state
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
                entry["current_sustained_raw"] = self.sustained_raw.get(i, abs(m["current_raw"]))
                entry["read_glitches"] = self.read_glitches[i]
                entry["overridden"] = bool(m["torque"] and cmd and time.monotonic() - cmd[1] > 0.5
                                           and m["goal_pulse"] is not None and abs(m["goal_pulse"] - cmd[0]) > 1)
                if entry["calibrated"]:
                    entry["cable_mm"] = (angle - zero) * s["radius_mm"] * (-1 if s["invert"] else 1)
            out["motors"].append(entry)
        return out


ctl = Controller()
tip = tipctl.TipController(ctl, DATA_DIR / "tip_model.json")
STATIC = Path(__file__).parent / "static"
UI_FILES = ("index.html", "scene3d.js")
UI_VERSION = hashlib.sha1(b"".join((STATIC / f).read_bytes() for f in UI_FILES)).hexdigest()[:12]  # open pages reload when it changes
camera = {"jpeg": None, "t": 0.0}  # latest live-view image from tip_feed.py
cloud = {"data": None, "t": 0.0, "seq": 0}  # latest point cloud from tip_feed.py (3D view)


def full_snapshot():
    s = ctl.snapshot()
    s["tip"] = tip.snapshot()
    s["camera"] = {"age_s": time.monotonic() - camera["t"] if camera["jpeg"] else None}
    s["cloud"] = {"age_s": time.monotonic() - cloud["t"] if cloud["data"] else None, "seq": cloud["seq"]}
    s["ui_version"] = UI_VERSION
    return s


@asynccontextmanager
async def lifespan(_app):
    try:
        ctl.connect()  # reads state only; never changes torque or moves a motor
    except Exception as e:
        ctl.error = str(e)
    yield
    ctl.disconnect()


app = FastAPI(title="EMIO motor GUI", lifespan=lifespan)
app.mount("/static", StaticFiles(directory=STATIC), name="static")


@app.get("/")
def index():
    # always pick up GUI updates; the version tag makes the browser fetch a changed scene3d.js too
    html = (STATIC / "index.html").read_text().replace("{{UI_VERSION}}", UI_VERSION)
    return HTMLResponse(html, headers={"Cache-Control": "no-cache"})


@app.get("/api/state")
def get_state():
    return full_snapshot()


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
    m = ctl.state.get(mid)
    if req.on and m and m["hw_error"]:
        raise HTTPException(409, f"Motor {mid} has hardware error {m['hw_error']}; reboot it first")
    try:
        if req.on:
            # A motor that rebooted or lost power comes back with no speed limit: re-apply it.
            ctl.apply_motion_profile(bus)
        bus.set_torque(mid, req.on)
    except RuntimeError as e:
        raise HTTPException(500, str(e))
    if ctl.state.get(mid):  # don't wait for the next poll before accepting goals
        ctl.state[mid] = {**ctl.state[mid], "torque": req.on}
    ctl.commanded.pop(mid, None)
    return {"ok": True}


@app.post("/api/motor/{mid}/reboot")
def reboot(mid: int):
    """Clear a hardware error (e.g. overload). The motor restarts with torque off."""
    bus = ctl.require_bus()
    if mid not in motors.IDS:
        raise HTTPException(404, "Unknown motor")
    try:
        bus.reboot(mid)
    except RuntimeError as e:
        raise HTTPException(500, str(e))
    ctl.apply_motion_profile(bus)
    ctl.commanded.pop(mid, None)
    return {"ok": True}


class GoalReq(BaseModel):
    angle_rad: float
    relative: bool = False  # True: angle is relative to the calibrated zero


@app.post("/api/motor/{mid}/goal")
def goal(mid: int, req: GoalReq):
    bus = ctl.require_bus()
    if tip.busy:
        raise HTTPException(409, "Tip control is moving the arm; press Stop first")
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
    tip.stop()
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
    tip.stop()
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


class TipFeedReq(BaseModel):
    x: float | None = None
    y: float | None = None
    z: float | None = None
    markers: int = 0


@app.post("/api/tip")
def tip_feed(req: TipFeedReq):
    """Tip position from the camera (tip_feed.py)."""
    if req.markers and None in (req.x, req.y, req.z):
        raise HTTPException(422, "x, y, z required when a marker is seen")
    tip.add_feed(req.model_dump())
    return {"ok": True}


@app.post("/api/camera/frame")
async def camera_frame(request: Request):
    """Camera image (JPEG body) from tip_feed.py, for the live view."""
    body = await request.body()
    if not body.startswith(b"\xff\xd8"):
        raise HTTPException(415, "expected a JPEG image")
    camera.update(jpeg=body, t=time.monotonic())
    return {"ok": True}


@app.get("/api/camera/latest")
def camera_latest():
    if camera["jpeg"] is None:
        raise HTTPException(404, "No camera image; run tip_feed.py")
    return Response(camera["jpeg"], media_type="image/jpeg", headers={"Cache-Control": "no-store"})


@app.post("/api/camera/cloud")
async def camera_cloud(request: Request):
    """Point cloud of the workspace from tip_feed.py (binary, see CloudSender there), for the 3D view."""
    body = await request.body()
    if len(body) < 8:
        raise HTTPException(415, "expected a point cloud")
    cloud.update(data=body, t=time.monotonic(), seq=cloud["seq"] + 1)
    return {"ok": True}


@app.get("/api/camera/cloud")
def camera_cloud_latest(after: int = -1):
    """The latest point cloud; 204 if it isn't newer than `after`."""
    if cloud["data"] is None or cloud["seq"] <= after:
        return Response(status_code=204)
    return Response(cloud["data"], media_type="application/octet-stream",
                    headers={"Cache-Control": "no-store", "X-Seq": str(cloud["seq"])})


@app.get("/api/scene")
def scene():
    """3D view: tip surface, arm attachment and rod (changes with the model / obstacle)."""
    info = tip.scene_info()
    if info is None:
        raise HTTPException(404, "No tip model; run fit_tip_model.py")
    return info


@app.get("/api/tip/plan")
def tip_plan():
    """Arm shapes along the current / last move's planned path."""
    return tip.plan_shapes()


class ObstacleFeedReq(BaseModel):
    rods: list[dict] | None = None  # [{"p0": [x, y, z], "p1": [...], "radius": mm, "colour": name, ...}] by colour
    seen: bool = False              # rods seen in the latest image
    hsv_picked: list | None = None  # colour range picked at the clicked pixel
    marked: dict | None = None      # rod from marked ends
    error: str | None = None        # a pick / marking that failed
    objects: list[dict] | None = None  # other objects by colour (mesh, outline, shape); only sent when they change


@app.post("/api/obstacle")
def obstacle_feed(req: ObstacleFeedReq):
    """Rods and objects (obstacles) from tip_feed.py; the reply carries the colour and any pending pick / marking.
    Rods and objects found by colour are only shown; a marked rod is avoided."""
    for r in [*(req.rods or []), req.marked]:
        if r is not None and not (len(r.get("p0", [])) == 3 and len(r.get("p1", [])) == 3 and r.get("radius", -1) >= 0):
            raise HTTPException(422, "rod needs p0, p1 (x, y, z) and radius")
    for o in req.objects or []:
        if not all(k in o for k in ("id", "colour", "shape", "size_mm", "centre", "verts", "faces", "outline")):
            raise HTTPException(422, "object needs id, colour, shape, size_mm, centre, verts, faces and outline")
    return tip.obstacle_feed(req.rods, req.seen, req.hsv_picked, req.marked, req.error, req.objects)


@app.get("/api/obstacle/objects")
def obstacle_objects():
    """Objects found by colour, with their surface meshes (3D view)."""
    return tip.objects_info()


@app.get("/api/obstacle/map")
def obstacle_map():
    return tip.obstacle_map()


class ObstacleSettingsReq(BaseModel):
    avoid: bool | None = None
    clearance_mm: float | None = Field(None, ge=0, le=100)


@app.post("/api/obstacle/settings")
def obstacle_settings(req: ObstacleSettingsReq):
    tip.obstacle_settings(req.avoid, req.clearance_mm)
    return {"ok": True}


class ObstaclePickReq(BaseModel):
    u: float = Field(ge=0, le=1)  # position in the camera image, as fractions of its width / height
    v: float = Field(ge=0, le=1)


@app.post("/api/obstacle/pick")
def obstacle_pick(req: ObstaclePickReq):
    """Use the colour at this spot of the camera image as the rod's colour."""
    tip.obstacle_pick(req.u, req.v)
    return {"ok": True}


class ObstacleMarkReq(BaseModel):
    points: list[list[float]] = Field(min_length=2, max_length=2)  # the rod's two ends in the image (fractions)
    radius_mm: float = Field(5.0, gt=0, le=50)


@app.post("/api/obstacle/mark")
def obstacle_mark(req: ObstacleMarkReq):
    """Define the rod by its two ends in the camera image (any colour); tip_feed.py measures their depth."""
    tip.obstacle_mark(req.points, req.radius_mm)
    return {"ok": True}


@app.post("/api/obstacle/clear")
def obstacle_clear():
    tip.obstacle_clear()
    return {"ok": True}


@app.get("/api/tip/model")
def tip_model():
    info = tip.model_info()
    if info is None:
        raise HTTPException(404, "No tip model; run fit_tip_model.py")
    return info


class TipGotoReq(BaseModel):
    x: float
    z: float
    closed_loop: bool = True


class TipPreviewReq(BaseModel):
    x: float
    z: float


@app.post("/api/tip/preview")
def tip_preview(req: TipPreviewReq):
    """Plan a move to (x, z) without moving: arm shapes along the way (around the obstacle)."""
    try:
        return tip.plan_preview(req.x, req.z)
    except ValueError as e:
        raise HTTPException(409, str(e))


@app.post("/api/tip/goto")
def tip_goto(req: TipGotoReq):
    try:
        tip.goto(req.x, req.z, req.closed_loop)
    except ValueError as e:
        raise HTTPException(409, str(e))
    return tip.status


class MoveBendReq(BaseModel):
    A: float = Field(ge=-1, le=1)
    B: float = Field(ge=-1, le=1)
    soft_ma: float | None = None
    hard_ma: float | None = None


@app.post("/api/tip/move_bend")
def tip_move_bend(req: MoveBendReq):
    """Blocking smooth move to bend values (pairs 0/2 = A, 1/3 = B), for collect_tip_data.py."""
    try:
        return tip.move_bend(req.A, req.B, req.soft_ma, req.hard_ma)
    except ValueError as e:
        raise HTTPException(409, str(e))
    except RuntimeError as e:
        raise HTTPException(500, str(e))


class JoyReq(BaseModel):
    mode: str = Field("tip", pattern="^(tip|bend)$")
    x: float = Field(0.0, ge=-1.5, le=1.5)
    y: float = Field(0.0, ge=-1.5, le=1.5)
    enable: bool = False
    control: str = Field("rate", pattern="^(rate|position)$")


@app.post("/api/tip/joy/start")
def joy_start():
    try:
        tip.joy_start()
    except ValueError as e:
        raise HTTPException(409, str(e))
    return {"ok": True}


@app.post("/api/tip/joy")
def joy_input(req: JoyReq):
    """Stick input from the GUI (~20 Hz). The arm only moves while `enable` (the dead-man trigger) is held
    and inputs keep arriving."""
    try:
        tip.joy_input(req.mode, req.x, req.y, req.enable, req.control)
    except ValueError as e:
        raise HTTPException(409, str(e))
    return {"ok": True}


@app.post("/api/tip/stop")
def tip_stop():
    tip.stop()
    return {"ok": True}


@app.websocket("/ws")
async def ws(sock: WebSocket):
    await sock.accept()
    try:
        while True:
            await sock.send_text(json.dumps(full_snapshot()))
            await asyncio.sleep(1 / POLL_HZ)
    except (WebSocketDisconnect, RuntimeError):
        pass
