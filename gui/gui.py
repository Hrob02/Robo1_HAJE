"""
HAJE Engineering - Fuel Assessment & Risk Tool (GUI)
Windows-first build with external command runner (cmd.exe) + ReportLab PDFs.

Updates in this version:
- Mock camera frames keep displaying until ROS topics are actually received.
- No synthetic odom/imu/gps/scan are produced. UI shows "— waiting for ROS —" until data arrives.
- When ROS telemetry (any of odom/imu/gps/scan) is first received, mock cameras stop and
  ROS images (if cv_bridge is installed) will populate; otherwise we keep mock cams but still log telemetry.

Dependencies for ROS mode (optional):
  pip install rclpy
  pip install cv_bridge  (optional, for image topics)

Change topic names below if yours differ.
"""

from __future__ import annotations
import csv, json, math, platform, subprocess, textwrap, time, threading, shlex, os
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from threading import Lock, Thread
from typing import Any, Dict, List, Optional

# ----------------------------- Third-party -----------------------------
import customtkinter as ctk
from tkinter import messagebox
from PIL import Image

# Optional OpenCV for webcam fallback (not required for ROS)
try:
    import cv2
except Exception:
    cv2 = None

# Optional ReportLab (pure-Python PDF)
try:
    from reportlab.lib.pagesizes import A4
    from reportlab.pdfgen import canvas
    from reportlab.lib.units import mm
    from reportlab.lib.utils import ImageReader
    HAVE_RL = True
except Exception:
    HAVE_RL = False

# Optional ROS 2 stack
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan, NavSatFix, Image as RosImage
    HAVE_ROS = True
except Exception:
    HAVE_ROS = False

# Optional cv_bridge for ROS image conversion
try:
    from cv_bridge import CvBridge
    HAVE_CVBRIDGE = True
except Exception:
    HAVE_CVBRIDGE = False

# ----------------------------- Theme -----------------------------
BG          = "#0F0F11"
CARD        = "#1B1C1F"
TEXT        = "#D7DBE0"
SUBTEXT     = "#9AA3AD"
ACCENT      = "#E10600"
ACCENT_2    = "#FF3B2E"
BORDER      = "#2E3238"
WARN        = "#FFB020"
OK          = "#13A10E"

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

# ----------------------------- Helpers -----------------------------
def open_file(path: Path):
    try:
        sysname = platform.system()
        if sysname == "Windows":
            os.startfile(str(path))  # type: ignore[attr-defined]
        elif sysname == "Darwin":
            subprocess.Popen(["open", str(path)])
        else:
            subprocess.Popen(["xdg-open", str(path)])
    except Exception:
        pass

def now_id() -> str:
    return time.strftime("%Y%m%d-%H%M%S")

# ----------------------------- Behaviour State -----------------------------
class BState(str, Enum):
    OFF="OFF"; IDLE="IDLE"; TAKEOFF="TAKEOFF"; EXEC="EXEC"; SUSPEND="SUSPEND"; LAND="LAND"

# ----------------------------- Command pattern (+ undo/redo) ---------------
@dataclass
class Command:
    name: str
    params: Dict[str, Any] = field(default_factory=dict)
    def execute(self, ctx: "Context"): ...
    def undo(self, ctx: "Context"): ...

class TakeoffCmd(Command):
    def execute(self, ctx): ctx.set_state(BState.TAKEOFF); ctx.log_io("cmd","takeoff", self.params)
    def undo(self, ctx):    ctx.set_state(BState.IDLE);    ctx.log_io("undo","takeoff", {})

class SuspendCmd(Command):
    def execute(self, ctx): ctx.set_state(BState.SUSPEND); ctx.log_io("cmd","suspend", self.params)
    def undo(self, ctx):    ctx.set_state(BState.EXEC);    ctx.log_io("undo","suspend", {})

class LandCmd(Command):
    def execute(self, ctx): ctx.set_state(BState.LAND);    ctx.log_io("cmd","land", self.params)
    def undo(self, ctx):    ctx.set_state(BState.IDLE);    ctx.log_io("undo","land", {})

class CmdStack:
    def __init__(self): self.done: List[Command]=[]; self.undone: List[Command]=[]
    def do(self, cmd: Command, ctx: "Context"):
        cmd.execute(ctx); self.done.append(cmd); self.undone.clear()
    def undo(self, ctx: "Context"):
        if not self.done: return
        c=self.done.pop(); c.undo(ctx); self.undone.append(c)
    def redo(self, ctx: "Context"):
        if not self.undone: return
        c=self.undone.pop(); c.execute(ctx); self.done.append(c)

# ----------------------------- Data Recorder ------------------------------
class DataRecorder:
    """CSV logger + frame saver."""
    def __init__(self, session_dir: Path, keep_images=50):
        self.dir = Path(session_dir); self.dir.mkdir(parents=True, exist_ok=True)
        (self.dir/"frames").mkdir(exist_ok=True)
        self._lock = Lock()
        self._images: List[str] = []
        self._keep = keep_images
        self._csvs = {
            "odom": self._writer("odom", ["t","x","y","yaw"]),
            "gps":  self._writer("gps",  ["t","lat","lon","alt"]),
            "imu":  self._writer("imu",  ["t","roll","pitch","yaw"]),
            "scan": self._writer("scan", ["t","points","mean_range"]),
            "io":   self._writer("io",   ["t","kind","name","data"]),
        }
        self.meta = {"start": time.time()}

    def _writer(self, stem: str, fields: List[str]):
        f = open(self.dir/f"{stem}.csv", "w", newline="")
        w = csv.DictWriter(f, fieldnames=fields); w.writeheader()
        return (f, w)

    def add_row(self, stream: str, row: Dict[str, Any]):
        with self._lock:
            f, w = self._csvs.get(stream, (None, None))
            if w: w.writerow(row)

    def add_frame(self, img: Image.Image, cam: str, t: float):
        p = self.dir/"frames"/f"{cam}_{time.strftime('%H%M%S')}_{int((t%1)*1000):03d}.jpg"
        try:
            img.save(p, "JPEG", quality=88)
            self._images.append(str(p))
            self._images = self._images[-self._keep:]
        except Exception:
            pass

    def latest_images(self, n=6) -> List[str]:
        return self._images[-n:]

    def close(self):
        with self._lock:
            for f,_ in self._csvs.values():
                try: f.flush(); f.close()
                except Exception: pass
        (self.dir/"session_meta.json").write_text(json.dumps(self.meta|{"end": time.time()}, indent=2))

# ----------------------------- GUI Context ---------------------------------
class Context:
    """Holds session state for commands + logging."""
    def __init__(self, session_dir: Path, ui_cb_set_state, ui_cb_status):
        self.session_dir = session_dir
        self.state = BState.OFF
        self.ui_set_state = ui_cb_set_state
        self.ui_status = ui_cb_status
        self.rec = DataRecorder(session_dir)

    def set_state(self, s: BState):
        self.state = s
        self.ui_set_state(s)

    def log_io(self, kind: str, name: str, data: Dict[str, Any]):
        self.rec.add_row("io", {"t": time.time(), "kind": kind, "name": name, "data": json.dumps(data)})

# ----------------------------- External Command Runner ---------------------
class CommandRegistry:
    def __init__(self, json_path: Path, log_cb):
        self.json_path = Path(json_path)
        self.log_cb = log_cb
        self.commands: Dict[str,str] = {}
        self.load()

    def load(self):
        try:
            if self.json_path.exists():
                self.commands = json.loads(self.json_path.read_text())
            else:
                self.commands = {}
        except Exception as e:
            self.log_cb(f"[ERROR] Failed to load {self.json_path.name}: {e}")

    def run(self, name: str, cwd: Path | None = None, new_console: bool = True):
        cmd = self.commands.get(name)
        if not cmd:
            self.log_cb(f"[WARN] Command '{name}' not found in {self.json_path.name}")
            return
        sys = platform.system()
        popen_kwargs = dict(
            cwd=str(cwd or Path.cwd()),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            shell=True
        )
        try:
            if sys == "Windows" and new_console:
                flags = getattr(subprocess, "CREATE_NEW_CONSOLE", 0x00000010)
                proc = subprocess.Popen(cmd, creationflags=flags, **popen_kwargs)
            else:
                if sys != "Windows" and not cmd.strip().startswith(("bash", "sh")) and any(c in cmd for c in "|&;><$~"):
                    cmd = f"bash -lc {shlex.quote(cmd)}"
                proc = subprocess.Popen(cmd, **popen_kwargs)
        except Exception as e:
            self.log_cb(f"[ERROR] spawn '{name}': {e}")
            return

        threading.Thread(target=self._pump, args=(name, proc), daemon=True).start()

    def _pump(self, name: str, proc: subprocess.Popen):
        self.log_cb(f"[RUN] {name}: pid={proc.pid}")
        try:
            assert proc.stdout is not None
            for line in proc.stdout:
                self.log_cb(line.rstrip("\n"))
        except Exception as e:
            self.log_cb(f"[ERROR] {name}: {e}")
        rc = proc.wait()
        self.log_cb(f"[DONE] {name}: {'OK' if rc==0 else f'EXIT {rc}'}")

# ----------------------------- Mock Cameras ONLY ---------------------------
class MockCams:
    """Generates placeholder frames (no telemetry)."""
    def __init__(self, frame1_cb, frame2_cb, fps=10):
        self.frame1_cb=frame1_cb; self.frame2_cb=frame2_cb
        self.fps=fps; self._run=False
        self._cap1=None; self._cap2=None

    def start(self, video1=None, video2=None):
        self._run=True
        if cv2 and video1: self._cap1=cv2.VideoCapture(0 if video1=="webcam" else video1)
        if cv2 and video2: self._cap2=cv2.VideoCapture(0 if video2=="webcam" else video2)
        Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._run=False
        if self._cap1: self._cap1.release()
        if self._cap2: self._cap2.release()

    def _pull(self, cap, t)->Image.Image:
        if cap and cap.isOpened():
            ok,frame=(cap.read() if cv2 else (False,None))
            if ok and frame is not None:
                import cv2 as _cv2
                frame=_cv2.cvtColor(frame,_cv2.COLOR_BGR2RGB)
                from PIL import Image as _Image
                return _Image.fromarray(frame)
        # simple generated placeholder (red dot orbiting)
        from PIL import Image as _Image
        img=_Image.new("RGB",(640,360),(28,30,35))
        r=12; cx=int(320+180*math.cos(t/2)); cy=int(180+100*math.sin(t/2))
        for dx in range(-r,r):
            for dy in range(-r,r):
                if dx*dx+dy*dy<=r*r:
                    x2=cx+dx; y2=cy+dy
                    if 0<=x2<640 and 0<=y2<360: img.putpixel((x2,y2),(225,20,10))
        return img

    def _loop(self):
        while self._run:
            t=time.time()
            img1=self._pull(self._cap1,t); img2=self._pull(self._cap2,t)
            self.frame1_cb(img1,t); self.frame2_cb(img2,t)
            time.sleep(1/max(1,self.fps))

# ----------------------------- ROS Connector -------------------------------
class ROSConnector:
    """
    Spins a ROS2 node in the background. Calls UI callbacks when messages arrive.
    - When ANY telemetry arrives, sets 'connected' True (switch away from mock telemetry).
    - If cv_bridge is available, image topics are converted and sent to UI.
    """
    def __init__(self,
                 on_odom, on_imu, on_scan, on_gps,
                 on_cam1, on_cam2,
                 on_connected,
                 log_cb):
        self.on_odom=on_odom; self.on_imu=on_imu; self.on_scan=on_scan; self.on_gps=on_gps
        self.on_cam1=on_cam1; self.on_cam2=on_cam2
        self.on_connected=on_connected
        self.log_cb=log_cb
        self.connected=False
        self._thread=None
        self._bridge = CvBridge() if HAVE_CVBRIDGE else None

    def start(self):
        if not HAVE_ROS:
            self.log_cb("[INFO] ROS mode unavailable: rclpy not installed.")
            return
        self._thread = Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            rclpy.init(args=None)
            node = Node("haje_gui_bridge")

            def _set_connected():
                if not self.connected:
                    self.connected=True
                    self.on_connected()

            # Subscriptions (rename topics if your stack differs)
            node.create_subscription(Odometry,  "/odom", lambda msg: (self._cb_odom(msg), _set_connected()), 10)
            node.create_subscription(Imu,       "/imu",  lambda msg: (self._cb_imu(msg),  _set_connected()), 10)
            node.create_subscription(LaserScan, "/scan", lambda msg: (self._cb_scan(msg), _set_connected()), 10)
            node.create_subscription(NavSatFix, "/fix",  lambda msg: (self._cb_gps(msg),  _set_connected()), 10)

            if HAVE_CVBRIDGE:
                node.create_subscription(RosImage, "/camera1/image_raw", lambda msg: (self._cb_img(msg,1), _set_connected()), 10)
                node.create_subscription(RosImage, "/camera2/image_raw", lambda msg: (self._cb_img(msg,2), _set_connected()), 10)
            else:
                self.log_cb("[INFO] cv_bridge not installed: keeping mock cams even after ROS connects.")

            self.log_cb("[INFO] ROS bridge running. Waiting for topics…")
            rclpy.spin(node)
        except Exception as e:
            self.log_cb(f"[ERROR] ROS bridge error: {e}")
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass

    # ----- message adapters -----
    def _cb_odom(self, msg: Odometry):
        t=time.time()
        x=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        # yaw from quaternion (simple extract)
        q=msg.pose.pose.orientation
        yaw = self._yaw_from_quat(q.x,q.y,q.z,q.w)
        self.on_odom({"t":t,"x":x,"y":y,"yaw":yaw})

    def _cb_imu(self, msg: Imu):
        t=time.time()
        # roll/pitch/yaw from quaternion (approx)
        q=msg.orientation
        roll,pitch,yaw = self._rpy_from_quat(q.x,q.y,q.z,q.w)
        self.on_imu({"t":t,"roll":roll,"pitch":pitch,"yaw":yaw})

    def _cb_scan(self, msg: LaserScan):
        t=time.time()
        pts = len(msg.ranges)
        valid = [r for r in msg.ranges if (r is not None and r==r and r>0.0)]
        mean = sum(valid)/len(valid) if valid else 0.0
        self.on_scan({"t":t,"points":pts,"mean_range":mean})

    def _cb_gps(self, msg: NavSatFix):
        t=time.time()
        self.on_gps({"t":t,"lat":msg.latitude,"lon":msg.longitude,"alt":msg.altitude})

    def _cb_img(self, msg: RosImage, cam_idx: int):
        if not self._bridge: return
        try:
            cv = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            import cv2 as _cv2
            rgb = _cv2.cvtColor(cv, _cv2.COLOR_BGR2RGB)
            from PIL import Image as _Image
            pil = _Image.fromarray(rgb)
            t=time.time()
            if cam_idx==1: self.on_cam1(pil, t)
            else:          self.on_cam2(pil, t)
        except Exception as e:
            self.log_cb(f"[WARN] cv_bridge image conversion failed: {e}")

    @staticmethod
    def _yaw_from_quat(x,y,z,w):
        import math
        # yaw (Z axis rotation)
        siny_cosp = 2.0*(w*z + x*y); cosy_cosp = 1.0 - 2.0*(y*y + z*z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _rpy_from_quat(x,y,z,w):
        import math
        # roll
        sinr_cosp = 2*(w*x + y*z); cosr_cosp = 1 - 2*(x*x + y*y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # pitch
        sinp = 2*(w*y - z*x)
        if abs(sinp) >= 1: pitch = math.copysign(math.pi/2, sinp)
        else: pitch = math.asin(sinp)
        # yaw
        siny_cosp = 2*(w*z + x*y); cosy_cosp = 1 - 2*(y*y + z*z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

# ----------------------------- PDF (ReportLab) -----------------------------
def build_pdf(context: Dict[str,Any], out_dir: Path) -> Path:
    if not HAVE_RL:
        raise RuntimeError("ReportLab is required. Install with: pip install reportlab")
    out_dir.mkdir(parents=True, exist_ok=True)
    pdf_path = out_dir/f"report_{now_id()}.pdf"
    c = canvas.Canvas(str(pdf_path), pagesize=A4); W,H=A4

    # Title block
    c.setFont("Helvetica-Bold",16)
    c.drawString(20*mm,H-20*mm, context.get("title","Drone Mission Report"))
    c.setFont("Helvetica",10)
    c.drawString(20*mm,H-26*mm, f"Mission: {context.get('mission_id','')}")
    c.drawString(20*mm,H-31*mm, str(context.get("generated_at","")))
    y=H-40*mm

    # Summary
    c.setFont("Helvetica-Bold",12); c.drawString(20*mm,y,"Summary"); y-=6*mm
    c.setFont("Helvetica",10)
    for line in textwrap.wrap(context.get("summary","—"), 95):
        y-=5*mm
        if y<25*mm: c.showPage(); y=H-25*mm; c.setFont("Helvetica",10)
        c.drawString(20*mm,y,line)

    # Quantitative summary
    y-=8*mm
    c.setFont("Helvetica-Bold",12); c.drawString(20*mm,y,"Quantitative Summary"); y-=6*mm
    c.setFont("Helvetica",10)
    stats=context.get("stats",{})
    for line in [
        f"Mean odom.x = {stats.get('odom_x_mean','—')}",
        f"Mean odom.y = {stats.get('odom_y_mean','—')}",
        f"Mean scan range = {stats.get('scan_mean','—')}",
    ]:
        y-=5*mm
        if y<25*mm: c.showPage(); y=H-25*mm; c.setFont("Helvetica",10)
        c.drawString(20*mm,y,line)

    # Config
    y-=8*mm
    c.setFont("Helvetica-Bold",12); c.drawString(20*mm,y,"Configuration"); y-=6*mm
    c.setFont("Helvetica",10)
    for k,v in context.get("config",{}).items():
        y-=5*mm
        if y<25*mm: c.showPage(); y=H-25*mm; c.setFont("Helvetica",10)
        c.drawString(20*mm,y, f"{k}: {v}")

    # Live strings
    y-=8*mm
    c.setFont("Helvetica-Bold",12); c.drawString(20*mm,y,"Live Snapshots"); y-=6*mm
    c.setFont("Helvetica",10)
    for label, val in context.get("live",{}).items():
        y-=5*mm
        if y<25*mm: c.showPage(); y=H-25*mm; c.setFont("Helvetica",10)
        c.drawString(20*mm,y, f"{label}: {val}")

    # Images (limit 6)
    for p in context.get("images",[])[:6]:
        try:
            img=ImageReader(p); iw,ih=img.getSize()
            maxw=W-40*mm; maxh=55*mm; scale=min(maxw/iw, maxh/ih); w,h=iw*scale, ih*scale
            if y-h<20*mm: c.showPage(); y=H-20*mm
            c.drawImage(img, 20*mm, y-h, w, h, preserveAspectRatio=True, mask='auto'); y-=h+6*mm
        except Exception:
            pass

    c.showPage(); c.save(); return pdf_path

# ----------------------------- GUI -----------------------------------------
class HAJEGUI(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("HAJE Engineering Fuel Assessment & Risk Tool")
        self.geometry("1400x860"); self.minsize(1200,760); self.configure(fg_color=BG)

        # Session
        self.session_id: Optional[str] = None
        self.session_dir: Optional[Path] = None
        self.ctx: Optional[Context] = None
        self.stack = CmdStack()

        # Tabs
        self.tabs = ctk.CTkTabview(self, width=1300, height=800, fg_color=BG)
        self.tabs.pack(padx=16, pady=16, expand=True, fill="both")
        self.launch = self.tabs.add("Launch")
        self.live   = self.tabs.add("Live")
        self.report = self.tabs.add("Report")
        self._build_launch(); self._build_live(); self._build_report()

        # Shortcuts
        self.bind_all("<Control-t>", lambda e: (self._do(TakeoffCmd("takeoff")), self._run_cmd("takeoff")))
        self.bind_all("<Control-s>", lambda e: (self._do(SuspendCmd("suspend")), self._run_cmd("suspend")))
        self.bind_all("<Control-l>", lambda e: (self._do(LandCmd("land")),    self._run_cmd("land")))
        self.bind_all("<Control-z>", lambda e: self._undo())
        self.bind_all("<Control-y>", lambda e: self._redo())

        # Command registry (dictionary file)
        from pathlib import Path
        self.cmds = CommandRegistry(
            Path(__file__).resolve().parent.parent / "commands.json",
            log_cb=self._append_log
        )

        self._append_log(f"[DEBUG] Loaded commands: {list(self.cmds.commands.keys())}")

        # IMPORTANT: start with MockCams only (no telemetry)
        self.mock = MockCams(self._on_frame1, self._on_frame2, fps=10)
        self.mock.start()

        # Start ROS connector (if available). It will flip us to "connected" mode
        # once *any* telemetry message arrives.
        self.ros_connected = False
        self.ros = ROSConnector(
            on_odom=self._on_ros_odom,
            on_imu=self._on_ros_imu,
            on_scan=self._on_ros_scan,
            on_gps=self._on_ros_gps,
            on_cam1=self._on_frame1,
            on_cam2=self._on_frame2,
            on_connected=self._on_ros_connected,
            log_cb=self._append_log
        )
        self.ros.start()

    # ----------------- BUILD: Launch -----------------------------------
    def _build_launch(self):
        self.launch.grid_columnconfigure(0, weight=1)
        for r in range(6): self.launch.grid_rowconfigure(r, weight=0)
        self.launch.grid_rowconfigure(5, weight=1)

        ctk.CTkLabel(self.launch, text="1. LAUNCH", font=("Arial",26,"bold"), text_color=TEXT)\
            .grid(row=0, column=0, sticky="w", padx=20, pady=(16,8))

        top = self._card(self.launch); top.grid(row=1, column=0, sticky="ew", padx=20, pady=(6,8))
        top.body.grid_columnconfigure(3, weight=1)
        self._status_lbl = ctk.CTkLabel(top.body, text="Status: OFF", text_color=TEXT, font=("Arial",14,"bold"))
        self._status_lbl.grid(row=0, column=0, sticky="w", padx=6, pady=6)
        self._state_badge = ctk.CTkLabel(top.body, text=BState.OFF, fg_color=BORDER, corner_radius=10, padx=10)
        self._state_badge.grid(row=0, column=1, sticky="w", padx=6)

        # World selector dropdown
        self.world_var = ctk.StringVar(value="Select a World")
        self.world_menu = ctk.CTkOptionMenu(
            top.body,
            values=[
                "Small Test Site (2 Trees)",
                "Large Demonstration Field"
            ],
            variable=self.world_var,
            command=self._on_world_select
        )
        self.world_menu.set("Select a World")
        self.world_menu.grid(row=0, column=2, sticky="w", padx=8)

        # Start Session button (disabled by default)
        self.btn_start = ctk.CTkButton(
            top.body,
            text="START SESSION",
            command=self._start_session,
            fg_color="#444",  # greyed-out look
            state="disabled"
        )
        self.btn_start.grid(row=0, column=3, sticky="e", padx=8)

        self.btn_restart = ctk.CTkButton(top.body, text="CLEAN RESTART", command=self._clean_restart,
                                        fg_color="#444", hover_color="#555")
        self.btn_restart.grid(row=0, column=4, sticky="e", padx=8)

        self._msg = ctk.CTkTextbox(self.launch, height=140, fg_color="#121316",
                                    text_color=TEXT, border_color=BORDER, border_width=2,
                                    corner_radius=12)
        self._msg.grid(row=3, column=0, sticky="nsew", padx=20, pady=(6,16))
        self._append_log("[INFO] Ready. Select a world and start a session to begin.")


    # ----------------- BUILD: Live -------------------------------------
    def _build_live(self):
        for c in range(3): self.live.grid_columnconfigure(c, weight=1, uniform="live")
        self.live.grid_rowconfigure(1, weight=1); self.live.grid_rowconfigure(2, weight=1)
        ctk.CTkLabel(self.live, text="2. LIVE", font=("Arial",26,"bold"), text_color=TEXT)\
            .grid(row=0, column=0, columnspan=3, sticky="w", padx=20, pady=(16,8))

        # left: live data
        left = self._card(self.live); left.grid(row=1, column=0, rowspan=2, sticky="nsew", padx=(20,10), pady=(6,16))
        self.lbl_odom = ctk.CTkLabel(left.body, text="odom: — waiting for ROS —", text_color=TEXT); self.lbl_odom.pack(anchor="w", pady=4)
        self.lbl_gps  = ctk.CTkLabel(left.body, text="gps:  — waiting for ROS —",  text_color=TEXT); self.lbl_gps.pack(anchor="w", pady=4)
        self.lbl_imu  = ctk.CTkLabel(left.body, text="imu:  — waiting for ROS —",  text_color=TEXT); self.lbl_imu.pack(anchor="w", pady=4)
        self.lbl_scan = ctk.CTkLabel(left.body, text="scan: — waiting for ROS —", text_color=TEXT); self.lbl_scan.pack(anchor="w", pady=4)

        # mid: cameras
        mid1 = self._card(self.live); mid1.grid(row=1, column=1, sticky="nsew", padx=10, pady=(6,8))
        mid2 = self._card(self.live); mid2.grid(row=2, column=1, sticky="nsew", padx=10, pady=(8,16))
        self.cam1 = ctk.CTkLabel(mid1.body, text="CAM 1"); self.cam1.pack(expand=True, fill="both", padx=8, pady=8)
        self.cam2 = ctk.CTkLabel(mid2.body, text="CAM 2"); self.cam2.pack(expand=True, fill="both", padx=8, pady=8)

        # right: key/warnings + controls
        right = self._card(self.live); right.grid(row=1, column=2, sticky="nsew", padx=(10,20), pady=(6,8))
        ctk.CTkLabel(right.body, text="KEY / WARNINGS", text_color=TEXT, font=("Arial",18,"bold")).pack(anchor="w", pady=(4,6))
        self.warn_badge = ctk.CTkLabel(right.body, text="No warnings", fg_color=OK, corner_radius=8, padx=8)
        self.warn_badge.pack(padx=6, pady=6, anchor="w")

        ctrl = self._card(self.live); ctrl.grid(row=2, column=2, sticky="nsew", padx=(10,20), pady=(8,16))
        ctk.CTkLabel(ctrl.body, text="Controls (shortcuts)", text_color=TEXT, font=("Arial",18,"bold")).pack(anchor="w")
        ctk.CTkButton(ctrl.body, text="LAUNCH (open console)", command=lambda: self._run_cmd("launch")).pack(anchor="e", pady=6)
        ctk.CTkButton(ctrl.body, text="MOVEMENT", command=lambda: self._run_cmd("movement")).pack(anchor="e", pady=6)
        ctk.CTkButton(ctrl.body, text="TAKEOFF  (Ctrl+T)", command=lambda: (self._do(TakeoffCmd("takeoff")), self._run_cmd("takeoff"))).pack(anchor="e", pady=6)
        ctk.CTkButton(ctrl.body, text="SUSPEND (Ctrl+S)", command=lambda: (self._do(SuspendCmd("suspend")), self._run_cmd("suspend"))).pack(anchor="e", pady=6)
        ctk.CTkButton(ctrl.body, text="LAND    (Ctrl+L)", command=lambda: (self._do(LandCmd("land")), self._run_cmd("land"))).pack(anchor="e", pady=6)
        ctk.CTkButton(ctrl.body, text="Undo (Ctrl+Z)", command=self._undo).pack(anchor="e", pady=(18,4))
        ctk.CTkButton(ctrl.body, text="Redo (Ctrl+Y)", command=self._redo).pack(anchor="e", pady=4)

    # ----------------- BUILD: Report -----------------------------------
    def _build_report(self):
        for c in range(2): self.report.grid_columnconfigure(c, weight=1, uniform="rep")
        self.report.grid_rowconfigure(1, weight=1)
        ctk.CTkLabel(self.report, text="3. REPORT", font=("Arial",26,"bold"), text_color=TEXT)\
            .grid(row=0, column=0, columnspan=2, sticky="w", padx=20, pady=(16,8))

        left = self._card(self.report); left.grid(row=1, column=0, sticky="nsew", padx=(20,10), pady=(6,16))
        ctk.CTkLabel(left.body, text="Summary of Findings", font=("Arial",18,"bold")).pack(anchor="w")
        self.tb_summary = ctk.CTkTextbox(left.body); self.tb_summary.pack(expand=True, fill="both", pady=8)
        self.tb_summary.insert("end", "• Fuel hazard moderate in S-SE sector.\n• Flights stable; no signal drop.\n")

        right = self._card(self.report); right.grid(row=1, column=1, sticky="nsew", padx=(10,20), pady=(6,16))
        ctk.CTkLabel(right.body, text="Configuration", font=("Arial",18,"bold")).pack(anchor="w")
        self.tb_cfg = ctk.CTkTextbox(right.body, height=120); self.tb_cfg.pack(fill="x", pady=(4,10))
        self.tb_cfg.insert("end", "Mode: Standard\nMax Alt: 120 m\nPilot: A. Akhurst\n")
        ctk.CTkLabel(right.body, text="Mission Log (shell + system)", font=("Arial",18,"bold")).pack(anchor="w")
        self.tb_logs = ctk.CTkTextbox(right.body); self.tb_logs.pack(expand=True, fill="both", pady=6)
        self.tb_logs.insert("end", "System logs…\n")
        ctk.CTkButton(right.body, text="Generate Report", fg_color=ACCENT, hover_color=ACCENT_2,
                      command=self._gen_report).pack(anchor="e", pady=(10,0))

    # ----------------- helpers -----------------------------------------
    def _card(self, parent):
        f = ctk.CTkFrame(parent, fg_color=CARD, corner_radius=18)
        f.body = ctk.CTkFrame(f, fg_color=CARD, corner_radius=18)
        f.body.pack(expand=True, fill="both", padx=12, pady=12)
        return f

    def _append_log(self, line: str):
        try:
            self.tb_logs.insert("end", line + "\n"); self.tb_logs.see("end")
        except Exception:
            pass
        try:
            self._msg.insert("end", line + "\n"); self._msg.see("end")
        except Exception:
            pass

    def _status(self, txt: str): self._status_lbl.configure(text=f"Status: {txt}")

    def _set_state(self, s: BState):
        col = {"OFF":BORDER, "IDLE":OK, "TAKEOFF":"#0078D4", "EXEC":"#6A5ACD",
               "SUSPEND":WARN, "LAND":"#888"}[s.value]
        self._state_badge.configure(text=s.value, fg_color=col)
        self._status(s.value)
    
    def _on_world_select(self, choice: str):
        self._append_log(f"[DEBUG] on_world_select called with: {choice}")
        self.btn_start.configure(state="normal", fg_color=ACCENT, hover_color=ACCENT_2)
        self._append_log(f"[INFO] World selected: {choice}")

        if "Small Test Site" in choice:
            self.selected_world_cmd = self.cmds.commands.get("launch_small_world")
        elif "Large Demonstration" in choice:
            self.selected_world_cmd = self.cmds.commands.get("launch_large_world")
        else:
            self.selected_world_cmd = None

    # ----------------- session -----------------------------------------
    def _start_session(self):
        if not getattr(self, "selected_world_cmd", None):
            self._append_log("[WARN] Please select a world before starting.")
            return

        self.session_id = now_id()
        base = Path.cwd()/ "DroneReports" / "sessions"
        self.session_dir = base/self.session_id
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.ctx = Context(self.session_dir, self._set_state, self._status)
        self._set_state(BState.IDLE)

        # Save the command dynamically before running
        self.cmds.commands["launch_world"] = self.selected_world_cmd

        # Run it
        self._append_log(f"[INFO] Launching world with: {self.selected_world_cmd}")
        self.cmds.run("launch_world", cwd=self.session_dir, new_console=True)

    def _clean_restart(self):
        if messagebox.askyesno("Clean restart","Save report first?\nThis clears cached UI and resets states."):
            self._gen_report()
        # Reset UI/state
        self._state_badge.configure(text=BState.OFF, fg_color=BORDER)
        self._status("OFF")
        try: self._msg.delete("1.0","end")
        except Exception: pass
        try: self.tb_logs.delete("1.0","end")
        except Exception: pass
        self.stack = CmdStack()
        self.warn_badge.configure(text="No warnings", fg_color=OK)
        self.session_id=None; self.session_dir=None; self.ctx=None

    # ----------------- do/undo/redo ------------------------------------
    def _do(self, cmd: Command):
        if not self.ctx:
            self._append_log("[WARN] Start a session first.")
            return
        try:
            self.stack.do(cmd, self.ctx)
            self._append_log(f"[INFO] Executed: {cmd.name}")
        except Exception as e:
            self.warn_badge.configure(text="Command error", fg_color=WARN)
            self._append_log(f"[ERROR] {cmd.name}: {e}. Recovery: retry or land.")

    def _undo(self):
        if not self.ctx: return
        self.stack.undo(self.ctx); self._append_log("[INFO] Undo")
    def _redo(self):
        if not self.ctx: return
        self.stack.redo(self.ctx); self._append_log("[INFO] Redo")

    # ----------------- UI helpers --------------------------------------
    def _fmt(self,v,n=3):
        try: return f"{float(v):.{n}f}"
        except: return str(v)

    def _to_ctk(self, pil_img: Image.Image):
        from customtkinter import CTkImage
        res = pil_img.resize((640,360))
        return CTkImage(light_image=res, dark_image=res, size=(640,360))

    # Camera frame sinks
    def _on_frame1(self, pil_img: Image.Image, t: float):
        self.cam1.configure(image=self._to_ctk(pil_img))
        if self.ctx: self.ctx.rec.add_frame(pil_img, "cam1", t)

    def _on_frame2(self, pil_img: Image.Image, t: float):
        self.cam2.configure(image=self._to_ctk(pil_img))
        if self.ctx: self.ctx.rec.add_frame(pil_img, "cam2", t)

    # ----------------- ROS event sinks ---------------------------------
    def _on_ros_connected(self):
        self._append_log("[INFO] ROS telemetry connected.")
        self.warn_badge.configure(text="ROS connected", fg_color=OK)
        # Stop mock cams ONLY if we will receive ROS images via cv_bridge.
        if HAVE_CVBRIDGE:
            try:
                if getattr(self, "mock", None): self.mock.stop()
                self._append_log("[INFO] Switching camera source: ROS images (cv_bridge).")
            except Exception:
                pass

    def _on_ros_odom(self, d: Dict[str,Any]):
        self.lbl_odom.configure(text=f"odom: x={self._fmt(d['x'])}, y={self._fmt(d['y'])}, yaw={self._fmt(d['yaw'])}")
        if self.ctx: self.ctx.rec.add_row("odom", {"t":d["t"], "x":d["x"], "y":d["y"], "yaw":d["yaw"]})

    def _on_ros_imu(self, d: Dict[str,Any]):
        self.lbl_imu.configure(text=f"imu:  roll={self._fmt(d['roll'])}, pitch={self._fmt(d['pitch'])}, yaw={self._fmt(d['yaw'])}")
        if self.ctx: self.ctx.rec.add_row("imu", {"t":d["t"], "roll":d["roll"], "pitch":d["pitch"], "yaw":d["yaw"]})

    def _on_ros_scan(self, d: Dict[str,Any]):
        self.lbl_scan.configure(text=f"scan: {d['points']} pts, mean={self._fmt(d['mean_range'])} m")
        if self.ctx: self.ctx.rec.add_row("scan", d)

    def _on_ros_gps(self, d: Dict[str,Any]):
        self.lbl_gps.configure(text=f"gps:  lat={self._fmt(d['lat'],6)}, lon={self._fmt(d['lon'],6)}, alt={self._fmt(d['alt'])}")
        if self.ctx: self.ctx.rec.add_row("gps", d)

    # ----------------- external commands --------------------------------
    def _run_cmd(self, name: str, new_console: bool = True):
        if not self.session_dir:
            self._append_log("[WARN] Start a session first.")
            return
        env = os.environ.copy()
        if self.param_speed.get().strip(): env["HAJE_SPEED"] = self.param_speed.get().strip()
        if self.param_res.get().strip():   env["HAJE_RES"]   = self.param_res.get().strip()
        if self.param_dur.get().strip():   env["HAJE_DUR"]   = self.param_dur.get().strip()
        self._append_log(f"[INFO] Executing shell command '{name}'…")
        os.environ.update(env)
        self.cmds.run(name, cwd=self.session_dir, new_console=new_console)

    # ----------------- report ------------------------------------------
    def _collect_stats(self) -> Dict[str, Any]:
        stats = {"odom_x_mean":"—","odom_y_mean":"—","scan_mean":"—"}
        if not self.session_dir: return stats
        try:
            import statistics as st
            def read_csv(p: Path) -> List[dict]:
                return list(csv.DictReader(open(p, newline=""))) if p.exists() else []
            odom = read_csv(self.session_dir/"odom.csv")
            scan = read_csv(self.session_dir/"scan.csv")
            if odom:
                xs=[float(r["x"]) for r in odom]; ys=[float(r["y"]) for r in odom]
                stats["odom_x_mean"]=f"{st.fmean(xs):.3f}"; stats["odom_y_mean"]=f"{st.fmean(ys):.3f}"
            if scan:
                ms=[float(r["mean_range"]) for r in scan]; stats["scan_mean"]=f"{st.fmean(ms):.3f}"
        except Exception:
            pass
        return stats

    def _parse_cfg(self, text: str) -> Dict[str,str]:
        cfg={}
        for line in (text or "").splitlines():
            if ":" in line:
                k,v=line.split(":",1); cfg[k.strip()]=v.strip()
        if self.param_speed.get().strip(): cfg["Speed(m/s)"]=self.param_speed.get().strip()
        if self.param_res.get().strip():   cfg["Resolution"]=self.param_res.get().strip()
        if self.param_dur.get().strip():   cfg["Duration(min)"]=self.param_dur.get().strip()
        return cfg

    def _gen_report(self):
        try:
            if not self.session_dir:
                self._append_log("[WARN] No session directory; start session first.")
                return
            ctx = {
                "title":"Drone Mission Report",
                "mission_id": self.session_id or now_id(),
                "generated_at": time.strftime("%Y-%m-%d %H:%M"),
                "summary": self.tb_summary.get("1.0","end").strip(),
                "config": self._parse_cfg(self.tb_cfg.get("1.0","end")),
                "live": {
                    "odom": self.lbl_odom.cget("text").split(": ",1)[-1],
                    "gps":  self.lbl_gps.cget("text").split(": ",1)[-1],
                    "imu":  self.lbl_imu.cget("text").split(": ",1)[-1],
                    "scan": self.lbl_scan.cget("text").split(": ",1)[-1],
                },
                "images": (self.ctx.rec.latest_images(6) if self.ctx else []),
                "stats": self._collect_stats()
            }
            out_dir = Path.cwd()/ "DroneReports" / "sessions" / self.session_id
            pdf = build_pdf(ctx, out_dir)
            messagebox.showinfo("Report", f"Report created:\n{pdf}")
            open_file(pdf)
        except Exception as e:
            messagebox.showerror("Report Error", str(e))

    # ----------------- shutdown ----------------------------------------
    def destroy(self):
        try:
            if getattr(self, "mock", None): self.mock.stop()
            if getattr(self, "ctx", None):  self.ctx.rec.close()
        finally:
            super().destroy()

# ----------------------------- run -----------------------------------------
def main():
    app = HAJEGUI()
    app.mainloop()

if __name__ == "__main__":
    main()