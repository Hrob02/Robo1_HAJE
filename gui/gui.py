"""
The following GUI provides an advanced working prototype for the HAJE Engineering
Fuel Assessment and Risk Tool GUI. There is no dynamic interfacing with simulation or ROS.
This will be setup in the coming weeks with mock missions. To ensure that the integration has been
successful these mock missions should have randomised or unique inputs
"""

#! ----------------------------- Standard Library -----------------------------
import csv
import json
import math
import random
import shlex
import subprocess
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from threading import Thread, Lock

#! ----------------------------- Third-Party Libs -----------------------------
import customtkinter as ctk
from tkinter import messagebox
from jinja2 import Template
from PIL import Image, ImageTk  # for mock camera frames
try:
    import cv2  # optional, for webcam/video in mock mode
except ImportError:
    cv2 = None

# Optional plots in recorder (saved as PNGs if matplotlib is present)
try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None

# ----------------------------- Theme Constants ------------------------------
BG          = "#0F0F11"   # app background
CARD        = "#1B1C1F"   # card surface
SHADOW_DARK = "#0A0A0B"   # soft shadow plate
TEXT        = "#D7DBE0"   # primary text
SUBTEXT     = "#9AA3AD"   # secondary text
ACCENT      = "#E10600"   # red
ACCENT_2    = "#FF3B2E"   # red hover
BORDER      = "#2E3238"   # subtle border

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")  # we override colors per-widget

# ------------------------------ LaTeX Template ------------------------------
LATEX_TEMPLATE = r"""
\documentclass[11pt]{article}
\usepackage[margin=1in]{geometry}
\usepackage{graphicx}
\usepackage{xcolor}
\usepackage{booktabs}
\usepackage{hyperref}
\hypersetup{colorlinks=true, linkcolor=black, urlcolor=blue}

\begin{document}
\begin{center}
  {\LARGE \textbf{ {{ title }} }}\\[6pt]
  {\large Mission ID: {{ mission_id }} \quad | \quad Generated: {{ generated_at }}}
\end{center}
\hrule\vspace{1em}

\section*{Summary of Findings}
{{ summary }}

\section*{Fuel Risk}
{{ fuel_risk }}

\section*{Configuration}
\begin{tabular}{@{}ll@{}}
\toprule
\textbf{Key} & \textbf{Value} \\
\midrule
{% for k, v in config.items() -%}
{{ k }} & {{ v }} \\
{% endfor -%}
\bottomrule
\end{tabular}

\section*{Live Data Snapshots}
\begin{tabular}{@{}ll@{}}
\toprule
\textbf{Metric} & \textbf{Value} \\
\midrule
odom & {{ live.odom }} \\
gps  & {{ live.gps }} \\
imu  & {{ live.imu }} \\
scan & {{ live.scan }} \\
\bottomrule
\end{tabular}

{% if images %}
\section*{Images}
{% for img in images -%}
\begin{figure}[h]
\centering
\includegraphics[width=0.9\linewidth]{ {{ img }} }
\end{figure}
{% endfor -%}
{% endif %}

\section*{Artifacts}
Raw CSVs are saved in: \texttt{ {{ session_dir }} }.

\section*{Logs}
\noindent\texttt{ {{ logs }} }

\end{document}
"""

#! ------------------------------ LaTeX Escaping ------------------------------
#? This section handles the special characters because latex fails without adequate escaping
#* DETERMINE MECHANISM SO CAN EXPLAIN FULLY
def latex_escape(s: str) -> str:
    if not s:
        return ""
    repl = {
        "\\": r"\textbackslash{}",  "&": r"\&",  "%": r"\%", "$": r"\$",
        "#": r"\#",                 "_": r"\_",  "{": r"\{", "}": r"\}",
        "~": r"\textasciitilde{}", "^": r"\textasciicircum{}",
    }
    for k, v in repl.items():
        s = s.replace(k, v)
    return s

#! ---------------------------- Neumorphic Helpers ----------------------------
"""
Neumorphic Helpers manage colouring and presentation of GUI components to mimic shadowing
This must be handled manually as TKinter has less front end flexibility than CSS 
"""
class NeoCard(ctk.CTkFrame):
    def __init__(self, parent, pad=(10, 10), **kwargs):
        super().__init__(parent, fg_color=BG)
        self.padx, self.pady = pad
        self.back = ctk.CTkFrame(self, fg_color=SHADOW_DARK, corner_radius=18)
        self.back.place(relx=0, rely=0, relwidth=1, relheight=1)
        self.card = ctk.CTkFrame(self, fg_color=CARD, corner_radius=18, **kwargs)
        self.card.place(relx=0, rely=0, relwidth=1, relheight=1, x=-1, y=-1)
        self.body = ctk.CTkFrame(self.card, fg_color=CARD, corner_radius=18)
        self.body.pack(expand=True, fill="both", padx=self.padx, pady=self.pady)

def AccentButton(*args, **kwargs) -> ctk.CTkButton:
    return ctk.CTkButton(*args, fg_color=ACCENT, hover_color=ACCENT_2,
                         text_color="white", corner_radius=12, height=44, **kwargs)

#! ---------------------------- Mock ROS “Bus” ----------------------------
#! ---------------------------- TO BE REMOVED  ----------------------------
"""
Mock "Bus" for ROS Connection - THIS MUST BE REMOVED SO THERE ARE NO DISRUPTIONS TO ACTUAL CONNECTION
"""
class MockBus:
    """Feeds synthetic data into GUI update hooks at a fixed rate."""
    def __init__(self, tk_root,
                 on_odom, on_gps, on_imu, on_scan,
                 on_frame1, on_frame2, fps=10):
        self.tk = tk_root
        self.on_odom = on_odom
        self.on_gps = on_gps
        self.on_imu = on_imu
        self.on_scan = on_scan
        self.on_frame1 = on_frame1
        self.on_frame2 = on_frame2
        self.fps = fps
        self._running = False
        self._cap1 = None
        self._cap2 = None

    def start(self, video1: str | None = None, video2: str | None = None):
        self._running = True
        if cv2 and video1:
            self._cap1 = cv2.VideoCapture(0 if video1 == "webcam" else video1)
        if cv2 and video2:
            self._cap2 = cv2.VideoCapture(0 if video2 == "webcam" else video2)
        self._tick()

    def stop(self):
        self._running = False
        if self._cap1: self._cap1.release()
        if self._cap2: self._cap2.release()

    def _pull_frame(self, cap, t) -> Image.Image:
        if cap and cap.isOpened():
            ok, frame = cap.read()
            if ok and frame is not None:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                return Image.fromarray(frame)
        img = Image.new("RGB", (640, 360), (28, 30, 35))
        r = 12
        cx = int(320 + 180*math.cos(t/2))
        cy = int(180 + 100*math.sin(t/2))
        for dx in range(-r, r):
            for dy in range(-r, r):
                if dx*dx + dy*dy <= r*r:
                    x2 = cx+dx; y2 = cy+dy
                    if 0 <= x2 < 640 and 0 <= y2 < 360:
                        img.putpixel((x2, y2), (225, 20, 10))
        return img

    def _tick(self):
        if not self._running:
            return
        t = time.time()
        x = 10.0 * math.cos(t/4.0)
        y = 10.0 * math.sin(t/4.0)
        yaw = (t % (2*math.pi))
        self.on_odom({"t": t, "x": x, "y": y, "yaw": yaw})
        self.on_gps({"t": t, "lat": -33.864 + x*1e-6, "lon": 151.209 + y*1e-6, "alt": 89.3})
        self.on_imu({"t": t, "roll": 0.01*math.sin(t), "pitch": 0.01*math.cos(t), "yaw": yaw})
        self.on_scan({"t": t, "points": 720, "mean_range": 12.3 + 0.5*math.sin(t/3)})
        img1 = self._pull_frame(self._cap1, t)
        img2 = self._pull_frame(self._cap2, t)
        self.on_frame1(img1, t)
        self.on_frame2(img2, t)
        interval_ms = int(1000 / max(1, self.fps))
        self.tk.after(interval_ms, self._tick)

#! ------------------------------ Data Recorder ------------------------------
"""
Thread-safe recorder that saves telemetry to CSV and frames to disk.
One instance per mission/session. This is required for saving data 
"""
class DataRecorder:
    def __init__(self, session_dir: Path, frame_interval_s: float = 0.5, keep_images: int = 50):
        self.dir = Path(session_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        (self.dir / "frames").mkdir(exist_ok=True)
        self._lock = Lock()
        self._frames_dir = self.dir / "frames"
        self._last_frame_save = {"cam1": 0.0, "cam2": 0.0}
        self._frame_interval = frame_interval_s
        self._keep_images = keep_images
        # rolling image lists
        self._images = {"cam1": deque(maxlen=keep_images), "cam2": deque(maxlen=keep_images)}
        # csv files
        self._files = {
            "odom": open(self.dir / "odom.csv", "w", newline=""),
            "gps":  open(self.dir / "gps.csv", "w", newline=""),
            "imu":  open(self.dir / "imu.csv", "w", newline=""),
            "scan": open(self.dir / "scan.csv", "w", newline=""),
        }
        self._writers = {
            "odom": csv.DictWriter(self._files["odom"], fieldnames=["t","x","y","yaw"]),
            "gps":  csv.DictWriter(self._files["gps"],  fieldnames=["t","lat","lon","alt"]),
            "imu":  csv.DictWriter(self._files["imu"],  fieldnames=["t","roll","pitch","yaw"]),
            "scan": csv.DictWriter(self._files["scan"], fieldnames=["t","points","mean_range"]),
        }
        for w in self._writers.values():
            w.writeheader()
        self._meta = {"start_time": time.time()}

    def add(self, stream: str, row: dict):
        with self._lock:
            if stream in self._writers:
                self._writers[stream].writerow(row)

    def save_frame(self, cam: str, pil_img: Image.Image, t: float):
        now = t
        if now - self._last_frame_save.get(cam, 0.0) < self._frame_interval:
            return
        self._last_frame_save[cam] = now
        ts = datetime.fromtimestamp(now).strftime("%H%M%S_%f")[:-3]
        path = self._frames_dir / f"{cam}_{ts}.jpg"
        try:
            pil_img.save(path, format="JPEG", quality=88)
            with self._lock:
                self._images[cam].append(str(path))
        except Exception:
            pass

    def latest_images(self, limit=6):
        with self._lock:
            imgs = list(self._images["cam1"]) + list(self._images["cam2"])
        return imgs[-limit:] if limit else imgs

    def close(self):
        with self._lock:
            for f in self._files.values():
                try: f.flush(); f.close()
                except Exception: pass
            self._meta["end_time"] = time.time()
            try:
                (self.dir / "session_meta.json").write_text(json.dumps(self._meta, indent=2))
            except Exception:
                pass

    def make_quick_plots(self):
        # Optional: save small timeseries PNGs for the report
        if not plt:
            return []
        images = []
        try:
            import pandas as pd  # only used here; skip if not installed
        except Exception:
            return images
        try:
            for name in ["odom","gps","imu","scan"]:
                p = self.dir / f"{name}.csv"
                if not p.exists(): continue
                df = pd.read_csv(p)
                if df.empty: continue
                fig = plt.figure(figsize=(5,2.2))
                if name == "odom":
                    plt.plot(df["t"]-df["t"].iloc[0], df["x"]); plt.plot(df["t"]-df["t"].iloc[0], df["y"])
                    plt.ylabel("m")
                elif name == "gps":
                    plt.plot(df["t"]-df["t"].iloc[0], df["lat"]); plt.plot(df["t"]-df["t"].iloc[0], df["lon"])
                    plt.ylabel("deg")
                elif name == "imu":
                    plt.plot(df["t"]-df["t"].iloc[0], df["roll"]); plt.plot(df["t"]-df["t"].iloc[0], df["pitch"])
                    plt.ylabel("rad")
                else:
                    plt.plot(df["t"]-df["t"].iloc[0], df["mean_range"]); plt.ylabel("m")
                plt.xlabel("s")
                plt.tight_layout()
                out_png = self.dir / f"{name}.png"
                fig.savefig(out_png)
                plt.close(fig)
                images.append(str(out_png))
        except Exception:
            pass
        return images

#! --------------------------------- GUI  ----------------------------------
class HAJEEngineeringGUI(ctk.CTk):
    def __init__(self):
        super().__init__()
        # Window Configuration
        self.title("HAJE Engineering Fuel Assessment Risk Tool")
        self.geometry("1400x850")
        self.minsize(1200, 760)
        self.configure(fg_color=BG)

        # Tabs
        self.tabview = ctk.CTkTabview(self, width=1300, height=800, fg_color=BG)
        self.tabview.pack(padx=16, pady=16, expand=True, fill="both")
        self.tabview._segmented_button.configure(
            fg_color="#191A1D",
            selected_color=ACCENT, selected_hover_color=ACCENT_2,
            unselected_color="#2A2C31", unselected_hover_color="#33363C",
            text_color=TEXT,
        )
        self.launch_tab = self.tabview.add("Launch")
        self.live_tab   = self.tabview.add("Mission Feed")
        self.report_tab = self.tabview.add("Report")

        # Build pages
        self._build_launch_tab()
        self._build_live_tab()
        self._build_report_tab()

        # Session/recorder placeholders (created on Start)
        self.mission_id = None
        self.session_dir = None
        self.recorder: DataRecorder | None = None

        # ---------------- Mock wiring (flip to False when moving to ROS) -----------
        self.mock_mode = True
        if self.mock_mode:
            self.mock = MockBus(
                tk_root=self,
                on_odom=self.on_mock_odom,
                on_gps=self.on_mock_gps,
                on_imu=self.on_mock_imu,
                on_scan=self.on_mock_scan,
                on_frame1=self.on_mock_frame1,
                on_frame2=self.on_mock_frame2,
                fps=10
            )
            self.mock.start(video1=None, video2=None)  # set "webcam" to preview

    # ----------------------------- Launch Tab --------------------------------
    def _build_launch_tab(self):
        for r in range(6): self.launch_tab.grid_rowconfigure(r, weight=0)
        self.launch_tab.grid_rowconfigure(5, weight=1)
        self.launch_tab.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(self.launch_tab, text="1. LAUNCH PAGE",
                     font=("Arial", 26, "bold"), text_color=TEXT)\
            .grid(row=0, column=0, sticky="w", padx=20, pady=(16, 6))

        # START/STOP bar
        bar = NeoCard(self.launch_tab); bar.grid(row=1, column=0, sticky="ew", padx=20, pady=(10, 10))
        bar.body.grid_columnconfigure(2, weight=1)
        self.start_btn = ctk.CTkButton(bar.body, text="START", state="disabled",
                                       fg_color="#3A3B3F", hover_color="#3A3B3F",
                                       text_color="#666B73", corner_radius=12, height=44,
                                       command=self._start_mission, width=160)
        self.stop_btn  = ctk.CTkButton(bar.body, text="STOP", fg_color="#9B1C17",
                                       hover_color="#7B1612", text_color="white",
                                       corner_radius=12, height=44, command=self._stop_mission, width=160)
        self.start_btn.grid(row=0, column=0, padx=(0,10), pady=6, sticky="w")
        self.stop_btn.grid(row=0, column=1, padx=(0,10), pady=6, sticky="w")
        ctk.CTkLabel(bar.body, text="Start greyed out if empty",
                     text_color=SUBTEXT, font=("Arial", 12, "italic"))\
            .grid(row=1, column=0, columnspan=2, sticky="w", pady=(2, 2))

        # Flight procedure
        proc = NeoCard(self.launch_tab); proc.grid(row=2, column=0, sticky="ew", padx=20, pady=(6, 8))
        ctk.CTkLabel(proc.body, text="✓  FLIGHT PROCEDURE",
                     font=("Arial", 18, "bold"), text_color=TEXT)\
            .grid(row=0, column=0, sticky="w", pady=(4, 8))
        self.proc_var = ctk.StringVar(value="")
        ctk.CTkRadioButton(proc.body, text="Standard opp.", value="Standard",
                           variable=self.proc_var, text_color=TEXT,
                           fg_color=ACCENT, hover_color=ACCENT_2,
                           command=self._maybe_enable_start).grid(row=1, column=0, sticky="w", pady=(0,6))
        ctk.CTkRadioButton(proc.body, text="Flight data only.", value="DataOnly",
                           variable=self.proc_var, text_color=TEXT,
                           fg_color=ACCENT, hover_color=ACCENT_2,
                           command=self._maybe_enable_start).grid(row=2, column=0, sticky="w")

        # gps data
        gps = NeoCard(self.launch_tab); gps.grid(row=3, column=0, sticky="ew", padx=20, pady=(6, 10))
        ctk.CTkLabel(gps.body, text="GPS", font=("Arial", 18, "bold"), text_color=TEXT)\
            .grid(row=0, column=0, sticky="w", pady=(2, 6))
        self.gps_entry = ctk.CTkEntry(gps.body, placeholder_text="data...",
                                      height=44, corner_radius=12,
                                      fg_color="#111214", text_color=TEXT,
                                      border_color=BORDER, border_width=2)
        self.gps_entry.grid(row=1, column=0, sticky="ew")
        self.gps_entry.bind("<KeyRelease>", lambda e: self._maybe_enable_start())

    def _maybe_enable_start(self):
        filled = bool(self.gps_entry.get().strip())
        chosen = self.proc_var.get() != ""
        if filled and chosen:
            self.start_btn.configure(state="normal", fg_color=ACCENT, hover_color=ACCENT_2, text_color="white")
        else:
            self.start_btn.configure(state="disabled", fg_color="#3A3B3F", hover_color="#3A3B3F", text_color="#666B73")

    def _start_mission(self):
        # Create a new session folder and start a recorder
        self.mission_id = datetime.now().strftime("%Y%m%d-%H%M%S")
        base = Path.home() / "Documents" / "DroneReports" / "sessions"
        self.session_dir = base / self.mission_id
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.recorder = DataRecorder(self.session_dir)
        messagebox.showinfo("Start", f"Mission Started ✅\nMode: {self.proc_var.get()}\nSession: {self.session_dir}")

    def _stop_mission(self):
        if self.recorder:
            self.recorder.close()
            self.recorder = None
        messagebox.showinfo("Stop", "Mission Stopped ⛔\nRecorder closed.")

    # ------------------------------- Live Tab --------------------------------
    def _build_live_tab(self):
        for c in range(3): self.live_tab.grid_columnconfigure(c, weight=1, uniform="live")
        self.live_tab.grid_rowconfigure(1, weight=1)
        self.live_tab.grid_rowconfigure(2, weight=1)

        ctk.CTkLabel(self.live_tab, text="2. LIVE PAGE",
                     font=("Arial", 26, "bold"), text_color=TEXT)\
            .grid(row=0, column=0, columnspan=3, sticky="w", padx=20, pady=(16, 8))

        # Left: RVIZ + Live Data (big)
        left_top = NeoCard(self.live_tab); left_top.grid(row=1, column=0, sticky="nsew", padx=(20,10), pady=(4,6))
        left_bot = NeoCard(self.live_tab); left_bot.grid(row=2, column=0, sticky="nsew", padx=(20,10), pady=(6,16))

        rv = left_top.body
        rv.grid_rowconfigure(0, weight=1); rv.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(rv, text="RVIZ MAP", font=("Arial", 20, "bold"), text_color=TEXT)\
            .grid(row=0, column=0, sticky="n", pady=(8, 0))
        ctk.CTkLabel(rv, text="(visualization marker(s))", text_color=SUBTEXT)\
            .grid(row=0, column=0, sticky="s", pady=(0, 10))

        ld = left_bot.body
        ctk.CTkLabel(ld, text="Live Data", font=("Arial", 20, "bold"), text_color=TEXT)\
            .grid(row=0, column=0, sticky="w", pady=(2, 6))
        self.data_labels = {
            "odom": ctk.CTkLabel(ld, text="odom: —", text_color=TEXT),
            "gps":  ctk.CTkLabel(ld, text="gps: —",  text_color=TEXT),
            "imu":  ctk.CTkLabel(ld, text="imu: —",  text_color=TEXT),
            "scan": ctk.CTkLabel(ld, text="scan data: —", text_color=TEXT),
        }
        r = 1
        for lbl in self.data_labels.values():
            lbl.grid(row=r, column=0, sticky="w", pady=4, padx=4)
            r += 1

        # Middle: Cameras (image panes)
        mid_top = NeoCard(self.live_tab); mid_top.grid(row=1, column=1, sticky="nsew", padx=10, pady=(4, 6))
        mid_bot = NeoCard(self.live_tab); mid_bot.grid(row=2, column=1, sticky="nsew", padx=10, pady=(6, 16))

        for frame, title_text in [(mid_top.body, "CAM 1"), (mid_bot.body, "CAM 2")]:
            frame.grid_rowconfigure(1, weight=1); frame.grid_columnconfigure(0, weight=1)
            ctk.CTkLabel(frame, text=title_text, font=("Arial", 20, "bold"), text_color=TEXT)\
                .grid(row=0, column=0, sticky="w", pady=(8, 0), padx=10)

        self._cam1_label = ctk.CTkLabel(mid_top.body, text="")
        self._cam1_label.grid(row=1, column=0, sticky="nsew", pady=(6, 10), padx=10)
        self._cam1_imgtk = None

        self._cam2_label = ctk.CTkLabel(mid_bot.body, text="")
        self._cam2_label.grid(row=1, column=0, sticky="nsew", pady=(6, 10), padx=10)
        self._cam2_imgtk = None

        # Right: Key + Controls
        right = NeoCard(self.live_tab); right.grid(row=1, column=2, sticky="nsew", padx=(10,20), pady=(4,6))
        ctrl  = NeoCard(self.live_tab); ctrl.grid(row=2, column=2, sticky="nsew", padx=(10,20), pady=(6,16))

        kb = right.body
        ctk.CTkLabel(kb, text="KEY:", font=("Arial", 20, "bold"), text_color=TEXT)\
            .grid(row=0, column=0, columnspan=2, sticky="w", pady=(2,8))
        def tile(txt, row, col):
            t = ctk.CTkFrame(kb, fg_color="#26282D", corner_radius=12, width=130, height=60)
            t.grid_propagate(False)
            ctk.CTkLabel(t, text=txt, text_color=TEXT).pack(expand=True)
            t.grid(row=row, column=col, padx=8, pady=6, sticky="w")
        tile("Species 1", 1, 0); tile("Species 2", 1, 1)
        tile("Species 3", 2, 0); tile("User visual", 2, 1)

        cc = ctrl.body
        cc.grid_columnconfigure(0, weight=1)
        self.user_input = ctk.CTkEntry(cc, placeholder_text="USER INPUT",
                                       height=44, fg_color="#111214", text_color=TEXT,
                                       border_color=BORDER, border_width=2, corner_radius=12)
        self.user_input.grid(row=0, column=0, sticky="ew", pady=(4, 10))
        AccentButton(cc, text="SUSPEND", command=self._suspend, width=140)\
            .grid(row=1, column=0, sticky="e")

    def _suspend(self):
        messagebox.showinfo("Suspend", "Drone Suspended ⏸")

    #! -------------------------- Mock update hooks + record -------------------
    def _fmt(self, v, n=3):
        try: return f"{float(v):.{n}f}"
        except: return str(v)

    def on_mock_odom(self, d):
        self.data_labels["odom"].configure(
            text=f"odom: x={self._fmt(d['x'])}, y={self._fmt(d['y'])}, yaw={self._fmt(d['yaw'])}"
        )
        if self.recorder: self.recorder.add("odom", d)

    def on_mock_gps(self, d):
        self.data_labels["gps"].configure(
            text=f"gps: lat={self._fmt(d['lat'],6)}, lon={self._fmt(d['lon'],6)}, alt={self._fmt(d['alt'])}m"
        )
        if self.recorder: self.recorder.add("gps", d)

    def on_mock_imu(self, d):
        self.data_labels["imu"].configure(
            text=f"imu: roll={self._fmt(d['roll'])}, pitch={self._fmt(d['pitch'])}, yaw={self._fmt(d['yaw'])}"
        )
        if self.recorder: self.recorder.add("imu", d)

    def on_mock_scan(self, d):
        self.data_labels["scan"].configure(
            text=f"scan data: {d['points']} pts, mean={self._fmt(d['mean_range'])} m"
        )
        if self.recorder: self.recorder.add("scan", d)

    def on_mock_frame1(self, pil_img, t):
        self._cam1_imgtk = ImageTk.PhotoImage(pil_img.resize((640, 360)))
        self._cam1_label.configure(image=self._cam1_imgtk)
        if self.recorder: self.recorder.save_frame("cam1", pil_img, t)

    def on_mock_frame2(self, pil_img, t):
        self._cam2_imgtk = ImageTk.PhotoImage(pil_img.resize((640, 360)))
        self._cam2_label.configure(image=self._cam2_imgtk)
        if self.recorder: self.recorder.save_frame("cam2", pil_img, t)

    # ------------------------------ Report Tab --------------------------------
    def _build_report_tab(self):
        self.report_tab.grid_columnconfigure(0, weight=3, uniform="rep")
        self.report_tab.grid_columnconfigure(1, weight=2, uniform="rep")
        self.report_tab.grid_rowconfigure(1, weight=1)

        ctk.CTkLabel(self.report_tab, text="3. REPORT PAGE",
                     font=("Arial", 26, "bold"), text_color=TEXT)\
            .grid(row=0, column=0, columnspan=2, sticky="w", padx=20, pady=(16, 8))

        # Left: Summary
        left = NeoCard(self.report_tab); left.grid(row=1, column=0, sticky="nsew", padx=(20,10), pady=(6,16))
        left.body.grid_columnconfigure(0, weight=1)
        left.body.grid_rowconfigure(1, weight=1)
        ctk.CTkLabel(left.body, text="Summary of Findings", font=("Arial", 20, "bold"), text_color=TEXT)\
            .grid(row=0, column=0, sticky="w", pady=(2, 6))
        self._summary_tb = ctk.CTkTextbox(left.body, fg_color="#121316", text_color=TEXT,
                                          border_color=BORDER, border_width=2, corner_radius=12)
        self._summary_tb.insert("end", "Gauge/summary viz placeholder →\n\n• Fuel Risk, etc …")
        self._summary_tb.grid(row=1, column=0, sticky="nsew")

        # Right: Config + Logs + Generate
        right = NeoCard(self.report_tab); right.grid(row=1, column=1, sticky="nsew", padx=(10,20), pady=(6,16))
        right.body.grid_columnconfigure(0, weight=1)
        right.body.grid_rowconfigure(3, weight=1)
        ctk.CTkLabel(right.body, text="Configuration", font=("Arial", 20, "bold"), text_color=TEXT)\
            .grid(row=0, column=0, sticky="w", pady=(2, 6))
        self._cfg_tb = ctk.CTkTextbox(right.body, height=120, fg_color="#121316", text_color=TEXT,
                                      border_color=BORDER, border_width=2, corner_radius=12)
        self._cfg_tb.insert("end", "Example:\nMode: Standard\nMax Altitude: 120 m\nPilot: A. Akhurst")
        self._cfg_tb.grid(row=1, column=0, sticky="ew")

        ctk.CTkLabel(right.body, text="Mission Log", font=("Arial", 20, "bold"), text_color=TEXT)\
            .grid(row=2, column=0, sticky="w", pady=(6, 6))
        self._logs_tb = ctk.CTkTextbox(right.body, fg_color="#121316", text_color=TEXT,
                                       border_color=BORDER, border_width=2, corner_radius=12)
        self._logs_tb.insert("end", "System logs / ROS output stream…")
        self._logs_tb.grid(row=3, column=0, sticky="nsew")

        self.gen_btn = ctk.CTkButton(right.body, text="Generate Report",
                                     fg_color=ACCENT, hover_color=ACCENT_2,
                                     corner_radius=12, command=self._generate_report)
        self.gen_btn.grid(row=4, column=0, sticky="e", pady=(8, 0))

    # --------------------------- Report Data Hooks ----------------------------
    def _get_textbox_content_on_report(self, which: int) -> str:
        tbs = [getattr(self, "_summary_tb", None),
               getattr(self, "_cfg_tb", None),
               getattr(self, "_logs_tb", None)]
        tb = tbs[which] if 0 <= which < len(tbs) else None
        return tb.get("1.0", "end").strip() if tb else ""

    def _collect_report_data(self) -> dict:
        summary_text = latex_escape(self._get_textbox_content_on_report(0))
        config_text  = self._get_textbox_content_on_report(1)
        logs_text    = latex_escape(self._get_textbox_content_on_report(2))

        config_pairs = {}
        for line in (config_text or "").splitlines():
            if ":" in line:
                k, v = line.split(":", 1)
                config_pairs[latex_escape(k.strip())] = latex_escape(v.strip())

        # Prefer recorder values if available; else scrape labels
        if self.recorder and self.session_dir:
            # Make quick timeseries plots (optional)
            plot_imgs = self.recorder.make_quick_plots()
            image_paths = self.recorder.latest_images(limit=6) + plot_imgs
        else:
            image_paths = []

        live = {
            "odom": latex_escape(self.data_labels["odom"].cget("text").split(": ", 1)[-1]),
            "gps":  latex_escape(self.data_labels["gps"].cget("text").split(": ", 1)[-1]),
            "imu":  latex_escape(self.data_labels["imu"].cget("text").split(": ", 1)[-1]),
            "scan": latex_escape(self.data_labels["scan"].cget("text").split(": ", 1)[-1]),
        }

        return {
            "title":        "Drone Mission Report",
            "mission_id":   self.mission_id or datetime.now().strftime("%Y%m%d-%H%M%S"),
            "generated_at": datetime.now().strftime("%Y-%m-%d %H:%M"),
            "summary":      summary_text or "—",
            "fuel_risk":    latex_escape("Auto-rated fuel risk and notes…"),
            "config":       config_pairs,
            "live":         live,
            "images":       image_paths,
            "logs":         logs_text or "—",
            "session_dir":  str(self.session_dir) if self.session_dir else "",
        }

    # --------------------------- Report Generation ---------------------------
    def _generate_report(self):
        Thread(target=self._report_worker, daemon=True).start()

    def _report_worker(self):
        try:
            self.gen_btn.configure(state="disabled", text="Generating…")
            data = self._collect_report_data()

            out_dir = Path.home() / "Documents" / "DroneReports"
            out_dir.mkdir(parents=True, exist_ok=True)

            job_id   = data["mission_id"]
            tex_path = out_dir / f"report_{job_id}.tex"
            pdf_path = out_dir / f"report_{job_id}.pdf"

            tex = Template(LATEX_TEMPLATE).render(**data)
            tex_path.write_text(tex, encoding="utf-8")

            try:
                cmd = f"latexmk -pdf -interaction=nonstopmode -halt-on-error -outdir={shlex.quote(str(out_dir))} {shlex.quote(str(tex_path))}"
                subprocess.run(cmd, shell=True, check=True, cwd=str(out_dir))
            except subprocess.CalledProcessError:
                for _ in range(2):
                    cmd = (
                        f"pdflatex -interaction=nonstopmode -halt-on-error "
                        f"-output-directory={shlex.quote(str(out_dir))} {shlex.quote(str(tex_path))}"
                    )
                    subprocess.run(cmd, shell=True, check=True, cwd=str(out_dir))

            if pdf_path.exists():
                messagebox.showinfo("Report", f"PDF generated:\n{pdf_path}")
                try:
                    subprocess.Popen(["xdg-open", str(pdf_path)])
                except Exception:
                    pass
            else:
                messagebox.showerror("Report", "Failed to generate PDF. Check LaTeX logs in the output folder.")
        except Exception as ex:
            messagebox.showerror("Report Error", str(ex))
        finally:
            self.gen_btn.configure(state="normal", text="Generate Report")

    # ------------------------------ Cleanup -----------------------------------
    def destroy(self):
        try:
            if getattr(self, "recorder", None):
                self.recorder.close()
            if getattr(self, "mock", None):
                self.mock.stop()
        finally:
            super().destroy()

# ---------------------------------- Main ------------------------------------
if __name__ == "__main__":
    app = HAJEEngineeringGUI()
    app.mainloop()
