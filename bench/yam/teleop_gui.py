"""Teleop control panel: start/stop the SO-101 -> YAM teleop and watch the
gripper's position and torque live.

Runs so101_teleop.py as a subprocess (the GUI never touches the CAN bus) and
renders its 10Hz telemetry stream. Run from a terminal (see bench/README.md
for the venv setup):

  cd bench/yam && ../.venv/bin/python teleop_gui.py
"""

import os
import queue
import re
import signal
import subprocess
import threading
import tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
# bench/.venv is the one the runbooks build: it has i2rt + the gs_usb patches.
PYTHON = os.path.join(HERE, "..", ".venv", "bin", "python")
TELEOP = os.path.join(HERE, "so101_teleop.py")
CAL_FILE = os.path.join(HERE, "gripper_cal.json")
LEADER_PORT = "/dev/cu.usbmodemXXXXXXXXXXX"  # bench-specific; see bench/local.toml
TMAX = 2.0  # display range for the torque bar, Nm

TLM_RE = re.compile(r"TLM pos=([-\d.]+) goal=([-\d.]+) vel=([-\d.]+) eff=([-\d.]+)")

BG, FG, ACCENT, WARN = "#1e1e28", "#e8e8f0", "#4ec9b0", "#e05555"


class TeleopGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("YAM Teleop")
        root.configure(bg=BG)
        self.proc = None
        self.lines: queue.Queue = queue.Queue()
        try:
            from fastgripper_dm.calstore import load_store
            _g = load_store(CAL_FILE)["grippers"]
            cal = _g.get("yam") or next(iter(_g.values()))
            self.g_open, self.g_closed = cal["open"], cal["closed"]
        except (OSError, KeyError, StopIteration):
            self.g_open, self.g_closed = 0.0, 1.0

        top = tk.Frame(root, bg=BG)
        top.pack(fill="x", padx=12, pady=8)
        self.btn = tk.Button(top, text="START TELEOP", width=16, font=("Menlo", 13, "bold"),
                             command=self.toggle)
        self.btn.pack(side="left")
        tk.Label(top, text="seconds:", bg=BG, fg=FG, font=("Menlo", 11)).pack(side="left", padx=(16, 4))
        self.seconds = tk.Entry(top, width=6, font=("Menlo", 11))
        self.seconds.insert(0, "600")
        self.seconds.pack(side="left")
        # gripper channel off by default while the worm gear is rebuilt;
        # recalibrate (fastgripper-dm calibrate) before re-enabling
        self.gripper_on = tk.BooleanVar(value=False)
        tk.Checkbutton(top, text="gripper", variable=self.gripper_on, bg=BG, fg=FG,
                       selectcolor="#14141c", font=("Menlo", 11)).pack(side="left", padx=(12, 0))
        self.status = tk.Label(top, text="idle", bg=BG, fg=FG, font=("Menlo", 12))
        self.status.pack(side="left", padx=16)

        self.canvas = tk.Canvas(root, width=560, height=170, bg=BG, highlightthickness=0)
        self.canvas.pack(padx=12, pady=4)

        self.log = tk.Text(root, width=78, height=10, bg="#14141c", fg="#9a9ab0",
                           font=("Menlo", 10), state="disabled")
        self.log.pack(padx=12, pady=(4, 12))

        self.pos = self.goal = self.vel = self.eff = None
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.tick()

    # ---- process control -----------------------------------------------
    def toggle(self):
        if self.proc is None:
            secs = self.seconds.get().strip() or "600"
            cmd = [PYTHON, "-u", TELEOP, "--leader_port", LEADER_PORT, "--seconds", secs]
            if not self.gripper_on.get():
                cmd.append("--no_gripper")
            self.proc = subprocess.Popen(
                cmd,
                cwd=HERE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, env={**os.environ, "I2RT_CAN_BUSTYPE": "gs_usb"},
            )
            threading.Thread(target=self.reader, daemon=True).start()
            self.btn.config(text="STOP TELEOP")
            self.status.config(text="starting...", fg=FG)
        else:
            self.proc.send_signal(signal.SIGINT)  # clean shutdown path
            self.status.config(text="stopping...", fg=FG)

    def reader(self):
        proc = self.proc
        for line in proc.stdout:
            self.lines.put(line.rstrip("\n"))
        proc.wait()
        self.lines.put(f"__EXIT__ {proc.returncode}")

    def on_close(self):
        if self.proc is not None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.root.destroy()

    # ---- rendering -------------------------------------------------------
    def tick(self):
        while not self.lines.empty():
            line = self.lines.get()
            m = TLM_RE.search(line)
            if m:
                self.pos, self.goal, self.vel, self.eff = map(float, m.groups())
                continue
            if line.startswith("__EXIT__"):
                self.proc = None
                self.btn.config(text="START TELEOP")
                code = line.split()[1]
                self.status.config(text=f"stopped (exit {code})",
                                   fg=ACCENT if code == "0" else WARN)
                continue
            if line.startswith("INFO"):
                continue
            self.append_log(line)
            if "TELEOP LIVE" in line:
                self.status.config(text="LIVE", fg=ACCENT)
        self.draw()
        self.root.after(50, self.tick)

    def append_log(self, line: str):
        self.log.config(state="normal")
        self.log.insert("end", line + "\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def draw(self):
        cv = self.canvas
        cv.delete("all")
        w = 560

        # position gauge: open .. closed
        cv.create_text(10, 14, text="GRIPPER POSITION", anchor="w", fill=FG, font=("Menlo", 11, "bold"))
        x0, x1, y0, y1 = 10, w - 10, 28, 58
        cv.create_rectangle(x0, y0, x1, y1, outline="#3a3a4a", width=2)
        if self.pos is not None:
            span = self.g_closed - self.g_open
            frac = max(0.0, min(1.0, (self.pos - self.g_open) / span)) if span else 0.0
            gfrac = max(0.0, min(1.0, (self.goal - self.g_open) / span)) if span else 0.0
            cv.create_rectangle(x0 + 2, y0 + 2, x0 + 2 + frac * (x1 - x0 - 4), y1 - 2,
                                fill=ACCENT, outline="")
            gx = x0 + 2 + gfrac * (x1 - x0 - 4)
            cv.create_line(gx, y0 - 4, gx, y1 + 4, fill="#f0c674", width=2)  # goal marker
            cv.create_text(x0, y1 + 12, anchor="w", fill=FG, font=("Menlo", 11),
                           text=f"{self.pos:+.2f} rad   {frac * 100:5.1f}% closed   vel {self.vel:+.1f} rad/s")
        cv.create_text(x0, y0 - 4, text="open", anchor="sw", fill="#7a7a90", font=("Menlo", 9))
        cv.create_text(x1, y0 - 4, text="closed", anchor="se", fill="#7a7a90", font=("Menlo", 9))

        # torque bar: -TMAX .. +TMAX centered
        cv.create_text(10, 96, text="MOTOR TORQUE", anchor="w", fill=FG, font=("Menlo", 11, "bold"))
        y0, y1 = 110, 140
        cv.create_rectangle(x0, y0, x1, y1, outline="#3a3a4a", width=2)
        mid = (x0 + x1) / 2
        cv.create_line(mid, y0, mid, y1, fill="#3a3a4a")
        if self.eff is not None:
            frac = max(-1.0, min(1.0, self.eff / TMAX))
            color = WARN if abs(self.eff) > 0.75 * TMAX else ACCENT
            cv.create_rectangle(mid, y0 + 2, mid + frac * (x1 - x0 - 4) / 2, y1 - 2,
                                fill=color, outline="")
            cv.create_text(x0, y1 + 12, anchor="w", fill=color, font=("Menlo", 12, "bold"),
                           text=f"{self.eff:+.2f} Nm  (cap {TMAX:.1f})")


root = tk.Tk()
TeleopGUI(root)
root.mainloop()
