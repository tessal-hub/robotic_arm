#!/usr/bin/env python3
"""Standalone, hardware-free digital twin for the current 6-axis arm."""

from __future__ import annotations

import argparse
import json
import math
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk


CURRENT = {
    "geometry_mm": {"D1": 139.0, "A2": 138.0, "A3": 88.0, "D4": 125.0, "D6": 45.0, "TOOL": 30.0},
    "theta_offsets_deg": [0.0, -90.0, 0.0, 0.0, 0.0, 0.0],
    "limits_deg": [[-90.0, 90.0], [-90.0, 90.0], [0.0, 90.0], [-75.0, 75.0], [-90.0, 90.0], [-360.0, 360.0]],
    "gear_ratios": [6.0, 20.0, 20.0, 4.0, 3.0, 1.0],
    "step_signs": [1, 1, -1, -1, 1, 1],
    "encoder_signs": [-1, -1, 1, -1, 1, -1],
    "drivers": ["TMC2209", "TMC2209", "TMC2209", "TMC2209", "A4988", "A4988"],
    "microsteps": 16,
}

AXES = ["Base yaw", "Shoulder", "Elbow", "Wrist pan", "J5 revolute", "J6 roll"]
COLORS = ["#64748b", "#38bdf8", "#22c55e", "#16a34a", "#f59e0b", "#f43f5e"]
DRAW_PLANE_Z_MM = 20.0


def matmul(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(4)) for c in range(4)] for r in range(4)]


def mdh(a, alpha_deg, d, theta_deg):
    al, th = math.radians(alpha_deg), math.radians(theta_deg)
    ca, sa, ct, st = math.cos(al), math.sin(al), math.cos(th), math.sin(th)
    return [[ct, -st, 0, a], [st * ca, ct * ca, -sa, -d * sa],
            [st * sa, ct * sa, ca, d * ca], [0, 0, 0, 1]]


def transform_point(t, p):
    return tuple(sum(t[r][k] * p[k] for k in range(3)) + t[r][3] for r in range(3))


def forward(config, angles):
    g = config["geometry_mm"]
    th = [angles[i] + config["theta_offsets_deg"][i] for i in range(6)]
    rows = [(0, 0, g["D1"]), (0, -90, 0), (g["A2"], 0, 0),
            (g["A3"], -90, g["D4"]), (0, 90, 0), (0, -90, g["D6"])]
    t = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    points = [(0.0, 0.0, 0.0)]
    for i, (a, alpha, d) in enumerate(rows):
        t = matmul(t, mdh(a, alpha, d, th[i]))
        points.append(transform_point(t, (0, 0, 0)))
        if i == 2:  # Visible A3 bend before the D4 offset.
            points.append(transform_point(t, (g["A3"], 0, 0)))
    points.append(transform_point(t, (0, 0, g["TOOL"])))
    return points


def ik_pen_down(config, x, y, z, j6_deg=0.0):
    g, limits = config["geometry_mm"], config["limits_deg"]
    tool = g["D6"] + g["TOOL"]
    fore = math.hypot(g["A3"], g["D4"])
    delta = math.atan2(g["D4"], g["A3"])
    radius, height = math.hypot(x, y), z + tool - g["D1"]
    distance = math.hypot(radius, height)
    if distance > g["A2"] + fore or distance < abs(g["A2"] - fore) or distance == 0:
        return None
    cosine = max(-1.0, min(1.0, (g["A2"] ** 2 + distance ** 2 - fore ** 2) / (2 * g["A2"] * distance)))
    beta, gamma = math.acos(cosine), math.atan2(height, radius)
    candidates = []
    for sign in (1, -1):
        phi2 = gamma + sign * beta
        t2 = -phi2
        q23 = -math.atan2(height - g["A2"] * math.sin(phi2), radius - g["A2"] * math.cos(phi2)) - delta
        angles = [math.degrees(math.atan2(y, x)),
                  math.degrees(t2) - config["theta_offsets_deg"][1],
                  math.degrees(q23 - t2), 0.0,
                  -math.degrees(q23) - config["theta_offsets_deg"][4], j6_deg]
        if all(limits[i][0] <= angles[i] <= limits[i][1] for i in (0, 1, 2, 4)):
            candidates.append(angles)
    return min(candidates, key=lambda a: abs(a[2])) if candidates else None


def sample_segment(a, b, step=4.0):
    count = max(1, math.ceil(math.dist(a, b) / step))
    return [tuple(a[k] + (b[k] - a[k]) * i / count for k in range(3)) for i in range(count + 1)]


def drawing_path(kind, x, y, z, size):
    if kind == "Line":
        strokes = [((x, y, z), (x + size, y, z))]
    if kind == "Circle":
        radius, count = size / 2, max(24, math.ceil(math.pi * size / 4))
        circle = [(x + radius + radius * math.cos(2 * math.pi * i / count),
                   y + radius * math.sin(2 * math.pi * i / count), z) for i in range(count + 1)]
        return with_pen_lifts([circle])
    if kind == "Line":
        return with_pen_lifts([sample_segment(*strokes[0])])
    glyph_strokes = [
        ((0, 0), (0, 1.6)), ((1, 0), (1, 1.6)), ((0, .8), (1, .8)),
        ((1.35, 0), (1.35, 1.6)), ((1.35, 1.6), (2.35, 1.6)),
        ((1.35, .8), (2.15, .8)), ((1.35, 0), (2.35, 0)),
        ((2.7, 1.6), (2.7, 0)), ((2.7, 0), (3.7, 0)),
        ((4.05, 1.6), (4.05, 0)), ((4.05, 0), (5.05, 0)),
        ((5.4, 0), (5.4, 1.6)), ((5.4, 1.6), (6.4, 1.6)),
        ((6.4, 1.6), (6.4, 0)), ((6.4, 0), (5.4, 0)),
    ]
    unit = size / 6.4
    strokes = []
    for start, end in glyph_strokes:
        strokes.append(sample_segment((x + start[0] * unit, y + start[1] * unit, z),
                                      (x + end[0] * unit, y + end[1] * unit, z)))
    return with_pen_lifts(strokes)


def with_pen_lifts(strokes, lift=5.0):
    result = []
    previous = None
    for points in strokes:
        start, end = points[0], points[-1]
        start_lift = (start[0], start[1], start[2] + lift)
        if previous is not None:
            previous_lift = (previous[0], previous[1], previous[2] + lift)
            result.extend((point, False) for point in sample_segment(previous_lift, start_lift))
        else:
            result.append((start_lift, False))
        result.append((start, False))
        result.extend((point, True) for point in points[1:])
        result.append(((end[0], end[1], end[2] + lift), False))
        previous = end
    return result


def validate(config):
    if any(not math.isfinite(float(v)) or float(v) <= 0 for v in config["geometry_mm"].values()):
        raise ValueError("All geometry lengths must be positive numbers.")
    if len(config["theta_offsets_deg"]) != 6 or len(config["limits_deg"]) != 6:
        raise ValueError("Offsets and limits must contain exactly six joints.")
    for i, (low, high) in enumerate(config["limits_deg"]):
        if not math.isfinite(float(low)) or not math.isfinite(float(high)) or low >= high:
            raise ValueError(f"J{i + 1}: minimum must be below maximum.")


class TwinApp:
    def __init__(self, root):
        self.root = root
        self.config = json.loads(json.dumps(CURRENT))
        self.angles = [0.0] * 6
        self.azimuth, self.elevation = -42.0, 24.0
        self.drag = None
        self.geometry_vars = {k: tk.StringVar(value=str(v)) for k, v in self.config["geometry_mm"].items()}
        self.offset_vars = [tk.StringVar(value=str(v)) for v in self.config["theta_offsets_deg"]]
        self.limit_vars = [[tk.StringVar(value=str(x)) for x in pair] for pair in self.config["limits_deg"]]
        self.angle_vars = [tk.DoubleVar(value=0) for _ in range(6)]
        self.view_var = tk.StringVar(value="3D")
        self.status = tk.StringVar(value="Standalone simulation — no hardware connection")
        self.draw_kind = tk.StringVar(value="Line")
        self.draw_vars = {"Start X": tk.StringVar(value="112.5"), "Start Y": tk.StringVar(value="0"),
                          "Plane Z": tk.StringVar(value=str(DRAW_PLANE_Z_MM)), "Size": tk.StringVar(value="95")}
        self.draw_path, self.draw_frames, self.animation_job, self.animation_index = [], [], None, 0
        self._style()
        self._build()
        self.redraw()

    def _style(self):
        self.root.title("NEMA 6-Axis Arm — Offline Digital Twin")
        self.root.geometry("1280x820")
        self.root.minsize(980, 650)
        self.root.configure(bg="#090d16")
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", background="#111827", foreground="#e5e7eb", fieldbackground="#0b1220")
        style.configure("TFrame", background="#090d16")
        style.configure("Card.TFrame", background="#111827")
        style.configure("TLabel", background="#111827", foreground="#cbd5e1")
        style.configure("Title.TLabel", font=("Segoe UI", 16, "bold"), foreground="#f8fafc")
        style.configure("Hint.TLabel", foreground="#94a3b8")
        style.configure("TButton", padding=7)
        style.configure("TNotebook", background="#090d16", borderwidth=0)
        style.configure("TNotebook.Tab", background="#172033", foreground="#cbd5e1", padding=(14, 9))
        style.map("TNotebook.Tab", background=[("selected", "#38bdf8")], foreground=[("selected", "#07111f")])
        style.configure("TEntry", padding=6, fieldbackground="#0b1220", foreground="#f8fafc")
        style.configure("TCombobox", padding=5, fieldbackground="#0b1220", foreground="#f8fafc")
        style.configure("Accent.TButton", background="#0ea5e9", foreground="#07111f", font=("Segoe UI", 9, "bold"))
        style.map("Accent.TButton", background=[("active", "#38bdf8")])

    def _build(self):
        header = ttk.Frame(self.root)
        header.pack(fill="x", padx=14, pady=(12, 6))
        title = ttk.Frame(header)
        title.pack(side="left")
        ttk.Label(title, text="Offline Digital Twin", style="Title.TLabel", background="#090d16").pack(anchor="w")
        ttk.Label(title, text="Inspect motion and drawing reachability without connecting the robot", style="Hint.TLabel", background="#090d16").pack(anchor="w")
        ttk.Label(header, textvariable=self.status, style="Hint.TLabel", background="#090d16").pack(side="right")

        body = ttk.Panedwindow(self.root, orient="horizontal")
        body.pack(fill="both", expand=True, padx=14, pady=6)
        left, right = ttk.Frame(body, style="Card.TFrame"), ttk.Frame(body, style="Card.TFrame")
        body.add(left, weight=3)
        body.add(right, weight=2)

        toolbar = ttk.Frame(left, style="Card.TFrame")
        toolbar.pack(fill="x", padx=10, pady=8)
        ttk.Label(toolbar, text="View").pack(side="left")
        ttk.Combobox(toolbar, textvariable=self.view_var, values=("3D", "Side XZ", "Top XY"), width=10, state="readonly").pack(side="left", padx=6)
        self.view_var.trace_add("write", lambda *_: self.redraw())
        ttk.Button(toolbar, text="Home pose", command=self.home).pack(side="left", padx=4)
        ttk.Button(toolbar, text="Fit", command=self.redraw).pack(side="left", padx=4)
        ttk.Label(toolbar, text="Drag the 3D view to orbit", style="Hint.TLabel").pack(side="right")

        self.canvas = tk.Canvas(left, bg="#070b12", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self.canvas.bind("<Configure>", lambda _e: self.redraw())
        self.canvas.bind("<ButtonPress-1>", lambda e: setattr(self, "drag", (e.x, e.y)))
        self.canvas.bind("<B1-Motion>", self.orbit)

        book = ttk.Notebook(right)
        book.pack(fill="both", expand=True, padx=8, pady=8)
        joints, drawing, geometry, config = (ttk.Frame(book, style="Card.TFrame") for _ in range(4))
        book.add(joints, text="Joints")
        book.add(drawing, text="Drawing")
        book.add(geometry, text="Geometry & limits")
        book.add(config, text="Drivetrain")
        self._build_joints(joints)
        self._build_drawing(drawing)
        self._build_geometry(geometry)
        self._build_drivetrain(config)

        bottom = ttk.Frame(self.root)
        bottom.pack(fill="x", padx=14, pady=(0, 12))
        self.tcp_label = ttk.Label(bottom, text="TCP", background="#090d16")
        self.tcp_label.pack(side="left")
        ttk.Button(bottom, text="Load JSON", command=self.load_json).pack(side="right", padx=4)
        ttk.Button(bottom, text="Save JSON", command=self.save_json).pack(side="right", padx=4)

    def _build_joints(self, parent):
        for i, name in enumerate(AXES):
            row = ttk.Frame(parent, style="Card.TFrame")
            row.pack(fill="x", padx=10, pady=8)
            ttk.Label(row, text=f"J{i + 1}  {name}", width=18).pack(side="left")
            low, high = self.config["limits_deg"][i]
            scale = ttk.Scale(row, from_=low, to=high, variable=self.angle_vars[i], command=lambda v, n=i: self.set_angle(n, v))
            scale.pack(side="left", fill="x", expand=True, padx=6)
            label = ttk.Label(row, width=8, anchor="e")
            label.pack(side="right")
            self.angle_vars[i].trace_add("write", lambda *_args, n=i, out=label: out.configure(text=f"{self.angle_vars[n].get():.1f}°"))
        self.joint_scales = [w for w in parent.winfo_children() for w in w.winfo_children() if isinstance(w, ttk.Scale)]

    def _build_geometry(self, parent):
        form = ttk.Frame(parent, style="Card.TFrame")
        form.pack(fill="x", padx=10, pady=8)
        for row, (key, var) in enumerate(self.geometry_vars.items()):
            ttk.Label(form, text=f"{key} (mm)").grid(row=row, column=0, sticky="w", pady=3)
            ttk.Entry(form, textvariable=var, width=12).grid(row=row, column=1, sticky="ew", padx=6)
        base = len(self.geometry_vars)
        for i in range(6):
            ttk.Label(form, text=f"J{i + 1} offset / min / max (deg)").grid(row=base + i, column=0, sticky="w", pady=3)
            box = ttk.Frame(form, style="Card.TFrame")
            box.grid(row=base + i, column=1, sticky="ew")
            ttk.Entry(box, textvariable=self.offset_vars[i], width=7).pack(side="left", padx=2)
            ttk.Entry(box, textvariable=self.limit_vars[i][0], width=7).pack(side="left", padx=2)
            ttk.Entry(box, textvariable=self.limit_vars[i][1], width=7).pack(side="left", padx=2)
        form.columnconfigure(1, weight=1)
        buttons = ttk.Frame(parent, style="Card.TFrame")
        buttons.pack(fill="x", padx=10, pady=8)
        ttk.Button(buttons, text="Apply to twin", command=self.apply_config).pack(side="left")
        ttk.Button(buttons, text="Reset current arm", command=self.reset_config).pack(side="left", padx=6)
        ttk.Label(parent, text="Simulation only. These values never reach firmware or motors.", style="Hint.TLabel", wraplength=360).pack(fill="x", padx=10, pady=8)

    def _build_drawing(self, parent):
        ttk.Label(parent, text="Simulate a complete tool path", font=("Segoe UI", 12, "bold")).pack(anchor="w", padx=12, pady=(14, 4))
        ttk.Label(parent, text="The path is solved with the same pen-down IK and current joint limits.", style="Hint.TLabel", wraplength=380).pack(anchor="w", padx=12, pady=(0, 12))
        shapes = ttk.Frame(parent, style="Card.TFrame")
        shapes.pack(fill="x", padx=10)
        for kind in ("Line", "Circle", "HELLO"):
            ttk.Radiobutton(shapes, text=kind, value=kind, variable=self.draw_kind).pack(side="left", padx=(0, 12))
        form = ttk.Frame(parent, style="Card.TFrame")
        form.pack(fill="x", padx=10, pady=14)
        for row, (name, var) in enumerate(self.draw_vars.items()):
            ttk.Label(form, text=f"{name} (mm)").grid(row=row, column=0, sticky="w", pady=4)
            ttk.Entry(form, textvariable=var, state="readonly" if name == "Plane Z" else "normal").grid(row=row, column=1, sticky="ew", padx=(12, 0), pady=4)
        form.columnconfigure(1, weight=1)
        actions = ttk.Frame(parent, style="Card.TFrame")
        actions.pack(fill="x", padx=10)
        ttk.Button(actions, text="Preview path", command=self.preview_drawing).pack(side="left")
        ttk.Button(actions, text="Play", style="Accent.TButton", command=self.play_drawing).pack(side="left", padx=6)
        ttk.Button(actions, text="Stop", command=self.stop_animation).pack(side="left")
        self.draw_status = ttk.Label(parent, text="Choose a shape, then Preview path.", style="Hint.TLabel", wraplength=380)
        self.draw_status.pack(fill="x", padx=10, pady=14)

    def _build_drivetrain(self, parent):
        columns = ("joint", "ratio", "step", "encoder", "driver")
        tree = ttk.Treeview(parent, columns=columns, show="headings", height=6)
        for key, title, width in zip(columns, ("Joint", "Ratio", "Step", "Encoder", "Driver"), (55, 60, 55, 70, 90)):
            tree.heading(key, text=title)
            tree.column(key, width=width, anchor="center")
        for i in range(6):
            tree.insert("", "end", values=(f"J{i+1}", f"{self.config['gear_ratios'][i]}:1", f"{self.config['step_signs'][i]:+d}", f"{self.config['encoder_signs'][i]:+d}", self.config["drivers"][i]))
        tree.pack(fill="x", padx=10, pady=10)
        ttk.Label(parent, text=f"Microsteps: 1/{self.config['microsteps']}\nJ1–J4: shared TMC2209 UART\nJ5–J6: independent STEP/DIR + AS5600", justify="left").pack(anchor="w", padx=10, pady=8)

    def set_angle(self, axis, value):
        self.angles[axis] = float(value)
        self.redraw()

    def preview_drawing(self):
        self.stop_animation()
        try:
            values = {key: float(var.get()) for key, var in self.draw_vars.items()}
            if not all(math.isfinite(v) for v in values.values()) or values["Size"] <= 0:
                raise ValueError("X, Y, Z must be finite and Size must be positive.")
            self.draw_path = drawing_path(self.draw_kind.get(), values["Start X"], values["Start Y"], DRAW_PLANE_Z_MM, values["Size"])
            self.draw_frames = []
            held_j6 = self.angle_vars[5].get()
            for index, (point, drawing) in enumerate(self.draw_path):
                angles = ik_pen_down(self.config, *point, j6_deg=held_j6)
                if angles is None:
                    raise ValueError(f"Path is outside reach at point {index + 1}: ({point[0]:.1f}, {point[1]:.1f}, {point[2]:.1f})")
                self.draw_frames.append((angles, drawing))
            self.draw_status.configure(text=f"READY · {self.draw_kind.get()} · {len(self.draw_frames)} IK-safe samples · J6 HOLD {held_j6:.1f}°", foreground="#86efac")
            self.status.set("Drawing preview ready — hardware unchanged")
            self.redraw()
            return True
        except ValueError as exc:
            self.draw_path, self.draw_frames = [], []
            self.draw_status.configure(text=f"OUT OF REACH · {exc}", foreground="#fca5a5")
            self.redraw()
            return False

    def play_drawing(self):
        if not self.preview_drawing():
            return
        self.animation_index = 0
        self._animate_next()

    def _animate_next(self):
        if self.animation_index >= len(self.draw_frames):
            self.animation_job = None
            self.draw_status.configure(text=f"COMPLETE · {self.draw_kind.get()}", foreground="#86efac")
            return
        angles, _drawing = self.draw_frames[self.animation_index]
        for var, angle in zip(self.angle_vars, angles):
            var.set(angle)
        self.redraw()
        self.animation_index += 1
        self.animation_job = self.root.after(35, self._animate_next)

    def stop_animation(self):
        if self.animation_job is not None:
            self.root.after_cancel(self.animation_job)
            self.animation_job = None

    def home(self):
        for var in self.angle_vars:
            var.set(0.0)
        self.angles = [0.0] * 6
        self.redraw()

    def orbit(self, event):
        if self.drag and self.view_var.get() == "3D":
            self.azimuth += (event.x - self.drag[0]) * 0.5
            self.elevation = max(-80, min(80, self.elevation - (event.y - self.drag[1]) * 0.5))
            self.drag = (event.x, event.y)
            self.redraw()

    def apply_config(self):
        try:
            candidate = json.loads(json.dumps(self.config))
            candidate["geometry_mm"] = {k: float(v.get()) for k, v in self.geometry_vars.items()}
            candidate["theta_offsets_deg"] = [float(v.get()) for v in self.offset_vars]
            candidate["limits_deg"] = [[float(v.get()) for v in pair] for pair in self.limit_vars]
            validate(candidate)
            self.config = candidate
            desired = [var.get() for var in self.angle_vars]
            for i, scale in enumerate(self.joint_scales):
                low, high = self.config["limits_deg"][i]
                scale.configure(from_=low, to=high)
                self.angle_vars[i].set(max(low, min(high, desired[i])))
            self.status.set("Twin configuration applied — hardware unchanged")
            self.redraw()
            return True
        except (ValueError, TypeError) as exc:
            messagebox.showerror("Invalid twin configuration", str(exc))
            return False

    def reset_config(self):
        self.config = json.loads(json.dumps(CURRENT))
        for key, value in self.config["geometry_mm"].items():
            self.geometry_vars[key].set(str(value))
        for i in range(6):
            self.offset_vars[i].set(str(self.config["theta_offsets_deg"][i]))
            for j in range(2):
                self.limit_vars[i][j].set(str(self.config["limits_deg"][i][j]))
        self.apply_config()

    def save_json(self):
        if not self.apply_config():
            return
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")], initialfile="arm_twin_config.json")
        if path:
            Path(path).write_text(json.dumps(self.config, indent=2), encoding="utf-8")

    def load_json(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not path:
            return
        try:
            loaded = json.loads(Path(path).read_text(encoding="utf-8"))
            validate(loaded)
            self.config = loaded
            for key in self.geometry_vars:
                self.geometry_vars[key].set(str(loaded["geometry_mm"][key]))
            for i in range(6):
                self.offset_vars[i].set(str(loaded["theta_offsets_deg"][i]))
                for j in range(2):
                    self.limit_vars[i][j].set(str(loaded["limits_deg"][i][j]))
            self.apply_config()
        except (OSError, KeyError, ValueError, TypeError, json.JSONDecodeError) as exc:
            messagebox.showerror("Cannot load configuration", str(exc))

    def project(self, p, scale, cx, cy):
        x, y, z = p
        view = self.view_var.get()
        if view == "Side XZ":
            return cx + x * scale, cy - z * scale
        if view == "Top XY":
            return cx + x * scale, cy - y * scale
        az, el = math.radians(self.azimuth), math.radians(self.elevation)
        xr, yr = x * math.cos(az) - y * math.sin(az), x * math.sin(az) + y * math.cos(az)
        zr, depth = z * math.cos(el) - yr * math.sin(el), z * math.sin(el) + yr * math.cos(el)
        return cx + xr * scale, cy - zr * scale + depth * 0.08 * scale

    def redraw(self):
        if not hasattr(self, "canvas"):
            return
        self.angles = [v.get() for v in self.angle_vars]
        points = forward(self.config, self.angles)
        c, w, h = self.canvas, max(1, self.canvas.winfo_width()), max(1, self.canvas.winfo_height())
        c.delete("all")
        extent = max(350.0, max(abs(v) for point in points for v in point) * 1.2)
        scale, cx, cy = min(w, h) / (extent * 2.1), w / 2, h * 0.72
        for radius in (100, 200, 300):
            box = [self.project((x, y, 0), scale, cx, cy) for x, y in ((-radius, -radius), (radius, -radius), (radius, radius), (-radius, radius))]
            c.create_line(*sum(box + [box[0]], ()), fill="#172033", width=1)
        if self.draw_path:
            projected = [(self.project(point, scale, cx, cy), drawing) for point, drawing in self.draw_path]
            for i in range(1, len(projected)):
                if projected[i][1]:
                    c.create_line(*projected[i - 1][0], *projected[i][0], fill="#f472b6", width=3)
        screen = [self.project(p, scale, cx, cy) for p in points]
        for i in range(len(screen) - 1):
            c.create_line(*screen[i], *screen[i + 1], fill=COLORS[min(i, 5)], width=8, capstyle="round")
        for i, ((x, y), p) in enumerate(zip(screen, points)):
            c.create_oval(x - 6, y - 6, x + 6, y + 6, fill="#f59e0b", outline="#090d16", width=2)
            labels = ("Base", "J1", "J2", "J3", "Forearm bend", "J4", "J5", "J6", "TCP")
            c.create_text(x + 9, y - 10, text=labels[i], fill="#cbd5e1", anchor="sw")
        tcp = points[-1]
        self.tcp_label.configure(text=f"TCP  X {tcp[0]:.1f} mm   Y {tcp[1]:.1f} mm   Z {tcp[2]:.1f} mm")


def self_test():
    validate(CURRENT)
    points = forward(CURRENT, [0.0] * 6)
    tcp = points[-1]
    assert all(abs(a - b) < 1e-6 for a, b in zip(tcp, (200.0, 0.0, 365.0))), tcp
    for kind in ("Line", "Circle", "HELLO"):
        path = drawing_path(kind, 112.5, 0, DRAW_PLANE_Z_MM, 80)
        assert path and all(ik_pen_down(CURRENT, *point) is not None for point, _drawing in path), kind
    assert ik_pen_down(CURRENT, 160, 0, DRAW_PLANE_Z_MM, j6_deg=37.0)[5] == 37.0
    bad = json.loads(json.dumps(CURRENT))
    bad["limits_deg"][0] = [10, -10]
    try:
        validate(bad)
    except ValueError:
        return
    raise AssertionError("invalid limits were accepted")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        print("digital twin self-test: PASS")
        return
    root = tk.Tk()
    TwinApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
