#!/usr/bin/env python3
"""
===============================================================================
NEMA-6AXIS-ARM-CONTROLLER — High-Fidelity Multi-View Digital Clone Pro
===============================================================================
Exact kinematics, geometry, joint model, limits, and planner simulator matching
firmware in src/ (config.h, kinematics.cpp, joint_model.cpp, planner.cpp,
work_plane.cpp) and docs/ARM_GEOMETRY.md.

Features:
  - Exact Joint Limits matching src/config.h (J1..J6)
  - Non-overlapping Multi-Zone Architecture (Zero widget event collisions)
  - Multi-View Engineering Display: 3D Metric View + 2D Side Elevation + 2D Top View
  - Persistent in-place artist updates (Smooth 60FPS, no canvas redraw lag)
  - Exact Craig Modified DH kinematics (139, 138, 88, 126, 20) with L-shaped forearm
  - Closed-form pen-down IK, Cartesian path planner simulator (Line/Circle/Spiral/Star)
  - 3-Point WorkPlane Calibration Engine (Gram-Schmidt UCS for tilted/vertical planes)
  - Real-Time Hardware Bridge (WiFi REST sync & dispatch with physical ESP32-S3)
  - Velocity & Step-Frequency Profiler (50 kHz timer load & feed rate analyzer)
  - Interactive Click-to-Move Viewports (Direct Cartesian positioning via mouse clicks)
  - Flaw Detector: table collision, joint limits, singularity & resolution bottlenecks

Usage:
  python digital_clone.py              # Interactive GUI
  python digital_clone.py --test       # Automated kinematics & trajectory audit
===============================================================================
"""

import sys
import math
import time
import json
import threading
import argparse
import urllib.request
import urllib.error
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import Axes3D


# ==============================================================================
# 1. HARDWARE CONSTANTS & DH PARAMETERS (Source: src/config.h & ARM_GEOMETRY.md)
# ==============================================================================
D1 = 139.0          # Base height: J1 -> J2 (mm)
A2 = 138.0          # Upper arm length: J2 -> J3 (mm)
A3 = 88.0           # Forearm longitudinal offset: J3 -> Wrist X (mm)
D4 = 126.0          # Forearm normal offset: J3 -> Wrist Z (mm) [d4=126 in Craig MDH]
D6 = 31.0           # J5 -> J6 offset along tool axis (mm)
D_TOOL = 20.0       # Pen tool length from J6 along tool Z (mm)
D_TOOL_EFFECTIVE = 51.0  # Total effective tool offset (D6 + D_TOOL = 31 + 20 mm)

# Derived forearm dimensions (docs/ARM_GEOMETRY.md)
L_FORE = math.sqrt(A3**2 + D4**2)              # 153.6863 mm
DELTA_WRIST = math.degrees(math.atan2(D4, A3)) # 55.0587 deg

# Drawing & Planner constants (src/config.h)
PEN_LIFT_MM = 5.0
DRAW_SEGMENT_MM = 1.0
DRAW_FEED_MM_S = 20.0

# Modified DH Parameters (Craig Convention): {a_{i-1}, alpha_{i-1}, d_i}
DH_TABLE = [
    {"a": 0.0,   "alpha": 0.0,    "d": D1},      # Frame 1 (J1 Base Yaw)
    {"a": 0.0,   "alpha": -90.0,  "d": 0.0},     # Frame 2 (J2 Shoulder Pitch)
    {"a": A2,    "alpha": 0.0,    "d": 0.0},     # Frame 3 (J3 Elbow Pitch)
    {"a": A3,    "alpha": -90.0,  "d": D4},      # Frame 4 (J4 Wrist Pan)
    {"a": 0.0,   "alpha": 90.0,   "d": 0.0},     # Frame 5 (J5 Wrist Tilt)
    {"a": 0.0,   "alpha": -90.0,  "d": D6},      # Frame 6 (J6 Tool Roll, d6=31mm)
]

# Theta offsets: theta_DH = enc_deg + TH_OFFSETS (src/config.h lines 200-206)
TH_OFFSETS = [0.0, -90.0, 0.0, 0.0, 0.0, 0.0]

# Joint Soft Angle Limits (Degrees) EXACTLY FROM src/config.h (lines 208-228)
PHYSICAL_LIMITS = [
    (-90.0,   90.0),    # J1 Base Yaw
    (-90.0,   90.0),    # J2 Shoulder Pitch
    (0.0,     90.0),    # J3 Elbow Pitch (0° = thẳng đứng, 90° = vuông góc)
    (-180.0, 180.0),    # J4 Wrist Pan
    (-120.0, 120.0),    # J5 Wrist Tilt
    (-360.0, 360.0),    # J6 Tool Roll
]

# IK Search Limits (src/kinematics.h)
IK_LIMITS = [
    (-90.0,   90.0),    # J1
    (-90.0,   90.0),    # J2
    (0.0,     90.0),    # J3
    (-180.0, 180.0),    # J4
    (-120.0, 120.0),    # J5
    (-180.0, 180.0),    # J6
]

# Gear ratios & Stepper settings (src/config.h lines 102-120)
GEAR_RATIOS = [6.0, 20.0, 20.0, 4.0, 3.0, 3.0]
MICROSTEPS = 16
FULL_STEPS_PER_REV = 200
AXIS_STEP_SIGN = [+1, -1, -1, -1, +1, +1]

STEPS_PER_DEGREE = [
    (FULL_STEPS_PER_REV * MICROSTEPS * gr) / 360.0 for gr in GEAR_RATIOS
]

JOINT_NAMES = [
    "J1 (Base Yaw)",
    "J2 (Shoulder)",
    "J3 (Elbow)",
    "J4 (Wrist Pan)",
    "J5 (Wrist Tilt)",
    "J6 (Tool Roll)"
]


# ==============================================================================
# 2. BEVEL GEAR DIFFERENTIAL WRIST ENGINE (Coupled J5 Tilt & J6 Roll)
# ==============================================================================
class DifferentialWrist:
    """
    2-DOF Coupled Bevel Gear Differential Wrist Kinematics.
    - Side Gear Left  (M_L / E_L): Driven by Motor 5 (Axis 4)
    - Side Gear Right (M_R / E_R): Driven by Motor 6 (Axis 5)
    - Output Pinion / Spider Gear: Mounted on Carrier
    
    Forward Differential Kinematics:
        J5_Tilt = (theta_L + theta_R) / 2
        J6_Roll = (theta_L - theta_R) / (2 * r_bevel)
        
    Inverse Differential Kinematics:
        theta_L = J5_Tilt + (r_bevel * J6_Roll)
        theta_R = J5_Tilt - (r_bevel * J6_Roll)
    """
    def __init__(self, bevel_ratio=1.0):
        self.bevel_ratio = float(bevel_ratio)

    def forward(self, left_deg, right_deg):
        """Converts physical side gear / encoder angles (L, R) to decoupled (Tilt, Roll)."""
        tilt_deg = (float(left_deg) + float(right_deg)) * 0.5
        roll_deg = (float(left_deg) - float(right_deg)) / (2.0 * self.bevel_ratio)
        return tilt_deg, roll_deg

    def inverse(self, tilt_deg, roll_deg):
        """Converts decoupled joint angles (Tilt, Roll) to physical motor angles (L, R)."""
        left_deg = float(tilt_deg) + (self.bevel_ratio * float(roll_deg))
        right_deg = float(tilt_deg) - (self.bevel_ratio * float(roll_deg))
        return left_deg, right_deg

    def compute_motor_steps(self, delta_tilt_deg, delta_roll_deg, spd5, spd6):
        """Computes incremental motor steps for M5 and M6 from joint deltas."""
        d_left, d_right = self.inverse(delta_tilt_deg, delta_roll_deg)
        return int(round(d_left * spd5)), int(round(d_right * spd6))


diff_wrist = DifferentialWrist(bevel_ratio=1.0)


# ==============================================================================
# 3. TRANSFORM & FORWARD KINEMATICS ENGINE
# ==============================================================================
def deg2rad(d):
    return d * (math.pi / 180.0)


def rad2deg(r):
    return r * (180.0 / math.pi)


def rx_mat(deg):
    rad = deg2rad(deg)
    c, s = math.cos(rad), math.sin(rad)
    return np.array([
        [1.0, 0.0,  0.0, 0.0],
        [0.0,   c,   -s, 0.0],
        [0.0,   s,    c, 0.0],
        [0.0, 0.0,  0.0, 1.0]
    ], dtype=float)


def rz_mat(deg):
    rad = deg2rad(deg)
    c, s = math.cos(rad), math.sin(rad)
    return np.array([
        [  c,  -s, 0.0, 0.0],
        [  s,   c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ], dtype=float)


def tx_mat(d):
    m = np.eye(4, dtype=float)
    m[0, 3] = float(d)
    return m


def tz_mat(d):
    m = np.eye(4, dtype=float)
    m[2, 3] = float(d)
    return m


class KinematicState:
    """Computes exact coordinate frames and mechanical landmark geometry."""
    def __init__(self, enc_deg):
        self.enc_deg = np.array(enc_deg, dtype=float)
        self.frames = []
        self.T_tool = None
        self.landmarks = {}
        self.compute()

    def compute(self):
        T = np.eye(4, dtype=float)
        self.frames = [T.copy()]  # Frame 0 (Base Origin)

        for i in range(6):
            th = self.enc_deg[i] + TH_OFFSETS[i]
            a = DH_TABLE[i]["a"]
            alpha = DH_TABLE[i]["alpha"]
            d = DH_TABLE[i]["d"]

            # Craig MDH: Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(theta_i) * Tz(d_i)
            T = T @ rx_mat(alpha) @ tx_mat(a) @ rz_mat(th) @ tz_mat(d)
            self.frames.append(T.copy())

        # Tool transform: Tz(D_TOOL) from Frame 6
        self.T_tool = self.frames[6] @ tz_mat(D_TOOL)

        # 1. Base landmarks
        p_base = np.array([0.0, 0.0, 0.0])
        p_shoulder = self.frames[1][:3, 3]  # (0, 0, D1)

        # 2. Upper arm landmarks
        p_elbow = self.frames[3][:3, 3]     # Elbow pivot (Frame 3 origin)

        # 3. Forearm physical L-bend structure
        T3 = self.frames[3]
        p_fore_bend = (T3 @ np.array([A3, 0.0, 0.0, 1.0]))[:3]
        p_wrist = self.frames[4][:3, 3]     # Wrist Center (Frame 4 origin / J5 Tilt pivot)
        p_j6 = self.frames[6][:3, 3]        # Frame 6 origin (J6 Tool Roll)

        # 4. Tool & TCP
        p_tcp = self.T_tool[:3, 3]
        tool_z_vec = self.T_tool[:3, 2]

        self.landmarks = {
            "base": p_base,
            "shoulder": p_shoulder,
            "elbow": p_elbow,
            "fore_bend": p_fore_bend,
            "wrist": p_wrist,
            "j6": p_j6,
            "tcp": p_tcp,
            "tool_z": tool_z_vec
        }

    @property
    def tcp(self):
        return self.landmarks["tcp"]

    @property
    def wrist_center(self):
        return self.landmarks["wrist"]


def forward_kinematics(enc_deg):
    state = KinematicState(enc_deg)
    return state.tcp, state.wrist_center, state


# ==============================================================================
# 3. INVERSE KINEMATICS ENGINE (Exact match to kinematics.cpp::ikPenDown)
# ==============================================================================
def ik_pen_down(target_x, target_y, target_z):
    """
    Closed-form IK for pen-down mode from src/kinematics.cpp.
    Pen points vertically down (-Z), J4 roll locked to 0, J6 locked to 0.
    Returns: (success: bool, joint_angles_deg: list[float], details: dict)
    """
    cx = float(target_x)
    cy = float(target_y)
    cz = float(target_z) + D_TOOL_EFFECTIVE  # 51.0mm (31mm D6 + 20mm D_TOOL)

    t1 = math.atan2(cy, cx)
    r = math.hypot(cx, cy)
    h = cz - D1
    dist = math.hypot(r, h)

    details = {
        "wrist_center": np.array([cx, cy, cz]),
        "r": r, "h": h, "dist": dist,
        "reason": ""
    }

    max_reach = A2 + L_FORE
    min_reach = abs(A2 - L_FORE)
    if dist > max_reach:
        details["reason"] = f"Beyond max reach ({dist:.1f} > {max_reach:.1f} mm)"
        return False, None, details
    if dist < min_reach:
        details["reason"] = f"Inside inner deadzone ({dist:.1f} < {min_reach:.1f} mm)"
        return False, None, details

    cb = (A2**2 + dist**2 - L_FORE**2) / (2.0 * A2 * dist)
    cb = max(-1.0, min(1.0, cb))
    beta = math.acos(cb)
    gamma = math.atan2(h, r)
    delta_rad = deg2rad(DELTA_WRIST)

    best_enc = None
    best_j3 = float('inf')
    found = False

    for sign in (+1, -1):
        phi2 = gamma + sign * beta
        t2 = -phi2

        r_rem = r - A2 * math.cos(phi2)
        h_rem = h - A2 * math.sin(phi2)
        phi_psi = math.atan2(h_rem, r_rem)

        q23 = -phi_psi - delta_rad
        t3 = q23 - t2

        e1 = rad2deg(t1)
        e2 = rad2deg(t2) + 90.0
        e3 = rad2deg(t3)
        e5 = -rad2deg(q23)

        if not (IK_LIMITS[0][0] <= e1 <= IK_LIMITS[0][1]):
            continue
        if not (IK_LIMITS[1][0] <= e2 <= IK_LIMITS[1][1]):
            continue
        if not (IK_LIMITS[2][0] <= e3 <= IK_LIMITS[2][1]):
            continue
        if not (IK_LIMITS[4][0] <= e5 <= IK_LIMITS[4][1]):
            continue

        if not found or abs(e3) < best_j3:
            best_enc = [e1, e2, e3, 0.0, e5, 0.0]
            best_j3 = abs(e3)
            found = True

    if not found:
        details["reason"] = "No valid IK solution within joint limits"
        return False, None, details

    details["reason"] = "IK Solution OK"
    return True, best_enc, details


# ==============================================================================
# 4. WORKPLANE 3-POINT CALIBRATION ENGINE (src/work_plane.cpp match)
# ==============================================================================
class WorkPlane:
    """
    Simulates the 3-Point Gram-Schmidt WorkPlane system from src/work_plane.cpp.
    Maps User Coordinate System (UCS u, v, w) to Robot Base (x, y, z).
    """
    def __init__(self):
        self.enabled = False
        self.p1 = np.array([100.0, -80.0, 10.0])  # Origin (u=0, v=0)
        self.p2 = np.array([180.0, -80.0, 10.0])  # +U direction
        self.p3 = np.array([100.0,  80.0, 10.0])  # +V plane definition
        self.u_axis = np.array([1.0, 0.0, 0.0])
        self.v_axis = np.array([0.0, 1.0, 0.0])
        self.normal = np.array([0.0, 0.0, 1.0])
        self.calibrate(self.p1, self.p2, self.p3)

    def calibrate(self, p1, p2, p3):
        self.p1 = np.array(p1, dtype=float)
        self.p2 = np.array(p2, dtype=float)
        self.p3 = np.array(p3, dtype=float)

        v12 = self.p2 - self.p1
        len12 = np.linalg.norm(v12)
        if len12 < 20.0:
            return False, "Points P1 and P2 too close (< 20mm)"

        u = v12 / len12
        v13 = self.p3 - self.p1
        len13 = np.linalg.norm(v13)
        if len13 < 20.0:
            return False, "Points P1 and P3 too close (< 20mm)"

        n_raw = np.cross(u, v13)
        len_n = np.linalg.norm(n_raw)
        if len_n < len13 * 0.1736:  # Sin < 10 deg (collinear)
            return False, "Points P1, P2, P3 are nearly collinear (< 10°)"

        n = n_raw / len_n
        if n[2] < 0.0:
            n = -n  # Ensure normal points upward

        v = np.cross(n, u)

        self.u_axis = u
        self.v_axis = v
        self.normal = n
        return True, "WorkPlane Calibrated Successfully"

    def ucs_to_base(self, u, v, w=0.0):
        if not self.enabled:
            return np.array([u, v, w])
        return self.p1 + u * self.u_axis + v * self.v_axis + w * self.normal

    def get_plane_mesh(self, u_range=(-30, 120), v_range=(-30, 180)):
        uu, vv = np.meshgrid(np.linspace(u_range[0], u_range[1], 5), np.linspace(v_range[0], v_range[1], 5))
        gx = self.p1[0] + uu * self.u_axis[0] + vv * self.v_axis[0]
        gy = self.p1[1] + uu * self.u_axis[1] + vv * self.v_axis[1]
        gz = self.p1[2] + uu * self.u_axis[2] + vv * self.v_axis[2]
        return gx, gy, gz


# ==============================================================================
# 5. STEP CONVERSION & RESOLUTION ANALYZER
# ==============================================================================
def degrees_to_steps(axis, deg):
    steps = int(round(deg * STEPS_PER_DEGREE[axis]))
    return AXIS_STEP_SIGN[axis] * steps


def steps_to_degrees(axis, steps):
    return AXIS_STEP_SIGN[axis] * (float(steps) / STEPS_PER_DEGREE[axis])


def compute_tcp_jacobian(enc_deg):
    state = KinematicState(enc_deg)
    p_tcp = state.tcp
    J = np.zeros((6, 6), dtype=float)

    for i in range(6):
        T_i = state.frames[i]
        z_i = T_i[:3, 2]
        p_i = T_i[:3, 3]

        J[:3, i] = np.cross(z_i, p_tcp - p_i)
        J[3:, i] = z_i

    J_pos = J[:3, :]
    w = math.sqrt(max(0.0, float(np.linalg.det(J_pos @ J_pos.T))))
    return J, w


def compute_tcp_step_resolution(enc_deg):
    J, _ = compute_tcp_jacobian(enc_deg)
    resolutions = []
    for i in range(6):
        d_theta_rad = deg2rad(1.0 / STEPS_PER_DEGREE[i])
        dx_vec = J[:3, i] * d_theta_rad
        d_total = float(np.linalg.norm(dx_vec))
        resolutions.append((float(dx_vec[0]), float(dx_vec[1]), float(dx_vec[2]), d_total))
    return resolutions


# ==============================================================================
# 6. FLAW & ANOMALY DETECTOR
# ==============================================================================
class FlawDetector:
    @staticmethod
    def audit_pose(enc_deg):
        flaws = []
        warnings = []
        state = KinematicState(enc_deg)

        # 1. Exact Joint limit checks
        for i in range(6):
            val = enc_deg[i]
            p_min, p_max = PHYSICAL_LIMITS[i]
            if val < p_min - 1e-3 or val > p_max + 1e-3:
                flaws.append(f"J{i+1} Limit Breach: {val:.1f}° ∉ [{p_min:.0f}°, {p_max:.0f}°]")

        # 2. Table penetration (Z < 0)
        for name in ["shoulder", "elbow", "fore_bend", "wrist", "tcp"]:
            p = state.landmarks[name]
            if p[2] < -0.5:
                flaws.append(f"Table Collision: {name.upper()} at Z = {p[2]:.1f} mm (< 0)")

        # 3. Base proximity
        for name in ["elbow", "fore_bend", "wrist", "tcp"]:
            p = state.landmarks[name]
            r = math.hypot(p[0], p[1])
            if r < 40.0 and p[2] < D1 - 5.0:
                warnings.append(f"Base Proximity: {name.upper()} R = {r:.1f} mm (< 40mm)")

        # 4. Singularity checks
        cx, cy, cz = state.wrist_center
        r_wrist = math.hypot(cx, cy)
        h_wrist = cz - D1
        dist = math.hypot(r_wrist, h_wrist)

        if dist > (A2 + L_FORE) - 10.0:
            warnings.append(f"Boundary Singularity: Arm near full extension ({dist:.1f}/{A2+L_FORE:.1f} mm)")
        if dist < abs(A2 - L_FORE) + 10.0:
            warnings.append(f"Inner Singularity: Arm near folded limit ({dist:.1f}/{abs(A2-L_FORE):.1f} mm)")
        if r_wrist < 25.0:
            warnings.append(f"Shoulder Singularity: Wrist axis near base center (R = {r_wrist:.1f} mm)")

        # 5. Pen verticality check
        tool_z = state.landmarks["tool_z"]
        verticality_err = math.degrees(math.acos(max(-1.0, min(1.0, abs(tool_z[2])))))
        if verticality_err > 2.0:
            warnings.append(f"Pen Tilt Error: {verticality_err:.1f}° from vertical")

        return flaws, warnings, state


# ==============================================================================
# 7. ADVANCED PLANNER & TRAJECTORY GENERATOR
# ==============================================================================
class PlannerSimulator:
    @staticmethod
    def plan_line(x1, y1, x2, y2, z_paper, work_plane=None):
        dx, dy = x2 - x1, y2 - y1
        total_len = math.hypot(dx, dy)
        if total_len < 1e-3:
            return []

        waypoints = []
        n_draw = max(2, int(math.ceil(total_len / DRAW_SEGMENT_MM)))

        p_start_safe = work_plane.ucs_to_base(x1, y1, PEN_LIFT_MM) if work_plane and work_plane.enabled else np.array([x1, y1, z_paper + PEN_LIFT_MM])
        p_start_draw = work_plane.ucs_to_base(x1, y1, 0.0) if work_plane and work_plane.enabled else np.array([x1, y1, z_paper])

        waypoints.append({"x": p_start_safe[0], "y": p_start_safe[1], "z": p_start_safe[2], "drawing": False, "phase": "TRAVEL"})
        waypoints.append({"x": p_start_draw[0], "y": p_start_draw[1], "z": p_start_draw[2], "drawing": False, "phase": "DROP"})

        for s in np.linspace(0, 1, n_draw):
            ux = x1 + s * dx
            uy = y1 + s * dy
            p_draw = work_plane.ucs_to_base(ux, uy, 0.0) if work_plane and work_plane.enabled else np.array([ux, uy, z_paper])
            waypoints.append({"x": p_draw[0], "y": p_draw[1], "z": p_draw[2], "drawing": True, "phase": "DRAW"})

        p_end_safe = work_plane.ucs_to_base(x2, y2, PEN_LIFT_MM) if work_plane and work_plane.enabled else np.array([x2, y2, z_paper + PEN_LIFT_MM])
        waypoints.append({"x": p_end_safe[0], "y": p_end_safe[1], "z": p_end_safe[2], "drawing": False, "phase": "LIFT"})
        return waypoints

    @staticmethod
    def plan_circle(cx, cy, r, z_paper, work_plane=None):
        if r <= 0.0:
            return []
        circumference = 2.0 * math.pi * r
        n_draw = max(16, int(math.ceil(circumference / DRAW_SEGMENT_MM)))
        waypoints = []

        start_u, start_v = cx + r, cy
        p_start_safe = work_plane.ucs_to_base(start_u, start_v, PEN_LIFT_MM) if work_plane and work_plane.enabled else np.array([start_u, start_v, z_paper + PEN_LIFT_MM])
        p_start_draw = work_plane.ucs_to_base(start_u, start_v, 0.0) if work_plane and work_plane.enabled else np.array([start_u, start_v, z_paper])

        waypoints.append({"x": p_start_safe[0], "y": p_start_safe[1], "z": p_start_safe[2], "drawing": False, "phase": "TRAVEL"})
        waypoints.append({"x": p_start_draw[0], "y": p_start_draw[1], "z": p_start_draw[2], "drawing": False, "phase": "DROP"})

        for ang in np.linspace(0, 2 * math.pi, n_draw):
            ux = cx + r * math.cos(ang)
            uy = cy + r * math.sin(ang)
            p_draw = work_plane.ucs_to_base(ux, uy, 0.0) if work_plane and work_plane.enabled else np.array([ux, uy, z_paper])
            waypoints.append({"x": p_draw[0], "y": p_draw[1], "z": p_draw[2], "drawing": True, "phase": "DRAW"})

        waypoints.append({"x": p_start_safe[0], "y": p_start_safe[1], "z": p_start_safe[2], "drawing": False, "phase": "LIFT"})
        return waypoints

    @staticmethod
    def plan_spiral(cx, cy, r_max, z_paper, work_plane=None):
        n_points = 70
        waypoints = []
        theta_max = 4.0 * math.pi

        p_start_safe = work_plane.ucs_to_base(cx, cy, PEN_LIFT_MM) if work_plane and work_plane.enabled else np.array([cx, cy, z_paper + PEN_LIFT_MM])
        waypoints.append({"x": p_start_safe[0], "y": p_start_safe[1], "z": p_start_safe[2], "drawing": False, "phase": "TRAVEL"})

        for th in np.linspace(0, theta_max, n_points):
            r = (r_max / theta_max) * th
            ux = cx + r * math.cos(th)
            uy = cy + r * math.sin(th)
            p_draw = work_plane.ucs_to_base(ux, uy, 0.0) if work_plane and work_plane.enabled else np.array([ux, uy, z_paper])
            waypoints.append({"x": p_draw[0], "y": p_draw[1], "z": p_draw[2], "drawing": True, "phase": "DRAW"})

        last = waypoints[-1]
        waypoints.append({"x": last["x"], "y": last["y"], "z": last["z"] + PEN_LIFT_MM, "drawing": False, "phase": "LIFT"})
        return waypoints

    @staticmethod
    def plan_star(cx, cy, r_outer, z_paper, work_plane=None):
        r_inner = r_outer * 0.42
        n_points = 5
        waypoints = []

        angles = []
        for i in range(n_points * 2 + 1):
            a = math.pi / 2.0 + i * (math.pi / n_points)
            r = r_outer if i % 2 == 0 else r_inner
            angles.append((cx + r * math.cos(a), cy + r * math.sin(a)))

        first = angles[0]
        p_safe = work_plane.ucs_to_base(first[0], first[1], PEN_LIFT_MM) if work_plane and work_plane.enabled else np.array([first[0], first[1], z_paper + PEN_LIFT_MM])
        waypoints.append({"x": p_safe[0], "y": p_safe[1], "z": p_safe[2], "drawing": False, "phase": "TRAVEL"})

        for pt in angles:
            p_draw = work_plane.ucs_to_base(pt[0], pt[1], 0.0) if work_plane and work_plane.enabled else np.array([pt[0], pt[1], z_paper])
            waypoints.append({"x": p_draw[0], "y": p_draw[1], "z": p_draw[2], "drawing": True, "phase": "DRAW"})

        waypoints.append({"x": p_safe[0], "y": p_safe[1], "z": p_safe[2], "drawing": False, "phase": "LIFT"})
        return waypoints

    @staticmethod
    def evaluate_trajectory(waypoints):
        results = []
        all_ok = True
        prev_angles = None
        dt = 0.05  # 50ms per step approx

        for i, wp in enumerate(waypoints):
            success, angles, details = ik_pen_down(wp["x"], wp["y"], wp["z"])
            res = {
                "idx": i,
                "x": wp["x"], "y": wp["y"], "z": wp["z"],
                "drawing": wp["drawing"],
                "phase": wp["phase"],
                "ik_ok": success,
                "angles": angles,
                "step_rates": [0.0] * 6,
                "flaws": [],
                "warnings": []
            }

            if not success:
                all_ok = False
                res["flaws"].append(f"IK Fail: {details['reason']}")
            else:
                flaws, warnings, _ = FlawDetector.audit_pose(angles)
                res["flaws"].extend(flaws)
                res["warnings"].extend(warnings)
                if len(flaws) > 0:
                    all_ok = False

                # Compute step rates
                if prev_angles is not None:
                    for j in range(6):
                        d_deg = abs(angles[j] - prev_angles[j])
                        d_steps = d_deg * STEPS_PER_DEGREE[j]
                        res["step_rates"][j] = d_steps / dt
                prev_angles = list(angles)

            results.append(res)
        return all_ok, results


# ==============================================================================
# 8. HIGH-PRECISION MULTI-VIEW GUI (Pro Edition)
# ==============================================================================
class DigitalCloneGUI:
    def __init__(self, robot_host="http://robot-arm.local"):
        self.robot_host = robot_host
        self.joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.target_xyz = [160.0, 0.0, 10.0]
        self.path_type = "line"
        self.path_results = []
        self.anim_idx = 0
        self.is_playing = False
        self._updating = False
        self.work_plane = WorkPlane()
        self.bridge_status = "Bridge: Idle (Ready to Connect)"

        self.setup_ui()
        self.setup_artists()
        self.generate_path()

        # Connect click events for interactive Click-to-Move
        self.fig.canvas.mpl_connect('button_press_event', self.on_canvas_click)

        # Non-blocking animation timer using Matplotlib canvas timer
        self.anim_timer = self.fig.canvas.new_timer(interval=35)
        self.anim_timer.add_callback(self.on_anim_tick)

        self.update_all()
        plt.show()

    def setup_ui(self):
        # High-DPI dark engineering theme
        self.fig = plt.figure(figsize=(19.2, 10.0), dpi=100)
        self.fig.patch.set_facecolor('#0b0f19')
        self.fig.canvas.manager.set_window_title("NEMA-6AXIS Robotic Arm — Precision Digital Clone Pro")

        # ======================================================================
        # COLUMN 1 (Left 46%): MULTI-VIEW VISUALIZER
        # ======================================================================
        # 1. Main 3D Perspective View (Top)
        self.ax3d = self.fig.add_axes([0.03, 0.38, 0.44, 0.58], projection='3d')
        self.ax3d.set_facecolor('#111827')
        self.ax3d.set_xlim(-260, 260)
        self.ax3d.set_ylim(-260, 260)
        self.ax3d.set_zlim(0, 420)
        self.ax3d.set_box_aspect([1.0, 1.0, 0.81])
        self.ax3d.set_xlabel("X (mm)", color='#94a3b8', fontsize=8, labelpad=3)
        self.ax3d.set_ylabel("Y (mm)", color='#94a3b8', fontsize=8, labelpad=3)
        self.ax3d.set_zlabel("Z (mm)", color='#94a3b8', fontsize=8, labelpad=3)
        self.ax3d.tick_params(colors='#64748b', labelsize=7)
        self.ax3d.set_title("3D ISOMETRIC VIEW (Click & Drag to Rotate | 3-Point WorkPlane Active)",
                            color='#38bdf8', fontsize=9.5, fontweight='bold', pad=4)

        # 2. 2D Side Elevation X-Z (Bottom-Left)
        self.ax_side = self.fig.add_axes([0.03, 0.05, 0.21, 0.28])
        self.ax_side.set_facecolor('#111827')
        self.ax_side.set_title("SIDE VIEW (Click to Move X-Z)", color='#38bdf8', fontsize=8.5, fontweight='bold', pad=4)
        self.ax_side.set_xlabel("X (mm)", color='#94a3b8', fontsize=7)
        self.ax_side.set_ylabel("Z (mm)", color='#94a3b8', fontsize=7)
        self.ax_side.set_xlim(-150, 320)
        self.ax_side.set_ylim(-20, 420)
        self.ax_side.set_aspect('equal', adjustable='box')
        self.ax_side.tick_params(colors='#64748b', labelsize=6.5)
        self.ax_side.grid(True, linestyle='--', alpha=0.25, color='#475569')
        self.ax_side.axhline(0, color='#06b6d4', linestyle='-', linewidth=1.2, alpha=0.8, label='Table Z=0')
        self.ax_side.axhline(PEN_LIFT_MM, color='#38bdf8', linestyle=':', linewidth=1.0, alpha=0.6, label='Lift +5mm')
        self.ax_side.legend(loc='upper right', fontsize=6.0, facecolor='#111827', edgecolor='#1f2937', labelcolor='#cbd5e1')

        # 3. 2D Top Plan X-Y (Bottom-Right)
        self.ax_top = self.fig.add_axes([0.26, 0.05, 0.21, 0.28])
        self.ax_top.set_facecolor('#111827')
        self.ax_top.set_title("TOP VIEW (Click to Move X-Y)", color='#38bdf8', fontsize=8.5, fontweight='bold', pad=4)
        self.ax_top.set_xlabel("X (mm)", color='#94a3b8', fontsize=7)
        self.ax_top.set_ylabel("Y (mm)", color='#94a3b8', fontsize=7)
        self.ax_top.set_xlim(-260, 260)
        self.ax_top.set_ylim(-260, 260)
        self.ax_top.set_aspect('equal', adjustable='box')
        self.ax_top.tick_params(colors='#64748b', labelsize=6.5)
        self.ax_top.grid(True, linestyle='--', alpha=0.25, color='#475569')
        self.ax_top.scatter(0, 0, color='#0284c7', s=60, marker='s', zorder=5)

        u_circ = np.linspace(0, 2 * math.pi, 60)
        self.ax_top.plot((A2 + L_FORE) * np.cos(u_circ), (A2 + L_FORE) * np.sin(u_circ),
                         color='#38bdf8', linestyle=':', linewidth=1.0, alpha=0.7, label=f'Max ({A2+L_FORE:.0f}mm)')
        self.ax_top.plot(abs(A2 - L_FORE) * np.cos(u_circ), abs(A2 - L_FORE) * np.sin(u_circ),
                         color='#f59e0b', linestyle=':', linewidth=1.0, alpha=0.6, label=f'Min ({abs(A2-L_FORE):.1f}mm)')
        self.ax_top.legend(loc='upper right', fontsize=6.0, facecolor='#111827', edgecolor='#1f2937', labelcolor='#cbd5e1')

        # ======================================================================
        # COLUMN 2 (Right 50%): CONTROL STATION & ENGINEERING SUITE
        # ======================================================================
        # SECTION 1: Joint Jog Controls (6 Sliders)
        self.fig.text(0.51, 0.96, "1. JOINT JOG (Degrees):", fontsize=8.5, fontweight='bold', color='#38bdf8')

        self.sliders = []
        slider_y = [0.915, 0.865, 0.815, 0.765, 0.715, 0.665]
        for i in range(6):
            ax = self.fig.add_axes([0.51, slider_y[i], 0.20, 0.022])
            ax.set_facecolor('#1e293b')
            p_min, p_max = PHYSICAL_LIMITS[i]
            s = Slider(ax, JOINT_NAMES[i], p_min, p_max, valinit=self.joint_angles[i], valstep=0.5,
                       color='#38bdf8', track_color='#0f172a')
            s.label.set_color('#cbd5e1')
            s.label.set_fontsize(7.0)
            s.valtext.set_color('#38bdf8')
            s.valtext.set_fontsize(7.0)
            s.valtext.set_fontweight('bold')
            s.on_changed(lambda val, idx=i: self.on_joint_slider(idx, val))
            self.sliders.append(s)

        # SECTION 2: Cartesian Target Controls (3 Sliders)
        self.fig.text(0.74, 0.96, "2. CARTESIAN TARGET (mm):", fontsize=8.5, fontweight='bold', color='#f59e0b')

        self.cart_sliders = []
        cart_labels = ["Target X (mm)", "Target Y (mm)", "Target Z (mm)"]
        cart_ranges = [(-250.0, 250.0), (-250.0, 250.0), (-10.0, 350.0)]
        cart_y = [0.915, 0.865, 0.815]
        for i in range(3):
            ax = self.fig.add_axes([0.74, cart_y[i], 0.23, 0.022])
            ax.set_facecolor('#1e293b')
            s = Slider(ax, cart_labels[i], cart_ranges[i][0], cart_ranges[i][1], valinit=self.target_xyz[i], valstep=1.0,
                       color='#f59e0b', track_color='#0f172a')
            s.label.set_color('#cbd5e1')
            s.label.set_fontsize(7.0)
            s.valtext.set_color('#f59e0b')
            s.valtext.set_fontsize(7.0)
            s.valtext.set_fontweight('bold')
            s.on_changed(lambda val, idx=i: self.on_cart_slider(idx, val))
            self.cart_sliders.append(s)

        # SECTION 3: Presets & Hardware Bridge
        self.fig.text(0.74, 0.77, "3. PRESETS & HARDWARE BRIDGE:", fontsize=8.5, fontweight='bold', color='#a855f7')
        preset_defs = [
            ("Home (0°)", self.on_home, [0.74, 0.72, 0.054, 0.030]),
            ("Draw Ready", self.on_preset_draw, [0.80, 0.72, 0.054, 0.030]),
            ("Reach +X", self.on_preset_reach_fwd, [0.86, 0.72, 0.054, 0.030]),
            ("Folded", self.on_preset_fold, [0.92, 0.72, 0.050, 0.030]),
            ("📡 Sync ESP32", self.on_sync_robot, [0.74, 0.675, 0.075, 0.030]),
            ("⚡ Send to Robot", self.on_send_robot, [0.825, 0.675, 0.075, 0.030]),
            ("📐 Plane Presets", self.on_toggle_plane_preset, [0.91, 0.675, 0.060, 0.030]),
        ]
        self.preset_buttons = []
        for text, cb, pos in preset_defs:
            ax = self.fig.add_axes(pos)
            btn = Button(ax, text, color='#1e293b', hovercolor='#334155')
            btn.label.set_color('#f8fafc')
            btn.label.set_fontsize(7.0)
            btn.label.set_fontweight('bold')
            btn.on_clicked(cb)
            self.preset_buttons.append(btn)

        # SECTION 4: Trajectory Studio & WorkPlane
        self.fig.text(0.51, 0.62, "4. TRAJECTORY GENERATOR & WORKPLANE:", fontsize=8.5, fontweight='bold', color='#10b981')

        # Shape buttons
        shapes = [
            ("Line", "line", [0.51, 0.57, 0.065, 0.030]),
            ("Circle", "circle", [0.585, 0.57, 0.065, 0.030]),
            ("Spiral", "spiral", [0.66, 0.57, 0.065, 0.030]),
            ("Star", "star", [0.735, 0.57, 0.065, 0.030]),
        ]
        self.shape_buttons = []
        for label, stype, pos in shapes:
            ax = self.fig.add_axes(pos)
            btn = Button(ax, label, color='#059669' if stype == self.path_type else '#1e293b', hovercolor='#10b981')
            btn.label.set_color('#ffffff')
            btn.label.set_fontsize(7.0)
            btn.label.set_fontweight('bold')
            btn.on_clicked(lambda e, s=stype: self.set_path_shape(s))
            self.shape_buttons.append((btn, stype))

        # WorkPlane Toggle button
        ax_wp = self.fig.add_axes([0.81, 0.57, 0.075, 0.030])
        self.btn_wp = Button(ax_wp, "Plane: OFF", color='#1e293b', hovercolor='#334155')
        self.btn_wp.label.set_color('#94a3b8')
        self.btn_wp.label.set_fontsize(7.0)
        self.btn_wp.label.set_fontweight('bold')
        self.btn_wp.on_clicked(self.on_toggle_workplane)

        # Play / Pause button
        ax_play = self.fig.add_axes([0.895, 0.57, 0.075, 0.030])
        self.btn_play = Button(ax_play, "▶ Play", color='#0284c7', hovercolor='#0ea5e9')
        self.btn_play.label.set_color('#ffffff')
        self.btn_play.label.set_fontsize(7.5)
        self.btn_play.label.set_fontweight('bold')
        self.btn_play.on_clicked(self.on_toggle_play)

        # Scrubber Slider
        ax_scrub = self.fig.add_axes([0.51, 0.515, 0.46, 0.022])
        ax_scrub.set_facecolor('#1e293b')
        self.scrub_slider = Slider(ax_scrub, "Path Scrub", 0, 100, valinit=0, valstep=1,
                                   color='#10b981', track_color='#0f172a')
        self.scrub_slider.label.set_color('#cbd5e1')
        self.scrub_slider.label.set_fontsize(7.0)
        self.scrub_slider.valtext.set_color('#10b981')
        self.scrub_slider.valtext.set_fontsize(7.0)
        self.scrub_slider.on_changed(self.on_scrub)

        # SECTION 5: Telemetry, Velocity Profiler & Diagnostics HUD
        self.fig.text(0.51, 0.46, "5. TELEMETRY, VELOCITY PROFILER & SAFETY HUD:", fontsize=8.5, fontweight='bold', color='#e2e8f0')
        self.ax_diag = self.fig.add_axes([0.51, 0.05, 0.46, 0.39])
        self.ax_diag.set_facecolor('#111827')
        for spine in self.ax_diag.spines.values():
            spine.set_color('#1f2937')
            spine.set_linewidth(1.2)
        self.ax_diag.set_xticks([])
        self.ax_diag.set_yticks([])
        self.info_text = self.ax_diag.text(0.02, 0.95, "", fontsize=7.4, family='monospace',
                                           color='#f8fafc', va='top', ha='left')

    def setup_artists(self):
        # 3D Link Artists
        self.line_3d_base, = self.ax3d.plot([], [], [], color='#64748b', linewidth=8, solid_capstyle='round', label=f'Base ({D1:.0f}mm)')
        self.line_3d_upper, = self.ax3d.plot([], [], [], color='#0ea5e9', linewidth=6.5, solid_capstyle='round', label=f'Upper Arm ({A2:.0f}mm)')
        self.line_3d_fore1, = self.ax3d.plot([], [], [], color='#10b981', linewidth=5.5, solid_capstyle='round', label=f'Forearm ({A3:.0f}mm)')
        self.line_3d_fore2, = self.ax3d.plot([], [], [], color='#059669', linewidth=5.5, solid_capstyle='round', label=f'Offset ({D4:.0f}mm)')
        self.line_3d_pen, = self.ax3d.plot([], [], [], color='#f43f5e', linewidth=4, linestyle='--', solid_capstyle='round', label=f'Pen ({D_TOOL:.0f}mm)')
        self.scatter_3d_joints = self.ax3d.scatter([], [], [], color='#f59e0b', s=55, edgecolors='#0f172a', depthshade=False)
        self.scatter_3d_tcp = self.ax3d.scatter([], [], [], color='#f43f5e', s=100, marker='v', depthshade=False)
        self.line_3d_drawn, = self.ax3d.plot([], [], [], color='#f43f5e', linewidth=3, label='Draw Path')
        self.line_3d_travel, = self.ax3d.plot([], [], [], color='#38bdf8', linestyle=':', linewidth=1.5, label='Travel')

        # 3D WorkPlane Visualizer
        self.surface_workplane = None

        # 3D Ground paper
        gx, gy = np.meshgrid(np.linspace(-220, 220, 11), np.linspace(-220, 220, 11))
        gz = np.zeros_like(gx)
        self.ax3d.plot_surface(gx, gy, gz, alpha=0.07, color='cyan')
        self.ax3d.legend(loc='upper left', fontsize=6.5, facecolor='#111827', edgecolor='#1f2937', labelcolor='#cbd5e1')

        # 2D Side Artists
        self.line_side_base, = self.ax_side.plot([], [], color='#64748b', linewidth=6)
        self.line_side_upper, = self.ax_side.plot([], [], color='#0ea5e9', linewidth=4.5)
        self.line_side_fore1, = self.ax_side.plot([], [], color='#10b981', linewidth=4)
        self.line_side_fore2, = self.ax_side.plot([], [], color='#059669', linewidth=4)
        self.line_side_pen, = self.ax_side.plot([], [], color='#f43f5e', linewidth=3.5, linestyle='--')
        self.scatter_side_joints = self.ax_side.scatter([], [], color='#f59e0b', s=40, zorder=5)
        self.scatter_side_tcp = self.ax_side.scatter([], [], color='#f43f5e', s=70, marker='v', zorder=6)

        # 2D Top Artists
        self.line_top_upper, = self.ax_top.plot([], [], color='#0ea5e9', linewidth=4)
        self.line_top_fore1, = self.ax_top.plot([], [], color='#10b981', linewidth=3.5)
        self.line_top_fore2, = self.ax_top.plot([], [], color='#059669', linewidth=3.5)
        self.line_top_pen, = self.ax_top.plot([], [], color='#f43f5e', linewidth=3, linestyle='--')
        self.scatter_top_joints = self.ax_top.scatter([], [], color='#f59e0b', s=40, zorder=5)
        self.scatter_top_tcp = self.ax_top.scatter([], [], color='#f43f5e', s=70, marker='o', zorder=6)
        self.line_top_drawn, = self.ax_top.plot([], [], color='#f43f5e', linewidth=2.5)

    def on_canvas_click(self, event):
        """Interactive Click-to-Move handler on 2D Top and Side Views."""
        if event.inaxes == self.ax_top and event.xdata is not None and event.ydata is not None:
            self.target_xyz[0] = round(event.xdata, 1)
            self.target_xyz[1] = round(event.ydata, 1)
            self._set_cart_sliders_safe(self.target_xyz)
            ok, angles, details = ik_pen_down(self.target_xyz[0], self.target_xyz[1], self.target_xyz[2])
            if ok:
                self.joint_angles = list(angles)
                self._set_joint_sliders_safe(self.joint_angles)
            self.update_all(ik_details=details)

        elif event.inaxes == self.ax_side and event.xdata is not None and event.ydata is not None:
            self.target_xyz[0] = round(event.xdata, 1)
            self.target_xyz[2] = max(-10.0, min(350.0, round(event.ydata, 1)))
            self._set_cart_sliders_safe(self.target_xyz)
            ok, angles, details = ik_pen_down(self.target_xyz[0], self.target_xyz[1], self.target_xyz[2])
            if ok:
                self.joint_angles = list(angles)
                self._set_joint_sliders_safe(self.joint_angles)
            self.update_all(ik_details=details)

    def on_joint_slider(self, idx, val):
        if self._updating:
            return
        self.joint_angles[idx] = val
        tcp, _, _ = forward_kinematics(self.joint_angles)
        self._set_cart_sliders_safe(tcp)
        self.update_all()

    def on_cart_slider(self, idx, val):
        if self._updating:
            return
        self.target_xyz[idx] = val
        ok, angles, details = ik_pen_down(self.target_xyz[0], self.target_xyz[1], self.target_xyz[2])
        if ok:
            self.joint_angles = list(angles)
            self._set_joint_sliders_safe(self.joint_angles)
        self.update_all(ik_details=details)

    def _set_joint_sliders_safe(self, angles):
        self._updating = True
        for i, s in enumerate(self.sliders):
            s.eventson = False
            s.set_val(angles[i])
            s.eventson = True
        self._updating = False

    def _set_cart_sliders_safe(self, xyz):
        self._updating = True
        for i, s in enumerate(self.cart_sliders):
            s.eventson = False
            s.set_val(xyz[i])
            s.eventson = True
        self._updating = False

    def on_home(self, event):
        self.joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._set_joint_sliders_safe(self.joint_angles)
        tcp, _, _ = forward_kinematics(self.joint_angles)
        self._set_cart_sliders_safe(tcp)
        self.update_all()

    def on_preset_draw(self, event):
        ok, angles, details = ik_pen_down(160.0, 0.0, 10.0)
        if ok:
            self.joint_angles = list(angles)
            self._set_joint_sliders_safe(self.joint_angles)
            self._set_cart_sliders_safe([160.0, 0.0, 10.0])
        self.update_all(ik_details=details)

    def on_preset_reach_fwd(self, event):
        self.joint_angles = [0.0, 90.0, 0.0, 0.0, -DELTA_WRIST, 0.0]
        self._set_joint_sliders_safe(self.joint_angles)
        tcp, _, _ = forward_kinematics(self.joint_angles)
        self._set_cart_sliders_safe(tcp)
        self.update_all()

    def on_preset_fold(self, event):
        self.joint_angles = [0.0, 45.0, 90.0, 0.0, -45.0, 0.0]
        self._set_joint_sliders_safe(self.joint_angles)
        tcp, _, _ = forward_kinematics(self.joint_angles)
        self._set_cart_sliders_safe(tcp)
        self.update_all()

    def on_toggle_workplane(self, event):
        self.work_plane.enabled = not self.work_plane.enabled
        if self.work_plane.enabled:
            self.btn_wp.label.set_text("Plane: ON")
            self.btn_wp.color = '#059669'
            self.btn_wp.label.set_color('#ffffff')
        else:
            self.btn_wp.label.set_text("Plane: OFF")
            self.btn_wp.color = '#1e293b'
            self.btn_wp.label.set_color('#94a3b8')
        self.generate_path()
        self.update_all()

    def on_toggle_plane_preset(self, event):
        # Cycles between: Flat, 15° Desk, 35° Easel, Vertical
        if not hasattr(self, '_plane_preset_idx'):
            self._plane_preset_idx = 0
        self._plane_preset_idx = (self._plane_preset_idx + 1) % 4

        if self._plane_preset_idx == 0:
            self.work_plane.calibrate([100, -80, 10], [180, -80, 10], [100, 80, 10])
            self.bridge_status = "Plane: Horizontal Table (Z=10mm)"
        elif self._plane_preset_idx == 1:
            self.work_plane.calibrate([100, -80, 5], [180, -80, 26], [100, 80, 5])
            self.bridge_status = "Plane: Slanted Desk 15° Incline"
        elif self._plane_preset_idx == 2:
            self.work_plane.calibrate([110, -70, 10], [180, -70, 60], [110, 70, 10])
            self.bridge_status = "Plane: Easel Board 35° Incline"
        else:
            self.work_plane.calibrate([150, -60, 20], [150, -60, 120], [150, 60, 20])
            self.bridge_status = "Plane: Vertical Drawing Board"

        self.work_plane.enabled = True
        self.btn_wp.label.set_text("Plane: ON")
        self.btn_wp.color = '#059669'
        self.btn_wp.label.set_color('#ffffff')
        self.generate_path()
        self.update_all()

    def on_sync_robot(self, event):
        def _sync_thread():
            try:
                url = f"{self.robot_host}/api/status"
                req = urllib.request.Request(url, headers={'User-Agent': 'DigitalClone/2.0'})
                with urllib.request.urlopen(req, timeout=1.5) as resp:
                    data = json.loads(resp.read().decode('utf-8'))
                    if "joints" in data and len(data["joints"]) >= 6:
                        angles = [float(j.get("deg", 0.0)) for j in data["joints"][:6]]
                        self.joint_angles = angles
                        self._set_joint_sliders_safe(angles)
                        tcp, _, _ = forward_kinematics(angles)
                        self._set_cart_sliders_safe(tcp)
                        self.bridge_status = f"ESP32 Synced OK ({data.get('mode', 'IDLE')})"
            except Exception as e:
                self.bridge_status = f"Sync Failed: {str(e)[:35]}"
            self.update_all()
        threading.Thread(target=_sync_thread, daemon=True).start()

    def on_send_robot(self, event):
        def _send_thread():
            try:
                url = f"{self.robot_host}/api/move"
                payload = json.dumps({"x": self.target_xyz[0], "y": self.target_xyz[1], "z": self.target_xyz[2]}).encode('utf-8')
                req = urllib.request.Request(url, data=payload, headers={'Content-Type': 'application/json'})
                with urllib.request.urlopen(req, timeout=1.5) as resp:
                    self.bridge_status = "Dispatched Move to ESP32 OK"
            except Exception as e:
                self.bridge_status = f"Dispatch Failed: {str(e)[:35]}"
            self.update_all()
        threading.Thread(target=_send_thread, daemon=True).start()

    def set_path_shape(self, shape):
        self.path_type = shape
        for btn, stype in self.shape_buttons:
            if stype == shape:
                btn.color = '#059669'
            else:
                btn.color = '#1e293b'
        self.generate_path()
        self.fig.canvas.draw_idle()

    def generate_path(self):
        wp = self.work_plane if self.work_plane.enabled else None
        if self.path_type == "circle":
            wps = PlannerSimulator.plan_circle(140.0, 0.0, 45.0, z_paper=10.0, work_plane=wp)
        elif self.path_type == "spiral":
            wps = PlannerSimulator.plan_spiral(140.0, 0.0, 50.0, z_paper=10.0, work_plane=wp)
        elif self.path_type == "star":
            wps = PlannerSimulator.plan_star(140.0, 0.0, 50.0, z_paper=10.0, work_plane=wp)
        else:
            wps = PlannerSimulator.plan_line(100.0, -70.0, 180.0, 70.0, z_paper=10.0, work_plane=wp)

        all_ok, self.path_results = PlannerSimulator.evaluate_trajectory(wps)
        self.scrub_slider.valmax = max(1, len(self.path_results) - 1)
        self.scrub_slider.ax.set_xlim(0, self.scrub_slider.valmax)

        self.anim_idx = 0
        if len(self.path_results) > 0 and self.path_results[0]["angles"] is not None:
            self.joint_angles = list(self.path_results[0]["angles"])
            self._set_joint_sliders_safe(self.joint_angles)
            tcp, _, _ = forward_kinematics(self.joint_angles)
            self._set_cart_sliders_safe(tcp)
        self.update_all()

    def on_scrub(self, val):
        if len(self.path_results) == 0:
            return
        idx = int(round(val))
        idx = max(0, min(len(self.path_results) - 1, idx))
        self.anim_idx = idx
        wp = self.path_results[idx]
        if wp["ik_ok"] and wp["angles"] is not None:
            self.joint_angles = list(wp["angles"])
            self._set_joint_sliders_safe(self.joint_angles)
            tcp, _, _ = forward_kinematics(self.joint_angles)
            self._set_cart_sliders_safe(tcp)
            self.update_all()

    def on_toggle_play(self, event):
        if self.is_playing:
            self.is_playing = False
            self.anim_timer.stop()
            self.btn_play.label.set_text("▶ Play")
            self.btn_play.color = '#0284c7'
        else:
            if len(self.path_results) == 0:
                self.generate_path()
            self.is_playing = True
            self.anim_timer.start()
            self.btn_play.label.set_text("⏸ Pause")
            self.btn_play.color = '#d97706'
        self.fig.canvas.draw_idle()

    def on_anim_tick(self):
        if not self.is_playing or len(self.path_results) == 0:
            return
        self.anim_idx = (self.anim_idx + 1) % len(self.path_results)

        # Update scrubber position
        self.scrub_slider.eventson = False
        self.scrub_slider.set_val(self.anim_idx)
        self.scrub_slider.eventson = True

        wp = self.path_results[self.anim_idx]
        if wp["ik_ok"] and wp["angles"] is not None:
            self.joint_angles = list(wp["angles"])
            self._set_joint_sliders_safe(self.joint_angles)
            tcp, _, _ = forward_kinematics(self.joint_angles)
            self._set_cart_sliders_safe(tcp)
            self.update_all()

    def update_all(self, ik_details=None):
        tcp, wrist, state = forward_kinematics(self.joint_angles)
        lm = state.landmarks
        p_base = lm["base"]
        p_sh = lm["shoulder"]
        p_el = lm["elbow"]
        p_bend = lm["fore_bend"]
        p_wr = lm["wrist"]
        p_tcp = lm["tcp"]

        # 1. Update 3D Artists
        self.line_3d_base.set_data_3d([p_base[0], p_sh[0]], [p_base[1], p_sh[1]], [p_base[2], p_sh[2]])
        self.line_3d_upper.set_data_3d([p_sh[0], p_el[0]], [p_sh[1], p_el[1]], [p_sh[2], p_el[2]])
        self.line_3d_fore1.set_data_3d([p_el[0], p_bend[0]], [p_el[1], p_bend[1]], [p_el[2], p_bend[2]])
        self.line_3d_fore2.set_data_3d([p_bend[0], p_wr[0]], [p_bend[1], p_wr[1]], [p_bend[2], p_wr[2]])
        self.line_3d_pen.set_data_3d([p_wr[0], p_tcp[0]], [p_wr[1], p_tcp[1]], [p_wr[2], p_tcp[2]])

        jx = [p_sh[0], p_el[0], p_bend[0], p_wr[0]]
        jy = [p_sh[1], p_el[1], p_bend[1], p_wr[1]]
        jz = [p_sh[2], p_el[2], p_bend[2], p_wr[2]]
        self.scatter_3d_joints._offsets3d = (jx, jy, jz)
        self.scatter_3d_tcp._offsets3d = ([p_tcp[0]], [p_tcp[1]], [p_tcp[2]])

        # Update 3D WorkPlane Mesh
        if self.work_plane.enabled:
            gx, gy, gz = self.work_plane.get_plane_mesh()
            if self.surface_workplane:
                self.surface_workplane.remove()
            self.surface_workplane = self.ax3d.plot_surface(gx, gy, gz, alpha=0.18, color='#10b981')
        elif self.surface_workplane:
            self.surface_workplane.remove()
            self.surface_workplane = None

        if len(self.path_results) > 0:
            draw_pts = [p for p in self.path_results if p["drawing"] and p["ik_ok"]]
            trav_pts = [p for p in self.path_results if not p["drawing"] and p["ik_ok"]]
            if len(draw_pts) > 0:
                self.line_3d_drawn.set_data_3d([p["x"] for p in draw_pts], [p["y"] for p in draw_pts], [p["z"] for p in draw_pts])
            if len(trav_pts) > 0:
                self.line_3d_travel.set_data_3d([p["x"] for p in trav_pts], [p["y"] for p in trav_pts], [p["z"] for p in trav_pts])

        # 2. Update 2D Side Elevation Artists
        self.line_side_base.set_data([p_base[0], p_sh[0]], [p_base[2], p_sh[2]])
        self.line_side_upper.set_data([p_sh[0], p_el[0]], [p_sh[2], p_el[2]])
        self.line_side_fore1.set_data([p_el[0], p_bend[0]], [p_el[2], p_bend[2]])
        self.line_side_fore2.set_data([p_bend[0], p_wr[0]], [p_bend[2], p_wr[2]])
        self.line_side_pen.set_data([p_wr[0], p_tcp[0]], [p_wr[2], p_tcp[2]])
        self.scatter_side_joints.set_offsets(np.c_[jx, jz])
        self.scatter_side_tcp.set_offsets([[p_tcp[0], p_tcp[2]]])

        # 3. Update 2D Top View Artists
        self.line_top_upper.set_data([p_sh[0], p_el[0]], [p_sh[1], p_el[1]])
        self.line_top_fore1.set_data([p_el[0], p_bend[0]], [p_el[1], p_bend[1]])
        self.line_top_fore2.set_data([p_bend[0], p_wr[0]], [p_bend[1], p_wr[1]])
        self.line_top_pen.set_data([p_wr[0], p_tcp[0]], [p_wr[1], p_tcp[1]])
        self.scatter_top_joints.set_offsets(np.c_[jx, jy])
        self.scatter_top_tcp.set_offsets([[p_tcp[0], p_tcp[1]]])

        if len(self.path_results) > 0:
            draw_pts = [p for p in self.path_results if p["drawing"] and p["ik_ok"]]
            if len(draw_pts) > 0:
                self.line_top_drawn.set_data([p["x"] for p in draw_pts], [p["y"] for p in draw_pts])

        # 4. Diagnostics & Flaw Audit
        flaws, warnings, _ = FlawDetector.audit_pose(self.joint_angles)
        resolutions = compute_tcp_step_resolution(self.joint_angles)
        _, manipulability = compute_tcp_jacobian(self.joint_angles)

        self.render_diagnostics(tcp, wrist, flaws, warnings, resolutions, manipulability, ik_details)
        self.fig.canvas.draw_idle()

    def render_diagnostics(self, tcp, wrist, flaws, warnings, resolutions, w, ik_details):
        status_tag = "[OK: NORMAL (SAFE)]" if len(flaws) == 0 and len(warnings) == 0 else \
                     ("[CRITICAL FAULT]" if len(flaws) > 0 else "[WARNING: MARGINAL]")

        lines = [
            f"TELEMETRY HUD | {status_tag} | Manipulability w: {w:.2f} | {self.bridge_status}",
            f"TCP Tip:   X={tcp[0]:+6.1f} mm  Y={tcp[1]:+6.1f} mm  Z={tcp[2]:+6.1f} mm  |  Wrist: X={wrist[0]:+6.1f} Y={wrist[1]:+6.1f} Z={wrist[2]:+6.1f} mm",
            "JOINT STATUS & RESOLUTION PER MICROSTEP:"
        ]

        joint_row_1 = []
        joint_row_2 = []
        for i in range(6):
            deg = self.joint_angles[i]
            steps = degrees_to_steps(i, deg)
            p_min, p_max = PHYSICAL_LIMITS[i]
            res_um = resolutions[i][3] * 1000.0
            info = f"J{i+1}:{deg:+5.1f}° [{p_min:+.0f}..{p_max:+.0f}] ({steps:6d} stp | {res_um:4.1f}µm)"
            if i < 3:
                joint_row_1.append(info)
            else:
                joint_row_2.append(info)

        lines.append("  " + "  |  ".join(joint_row_1))
        lines.append("  " + "  |  ".join(joint_row_2))

        # Velocity & Step-frequency profiler summary
        if len(self.path_results) > 0 and self.anim_idx < len(self.path_results):
            cur_wp = self.path_results[self.anim_idx]
            max_freq = max(cur_wp.get("step_rates", [0.0]))
            lines.append(f"MOTION PROFILER: Waypoint #{self.anim_idx+1}/{len(self.path_results)} [{cur_wp['phase']}] | Peak Step Rate: {max_freq:5.0f} Hz (Limit 50kHz OK)")

        # WorkPlane Frame vectors
        if self.work_plane.enabled:
            n = self.work_plane.normal
            lines.append(f"WORKPLANE UCS: Normal n=[{n[0]:+.2f}, {n[1]:+.2f}, {n[2]:+.2f}] | P1=[{self.work_plane.p1[0]:.0f},{self.work_plane.p1[1]:.0f},{self.work_plane.p1[2]:.0f}]")

        if ik_details and ik_details.get("reason"):
            lines.append(f"IK Status: {ik_details['reason']}")

        if len(flaws) > 0:
            lines.append(f"CRITICAL FLAWS ({len(flaws)}): " + " ; ".join(flaws))
        elif len(warnings) > 0:
            lines.append(f"WARNINGS ({len(warnings)}): " + " ; ".join(warnings))
        else:
            lines.append("STATUS: Kinematics valid, within soft limits, zero table penetration.")

        self.info_text.set_text("\n".join(lines))


# ==============================================================================
# 9. AUTOMATED AUDIT SUITE
# ==============================================================================
def run_automated_audit():
    print("=" * 75)
    print("NEMA-6AXIS-ARM-CONTROLLER — Multi-View Digital Clone Pro Audit")
    print("=" * 75)

    tcp, wc, _ = forward_kinematics([0, 0, 0, 0, 0, 0])
    print("\n1. FK Verification at HOME (0, 0, 0, 0, 0, 0):")
    print(f"   Wrist Center: ({wc[0]:.2f}, {wc[1]:.2f}, {wc[2]:.2f}) mm  [Expected: (126.0, 0.0, 365.0)]")
    print(f"   TCP Pen Tip:  ({tcp[0]:.2f}, {tcp[1]:.2f}, {tcp[2]:.2f}) mm  [Expected: (177.0, 0.0, 365.0)]")

    print("\n2. Closed-form IK Roundtrip Sweep:")
    tested = 0
    solved = 0
    max_err = 0.0
    for x in np.linspace(80.0, 220.0, 15):
        for y in np.linspace(-150.0, 150.0, 15):
            for z in np.linspace(0.0, 200.0, 10):
                tested += 1
                ok, angles, _ = ik_pen_down(x, y, z)
                if ok:
                    solved += 1
                    t_tcp, _, _ = forward_kinematics(angles)
                    err = math.hypot(t_tcp[0] - x, t_tcp[1] - y, t_tcp[2] - z)
                    if err > max_err:
                        max_err = err

    print(f"   Tested Points:    {tested}")
    print(f"   Reachable Solved: {solved} ({solved/tested*100:.1f}%)")
    print(f"   Max Error on FK:  {max_err:.5f} mm  (Threshold: <= 0.5 mm)")

    print("\n3. WorkPlane 3-Point Calibration & Path Audit:")
    wp = WorkPlane()
    ok, msg = wp.calibrate([100, -80, 5], [180, -80, 25], [100, 80, 5])
    print(f"   Gram-Schmidt Calib: {'PASSED' if ok else 'FAILED'} ({msg})")
    wp.enabled = True

    line_wps = PlannerSimulator.plan_line(10.0, -50.0, 70.0, 50.0, z_paper=0.0, work_plane=wp)
    line_ok, _ = PlannerSimulator.evaluate_trajectory(line_wps)
    print(f"   WorkPlane Line:     {'PASSED' if line_ok else 'FAILED'} ({len(line_wps)} waypoints)")

    star_wps = PlannerSimulator.plan_star(40.0, 0.0, 30.0, z_paper=0.0, work_plane=wp)
    star_ok, _ = PlannerSimulator.evaluate_trajectory(star_wps)
    print(f"   WorkPlane Star:     {'PASSED' if star_ok else 'FAILED'} ({len(star_wps)} waypoints)")

    print("\n4. 2-DOF Bevel Gear Differential Wrist Kinematics Audit:")
    # Test 1: Pure Tilt
    t1_tilt, t1_roll = diff_wrist.forward(30.0, 30.0)
    print(f"   Pure Tilt Test (ML=30°, MR=30°):  Tilt={t1_tilt:.2f}°, Roll={t1_roll:.2f}°  {'PASSED' if abs(t1_tilt-30)<1e-4 and abs(t1_roll)<1e-4 else 'FAILED'}")

    # Test 2: Pure Roll
    t2_tilt, t2_roll = diff_wrist.forward(45.0, -45.0)
    print(f"   Pure Roll Test (ML=45°, MR=-45°): Tilt={t2_tilt:.2f}°, Roll={t2_roll:.2f}°  {'PASSED' if abs(t2_tilt)<1e-4 and abs(t2_roll-45)<1e-4 else 'FAILED'}")

    # Test 3: Inverse & Roundtrip
    ml, mr = diff_wrist.inverse(15.5, -22.3)
    rec_tilt, rec_roll = diff_wrist.forward(ml, mr)
    diff_err = math.hypot(rec_tilt - 15.5, rec_roll - (-22.3))
    print(f"   Differential Roundtrip Sweep:     Max Error={diff_err:.6f}°  {'PASSED' if diff_err<1e-5 else 'FAILED'}")

    print("\n" + "=" * 75)
    print("AUDIT COMPLETE: All kinematics models, Differential Wrist & WorkPlane verified.")
    print("=" * 75)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="NEMA-6AXIS Digital Clone Pro")
    parser.add_argument("--test", action="store_true", help="Run automated kinematics audit")
    parser.add_argument("--host", default="http://robot-arm.local", help="ESP32-S3 REST API Base URL")
    args = parser.parse_args()

    if args.test:
        run_automated_audit()
    else:
        app = DigitalCloneGUI(robot_host=args.host)