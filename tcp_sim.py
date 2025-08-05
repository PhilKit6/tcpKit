"""
Rigid-mount two-laser preview demo (2-D) with automatic reset at end of surface.

Run:   python tcp_rigid_mount_reset.py
Press: m  (toggle moving-average smoothing)
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import patches

# ---------------------- user settings ---------------------- #
SURFACE_A, SURFACE_K = 5.0, 0.1      # sine surface: A*sin(Kx)+10
TARGET_OFFSET = 5.0                  # TCP standoff (mm)
SENSOR_LEAD_MM = (5.0, 10.0)         # rigid lead distances along tangent
STEP_SIZE = 0.5                      # mm per frame (forward X motion)
NOISE_STD = 0.2                      # beam noise (mm)
SMOOTH_WINDOW = 5                    # moving-average window
ARROW_LEN = 3.0
MAX_FRAMES = 200

# Visual geometry
TCP_RADIUS = 1.0
SENSOR_BODY_LEN = 4.0
LINK_WIDTH = 1.5

# Plot range / reset bounds
X_START = 0.0
X_END   = 100.0                      # reset when TCP goes beyond this

# ---------------------- helpers ---------------------------- #
def height(x: float) -> float:
    return SURFACE_A * np.sin(SURFACE_K * x) + 10.0

def cast_ray(x0, z0, dx, dz, step=0.1, max_len=50.0):
    """March a ray from (x0,z0) along (dx,dz) until z <= surface(x)."""
    for r in np.arange(0.0, max_len, step):
        x = x0 + dx * r
        z = z0 + dz * r
        if z <= height(x):
            return x, height(x), r
    return x0 + dx * max_len, z0 + dz * max_len, max_len

def seg_endpoints(center, unit_dir, length):
    """Endpoints of a segment centered at 'center' with direction 'unit_dir'."""
    half = unit_dir * (length / 2.0)
    return center - half, center + half

def unit_from_slope(m):
    """Unit tangent & downward unit normal from slope m."""
    t = np.array([1.0, m], dtype=float)
    t /= np.linalg.norm(t)
    n = np.array([ t[1], -t[0] ], dtype=float)   # 90° CW of tangent
    if n[1] > 0:  # ensure normal points toward surface (down)
        n *= -1.0
    return t, n

# ---------------------- figure setup ----------------------- #
fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(-5, X_END)
ax.set_ylim(0, 30)
ax.grid(True)
ax.set_xlabel("X")
ax.set_ylabel("Z")

x_plot = np.linspace(X_START, X_END, 1000)
ax.plot(x_plot, height(x_plot), color="black", label="Surface")

# TCP & path
tcp_dot,  = ax.plot([], [], "ro", label="TCP")
tcp_path, = ax.plot([], [], "r--", lw=0.6, label="TCP path")
tcp_circle = patches.Circle((0, 0), TCP_RADIUS, fill=False, ec="red", lw=1.2)
ax.add_patch(tcp_circle)

# Orientation arrows
norm_arrow  = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                          arrowprops=dict(arrowstyle="->", color="red"))
tan_arrow   = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                          arrowprops=dict(arrowstyle="->", color="blue"))

# Rigid links + sensor bodies
link1_line, = ax.plot([], [], color="k", lw=LINK_WIDTH)
link2_line, = ax.plot([], [], color="k", lw=LINK_WIDTH)
sens1_body, = ax.plot([], [], color="k", lw=3)
sens2_body, = ax.plot([], [], color="k", lw=3)

# Beams + hits
beam1_line, = ax.plot([], [], "gray", ls="--")
beam2_line, = ax.plot([], [], "gray", ls="--")
hits,       = ax.plot([], [], "go", label="Laser hits")

# HUD
title = ax.set_title("TCP Tracking  —  Smoothing: ON")
ax.text(0.02, 0.97, "press  m  to toggle smoothing", transform=ax.transAxes,
        ha="left", va="top", fontsize=9, color="gray")
labels = ["Gradient", "Distance error", "Sensor variance", "Beam-1 length", "Beam-2 length"]
hud = {k: ax.text(0.02, 0.93 - i * 0.04, "", transform=ax.transAxes, fontsize=9, ha="left")
       for i, k in enumerate(labels)}

# ---------------------- state (causal) --------------------- #
use_smoothing = True
buf1, buf2 = [], []              # moving-average buffers

tcp_x = 0.0
tcp_z = height(tcp_x) + TARGET_OFFSET
t_vec_k, n_vec_k = unit_from_slope((height(tcp_x + 0.1) - height(tcp_x - 0.1)) / 0.2)

next_tcp_x = tcp_x
next_tcp_z = tcp_z
t_vec_next = t_vec_k
n_vec_next = n_vec_k

hist_x, hist_z = [tcp_x], [tcp_z]

def reset_state():
    """Reset everything to the left/start once we traverse the surface."""
    global buf1, buf2, tcp_x, tcp_z, t_vec_k, n_vec_k
    global next_tcp_x, next_tcp_z, t_vec_next, n_vec_next, hist_x, hist_z
    buf1.clear(); buf2.clear()
    hist_x.clear(); hist_z.clear()
    tcp_x = X_START
    tcp_z = height(tcp_x) + TARGET_OFFSET
    t_vec_k, n_vec_k = unit_from_slope((height(tcp_x + 0.1) - height(tcp_x - 0.1)) / 0.2)
    next_tcp_x, next_tcp_z = tcp_x, tcp_z
    t_vec_next, n_vec_next = t_vec_k, n_vec_k
    hist_x.append(tcp_x); hist_z.append(tcp_z)

# ---------------------- controls --------------------------- #
def on_key(evt):
    global use_smoothing
    if evt.key == "m":
        use_smoothing = not use_smoothing
        title.set_text(f"TCP Tracking  —  Smoothing: {'ON' if use_smoothing else 'OFF'}")
fig.canvas.mpl_connect("key_press_event", on_key)

# ---------------------- animation update ------------------- #
def update(frame):
    global tcp_x, tcp_z, t_vec_k, n_vec_k
    global next_tcp_x, next_tcp_z, t_vec_next, n_vec_next
    global buf1, buf2

    # 1) Use current pose_k to place rigid sensors
    tcp = np.array([tcp_x, tcp_z])
    s1  = tcp + t_vec_k * SENSOR_LEAD_MM[0]
    s2  = tcp + t_vec_k * SENSOR_LEAD_MM[1]

    # 2) Beams along current normal_k
    x1, z1, d1 = cast_ray(s1[0], s1[1], n_vec_k[0], n_vec_k[1])
    x2, z2, d2 = cast_ray(s2[0], s2[1], n_vec_k[0], n_vec_k[1])

    # 3) Noise + optional smoothing
    z1m = z1 + np.random.normal(0, NOISE_STD)
    z2m = z2 + np.random.normal(0, NOISE_STD)
    buf1.append(z1m); buf2.append(z2m)
    if len(buf1) > SMOOTH_WINDOW:
        buf1.pop(0); buf2.pop(0)
    z1_f = np.mean(buf1) if use_smoothing else z1m
    z2_f = np.mean(buf2) if use_smoothing else z2m

    # 4) Estimate gradient from filtered hits → pose_{k+1}
    denom = (x2 - x1) if (x2 - x1) != 0 else 1e-9
    slope = (z2_f - z1_f) / denom
    t_vec_next, n_vec_next = unit_from_slope(slope)

    # Predicted surface under current tcp_x using fitted line
    z_pred_at_tcp = z1_f + slope * (tcp_x - x1)
    next_tcp_z = z_pred_at_tcp + TARGET_OFFSET
    next_tcp_x = tcp_x + STEP_SIZE

    # 5) Draw pose_k
    tcp_dot.set_data(tcp_x, tcp_z)
    tcp_circle.center = (tcp_x, tcp_z)
    hist_x.append(tcp_x); hist_z.append(tcp_z)
    tcp_path.set_data(hist_x, hist_z)

    norm_arrow.xy = (tcp_x + n_vec_k[0] * ARROW_LEN, tcp_z + n_vec_k[1] * ARROW_LEN)
    norm_arrow.set_position((tcp_x, tcp_z))
    tan_arrow.xy  = (tcp_x + t_vec_k[0] * ARROW_LEN, tcp_z + t_vec_k[1] * ARROW_LEN)
    tan_arrow.set_position((tcp_x, tcp_z))

    # Rigid links and sensor bodies (bars along tangent_k)
    sb1_p0, sb1_p1 = seg_endpoints(s1, t_vec_k, SENSOR_BODY_LEN)
    sb2_p0, sb2_p1 = seg_endpoints(s2, t_vec_k, SENSOR_BODY_LEN)
    sens1_body.set_data([sb1_p0[0], sb1_p1[0]], [sb1_p0[1], sb1_p1[1]])
    sens2_body.set_data([sb2_p0[0], sb2_p1[0]], [sb2_p0[1], sb2_p1[1]])
    link1_line.set_data([tcp[0], s1[0]], [tcp[1], s1[1]])
    link2_line.set_data([tcp[0], s2[0]], [tcp[1], s2[1]])

    # Beams and hits
    beam1_line.set_data([s1[0], x1], [s1[1], z1])
    beam2_line.set_data([s2[0], x2], [s2[1], z2])
    hits.set_data([x1, x2], [z1, z2])

    # HUD
    err = tcp_z - height(tcp_x) - TARGET_OFFSET
    var = np.var(buf1 + buf2) if buf1 else 0.0
    hud["Gradient"].set_text(f"Gradient       : {slope: .4f}")
    hud["Distance error"].set_text(f"Distance error : {err: .3f}")
    hud["Sensor variance"].set_text(f"Sensor variance: {var: .4f}")
    hud["Beam-1 length"].set_text(f"Beam-1 length  : {d1: .2f}")
    hud["Beam-2 length"].set_text(f"Beam-2 length  : {d2: .2f}")

    # 6) Advance to pose_{k+1} or reset if beyond X_END
    if next_tcp_x >= X_END:
        reset_state()
    else:
        tcp_x, tcp_z = next_tcp_x, next_tcp_z
        t_vec_k, n_vec_k = t_vec_next, n_vec_next

    return (tcp_dot, tcp_path, tcp_circle,
            norm_arrow, tan_arrow,
            link1_line, link2_line, sens1_body, sens2_body,
            beam1_line, beam2_line, hits,
            *hud.values())

ani = animation.FuncAnimation(fig, update, frames=MAX_FRAMES,
                              interval=50, blit=True, repeat=True)
ax.legend(loc="upper right")
plt.tight_layout()
plt.show()
