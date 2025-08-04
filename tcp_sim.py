"""
Two-laser preview controller demo
Keeps a TCP perpendicular to – and a fixed
distance above – a wavy surface.

Run:  python tcp_sim.py
Press m:  toggle moving-average smoothing
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ------------------------------------------------------------------#
# 1.  User-tweakable settings
# ------------------------------------------------------------------#
SURFACE_A      = 5                 # sine amplitude
SURFACE_K      = 0.1               # sine frequency
TARGET_OFFSET  = 5                 # mm above surface
SENSOR_LEAD_MM = (5, 10)           # two look-ahead distances
STEP_SIZE      = 0.5               # mm per animation frame
NOISE_STD      = 0.2               # beam noise (mm)
SMOOTH_WINDOW  = 5                 # moving-average length
ARROW_LEN      = 3                 # graphic length
MAX_FRAMES     = 200

# ------------------------------------------------------------------#
# 2.  Helper functions
# ------------------------------------------------------------------#
def height(x: float) -> float:
    """Surface height function."""
    return SURFACE_A * np.sin(SURFACE_K * x) + 10


def cast_ray(x0, z0, dx, dz, step=0.1, max_len=50):
    """March a ray until it intersects the surface."""
    for r in np.arange(0, max_len, step):
        x, z = x0 + dx * r, z0 + dz * r
        if z <= height(x):
            return x, height(x), r
    # If nothing found (shouldn’t happen) return max range point
    return x0 + dx * max_len, z0 + dz * max_len, max_len


# ------------------------------------------------------------------#
# 3.  Matplotlib boilerplate
# ------------------------------------------------------------------#
fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(-5, 100)
ax.set_ylim(0, 30)
ax.grid(True)
ax.set_xlabel("X")
ax.set_ylabel("Z")

x_plot = np.linspace(0, 120, 1_000)
ax.plot(x_plot, height(x_plot), color="black", label="Surface")

tcp_point,   = ax.plot([], [], "ro", label="TCP")
hit_points,  = ax.plot([], [], "go", label="Laser hits")
tcp_path,    = ax.plot([], [], "r--", lw=0.6, label="TCP path")
beam1_line,  = ax.plot([], [], "gray", ls="--")
beam2_line,  = ax.plot([], [], "gray", ls="--")

normal_arrow  = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                            arrowprops=dict(arrowstyle="->", color="red"))
tangent_arrow = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                            arrowprops=dict(arrowstyle="->", color="blue"))

title = ax.set_title("TCP Tracking  —  Smoothing: ON")
ax.text(0.02, 0.97, "press  m  to toggle smoothing", transform=ax.transAxes,
        ha="left", va="top", fontsize=9, color="gray")

stats = {k: ax.text(0.02, 0.93 - i * 0.04, "",
         transform=ax.transAxes, fontsize=9, ha="left")
         for i, k in enumerate(["Gradient",
                               "Distance error",
                               "Sensor variance",
                               "Beam-1 length",
                               "Beam-2 length"])}

# ------------------------------------------------------------------#
# 4.  Live-toggle for smoothing
# ------------------------------------------------------------------#
use_smoothing = True
def toggle_key(event):
    global use_smoothing
    if event.key == "m":
        use_smoothing = not use_smoothing
        flag = "ON" if use_smoothing else "OFF"
        title.set_text(f"TCP Tracking  —  Smoothing: {flag}")
fig.canvas.mpl_connect("key_press_event", toggle_key)

# ------------------------------------------------------------------#
# 5.  Animation state
# ------------------------------------------------------------------#
tcp_x = 0.0
history_x, history_z = [], []
buf1, buf2 = [], []  # moving-average buffers

# ------------------------------------------------------------------#
# 6.  Frame update
# ------------------------------------------------------------------#
def update(frame):
    global tcp_x

    # Reset every loop
    if frame == 0:
        tcp_x = 0
        history_x.clear(); history_z.clear()
        buf1.clear(); buf2.clear()

    # 6-1  Local surface slope from adjacent samples
    slope_raw = (height(tcp_x + 0.1) - height(tcp_x - 0.1)) / 0.2
    theta_tan = np.arctan(slope_raw)
    t_vec     = np.array([np.cos(theta_tan), np.sin(theta_tan)])

    # 6-2  TCP position
    tcp_z = height(tcp_x) + TARGET_OFFSET
    tcp   = np.array([tcp_x, tcp_z])

    # 6-3  Sensor bodies along tangent
    s1 = tcp + t_vec * SENSOR_LEAD_MM[0]
    s2 = tcp + t_vec * SENSOR_LEAD_MM[1]

    # 6-4  Ray-cast DOWN normal
    n_vec = np.array([ t_vec[1], -t_vec[0] ])   # rotate 90° CW
    if n_vec[1] > 0: n_vec *= -1                # ensure downward

    x1, z1, d1 = cast_ray(*s1, *n_vec)
    x2, z2, d2 = cast_ray(*s2, *n_vec)

    # 6-5  Sensor noise ➜ optional smoothing
    z1 += np.random.normal(0, NOISE_STD)
    z2 += np.random.normal(0, NOISE_STD)

    buf1.append(z1); buf2.append(z2)
    if len(buf1) > SMOOTH_WINDOW: buf1.pop(0); buf2.pop(0)
    z1_f = np.mean(buf1) if use_smoothing else z1
    z2_f = np.mean(buf2) if use_smoothing else z2

    # 6-6  Gradient from filtered hits
    slope = (z2_f - z1_f) / (x2 - x1)
    theta_tan = np.arctan(slope)
    t_vec = np.array([np.cos(theta_tan), np.sin(theta_tan)])
    n_vec = np.array([ t_vec[1], -t_vec[0] ])
    if n_vec[1] > 0: n_vec *= -1

    # 6-7  Re-compute sensor bodies and intersections with new tangent
    s1 = tcp + t_vec * SENSOR_LEAD_MM[0]
    s2 = tcp + t_vec * SENSOR_LEAD_MM[1]
    x1, z1, d1 = cast_ray(*s1, *n_vec)
    x2, z2, d2 = cast_ray(*s2, *n_vec)

    # 6-8  Feed-forward height
    z_pred = z1_f + slope * (tcp_x - x1)
    tcp_z = z_pred + TARGET_OFFSET
    tcp   = np.array([tcp_x, tcp_z])

    # ------------- draw everything --------------
    tcp_point.set_data(*tcp)
    hit_points.set_data([x1, x2], [z1, z2])
    history_x.append(tcp_x); history_z.append(tcp_z)
    tcp_path.set_data(history_x, history_z)

    normal_arrow.xy = tcp + n_vec * ARROW_LEN
    normal_arrow.set_position(tcp)
    tangent_arrow.xy = tcp + t_vec * ARROW_LEN
    tangent_arrow.set_position(tcp)

    beam1_line.set_data([s1[0], x1], [s1[1], z1])
    beam2_line.set_data([s2[0], x2], [s2[1], z2])

    # ------------- HUD text ---------------------
    err   = tcp_z - height(tcp_x) - TARGET_OFFSET
    var   = np.var(buf1 + buf2) if buf1 else 0
    stats["Gradient"].set_text(f"Gradient       : {slope: .4f}")
    stats["Distance error"].set_text(f"Distance error : {err: .3f}")
    stats["Sensor variance"].set_text(f"Sensor variance: {var: .4f}")
    stats["Beam-1 length"].set_text(f"Beam-1 length  : {d1: .2f}")
    stats["Beam-2 length"].set_text(f"Beam-2 length  : {d2: .2f}")

    # Advance TCP
    tcp_x += STEP_SIZE
    return (tcp_point, hit_points, tcp_path,
            beam1_line, beam2_line,
            normal_arrow, tangent_arrow,
            *stats.values())

ani = animation.FuncAnimation(fig, update, frames=MAX_FRAMES,
                              interval=50, blit=True, repeat=True)
ax.legend(loc="upper right")
plt.tight_layout()
plt.show()
