# Robot Arm Surface‑Tracking Simulator

![](tcp.gif)


This repository contains a Python/Matplotlib animation that demonstrates a **feed‑forward surface‑following strategy** for a robotic tool centre point (TCP).  Two forward‑looking laser range measurements are used to:

1. Estimate the local slope of an unknown surface (gradient).
2. Orient the TCP perpendicular to that surface (normal alignment).
3. Maintain a constant stand‑off distance (`target_offset`) above the surface.

The sensors are modeled as **rigidly mounted** to the TCP: their positions are a fixed transform in the TCP frame (constant lead along the tangent), and beams are emitted along the tool’s current **downward normal**. The pose estimated from the current beams is applied **next frame** (causal update), so geometry is rigid within each frame. The sim also auto‑resets after traversing the plotting range.

---

## Quick Start

```bash
python -m pip install numpy matplotlib
python tcp_sim.py
```

A window opens showing the surface (black), TCP (red), laser beams (grey, terminating in green hit points) and orientation arrows (blue – tangent, red – normal).

Press `m` to enable/disable the moving‑average filter applied to each beam measurement.

---

## Algorithm Overview

| Stage                   | Description                                                                                                                                                                                          |
| ----------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Preview sensing**     | Two laser beams are positioned ahead of the TCP along the current surface tangent.  Each beam is projected along the surface normal direction to intersect the surface and returns a height reading. |
| **Gradient estimation** | A straight line is fitted through the two hit points to compute the local surface gradient.                                                                                                          |
| **Orientation**         | The TCP is rotated so its tool axis is aligned with the calculated surface normal.                                                                                                                   |
| **Height command**      | The fitted line is evaluated at the TCP’s current *x*‑coordinate; the TCP *z*‑coordinate is then commanded to `z_surface + target_offset`.                                                           |
| **Filtering**           | An optional moving‑average filter (toggle with `m`) attenuates sensor noise.                                                                                                                         |

---

## Controls

| Key | Function                                 |
| --- | ---------------------------------------- |
| `m`  | Toggle the moving‑average filter on/off. |

## Underlying mathematics

### Symbols

| Symbol | Meaning |
| --- | --- |
| `x_T, z_T` | TCP position in the world `(x‑z)` plane |
| `t_hat_k, n_hat_k` | Unit **tangent** and **downward normal** at frame `k` |
| `d1, d2` | Rigid leads from TCP to sensors 1 and 2 along `t_hat_k` (e.g., 5 mm and 10 mm) |
| `s_i_k` | Sensor‑`i` body position at frame `k` |
| `x1, z1 ; x2, z2` | Filtered beam hit points |
| `a` | Local surface slope (gradient) at frame `k` |
| `b` | Intercept of the local line at frame `k` |
| `h_off` | Desired stand‑off distance |
| `z_cmd_k` | Commanded TCP height at frame `k` |

### 1) Local line fit from two hits

```
a  = (z2 - z1) / (x2 - x1)      # slope (gradient)
b  = z1 - a * x1                # intercept
z(x) = a * x + b
```

### 2) Tangent and (downward) normal

```
t_hat_k = (1,  a) / sqrt(1 + a*a)
n_hat_k = (a, -1) / sqrt(1 + a*a)    # choose sign so n_hat_k.z < 0 (toward surface)
```

### 3) **Rigid sensor kinematics** (TCP frame → world)
Sensors are **rigidly mounted**: their positions are a fixed transform from the TCP along the current tangent.

```
s_1_k = [x_T, z_T] + d1 * t_hat_k
s_2_k = [x_T, z_T] + d2 * t_hat_k
```

Beams are emitted **perpendicular** to the tangent (i.e., along `n_hat_k`).

### 4) Beam–surface intersection (closed‑form on the fitted line)
Parameterise the beam from sensor `i`:

```
p_i(r) = s_i_k + r * n_hat_k,   r >= 0
```

Intersect with the local line `z(x) = a*x + b`.  Write `n_hat_k = (n_x, n_z)` and `s_i_k = (s_x, s_z)`.

```
r_i*   = (a*s_x + b - s_z) / (n_z - a*n_x)
(x_i, z_i) = p_i(r_i*)
```

> The simulator also supports a simple numeric march to the surface; the formula above is exact for the local line representation.

### 5) Feed‑forward height and orientation (**causal update**)
Use the line fitted at frame `k` to command the **next** frame, keeping the geometry rigid within each frame:

```
z_pred_k    = a * x_T + b
z_cmd_{k+1} = z_pred_k + h_off
(t_hat_{k+1}, n_hat_{k+1})  <-  slope a
```

### 6) Distance error (for evaluation or feedback)

```
e_k = z_T - ( z_true(x_T) + h_off )
```

RMS/percentile statistics of `e_k` are useful to compare filters and controller settings.




