# Robot Arm Surface‑Tracking Simulator

![](tcp.gif)

This repository contains a Python/Matplotlib animation that demonstrates a **feed‑forward surface‑following strategy** for a robotic tool centre point (TCP).  Two forward‑looking laser range measurements are used to:

1. Estimate the local slope of an unknown surface (gradient).
2. Orient the TCP perpendicular to that surface (normal alignment).
3. Maintain a constant stand‑off distance (`target_offset`) above the surface.

---

## Quick Start

```bash
python -m pip install numpy matplotlib
python tcp_sim.py
```

A window opens showing the surface (black), TCP (red), laser beams (grey, terminating in green hit points) and orientation arrows (blue – tangent, red – normal).

Press `` to enable/disable the moving‑average filter applied to each beam measurement.

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
|     |                                          |

## Underlying mathematics

Below is the minimal geometry used each frame.

| Symbol             | Meaning                                         |
| ------------------ | ----------------------------------------------- |
| (x\_T, z\_T)       | Current TCP coordinate                          |
| (x1, z1), (x2, z2) | Filtered hit points from laser 1 and laser 2    |
| d\_lead^(1,2)      | Sensor lead distances (default 5 mm, 10 mm)     |
| m                  | Local surface slope (dz/dx)                     |
| t^                 | Unit tangent vector                             |
| n^                 | Unit surface normal (points toward the surface) |
| z\_cmd             | Commanded TCP height                            |
| h\_off             | Desired standoff distance (5 mm)                |

### Local slope

The surface is assumed locally linear, so a centred difference gives the gradient:

```
 m = (z2 - z1) / (x2 - x1)
```

### Tangent and normal vectors

```
 t_h = (1,  m) / sqrt(1 + m^2)
 n_h = (m, -1) / sqrt(1 + m^2)   # flipped so z-component is negative
```

### Predicted surface height under the TCP

Using the first hit point and the slope:

```
 z_pred = z1 + m * (x_T - x1)
```

### Height command

```
 z_cmd = z_pred + h_off
```
