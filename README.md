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
|     |                                          |

## Underlying mathematics

This section summarises the geometry used each frame and makes the **rigid sensor mounting** explicit.

### Symbols

| Symbol | Meaning |
|---|---|
| $(x_T, z_T)$ | TCP position in the world $(x\!-\!z)$ plane |
| $\hat{t}_k,\;\hat{n}_k$ | Unit **tangent** and **downward normal** at frame $k$ |
| $d_1, d_2$ | Rigid leads from TCP to sensor 1 and 2 along $\hat{t}_k$ (e.g., 5 mm, 10 mm) |
| $\mathbf{s}_{i,k}$ | Sensor-$i$ body position at frame $k$ |
| $(x_i, z_i)$ | Filtered hit point of beam $i$ on the surface |
| $m_k$ | Local surface slope (gradient) at frame $k$ |
| $h_{\text{off}}$ | Desired stand-off (e.g., 5 mm) |
| $z_{\text{cmd},k}$ | Commanded TCP height |

---

### 1) Local line fit from two hits

Given filtered hits $(x_1,z_1)$ and $(x_2,z_2)$ with $x_2>x_1$:
\[
 z(x) = a x + b,\qquad
 a = \frac{z_2 - z_1}{x_2 - x_1},\qquad
 b = z_1 - a x_1.
\]
The **gradient** is $m_k \equiv a$.

---

### 2) Tangent and (downward) normal

\[
 \hat{t}_k = \frac{(1,\,m_k)}{\sqrt{1+m_k^2}}, \qquad
 \hat{n}_k = \frac{(m_k,\,-1)}{\sqrt{1+m_k^2}}, \;\; \text{with } (\hat{n}_k)_z<0.
\]

---

### 3) Rigid sensor kinematics (TCP frame → world)

Sensors are fixed on the tool along the tangent at frame $k$:
\[
 \mathbf{s}_{i,k} =
 \begin{bmatrix} x_T \\ z_T \end{bmatrix}
 + d_i\,\hat{t}_k, \qquad i\in\{1,2\}.
\]
Each beam is emitted **perpendicular to the tangent**, i.e. along $\hat{n}_k$.

---

### 4) Beam–surface intersection (closed-form on the fitted line)

Beam from sensor $i$:
\[
 \mathbf{p}_i(r) = \mathbf{s}_{i,k} + r\,\hat{n}_k,\quad r\ge 0.
\]
Intersect with $z=ax+b$:
\[
 r_i^* = \frac{a\,s_{i,x} + b - s_{i,z}}{\,n_{k,z} - a\,n_{k,x}\,}, \qquad
 (x_i, z_i) = \mathbf{p}_i(r_i^*).
\]
(We also support a numeric march in code; the formula above is exact for the local line.)

---

### 5) Feed-forward height and orientation (causal update)

Using the line fitted at frame $k$:
\[
 z_{\text{pred},k} = a\,x_T + b,\qquad
 z_{\text{cmd},k+1} = z_{\text{pred},k} + h_{\text{off}},\qquad
 (\hat{t}_{k+1},\hat{n}_{k+1}) \leftarrow m_k.
\]
The next frame uses the slope from the current beams, keeping the sensor mounting rigid during each frame.

---

### 6) Distance error (for evaluation or feedback)

\[
 e_k = z_T - \bigl( z_{\text{true}}(x_T) + h_{\text{off}} \bigr).
\]

---

### 7) Optional filters

**Moving average (window $N$):**
\[
 z^{\text{MA}}[k] = \tfrac{1}{N}\sum_{i=0}^{N-1} z[k-i], \qquad
 \text{group delay} \approx \tfrac{N-1}{2}\text{ frames}.
\]

**Exponential (EMA):**
\[
 z^{\text{EMA}}[k] = \alpha\,z[k] + (1-\alpha)\,z^{\text{EMA}}[k-1], \quad 0<\alpha<1.
\]

---

### 8) Curvature note

For true surface $z=f(x)$ with curvature
\[
 \kappa(x) = \frac{|f''(x)|}{\bigl(1+f'(x)^2\bigr)^{3/2}},
\]
and chord length $L=x_2-x_1$, the maximum deviation between arc and chord (your line fit) is approximately
\[
 \delta_{\max} \approx \frac{\kappa\,L^2}{8}.
\]
This sets a bound on stand-off accuracy unless you shorten the lead or add more preview points.
