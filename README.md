# Cessna-182 Longitudinal 3-DOF Flight Dynamics Simulation

A longitudinal three-degree-of-freedom flight dynamics model of the Cessna-182, built in Python. The project covers trimming the aircraft across its speed envelope and simulating its dynamic response to an elevator doublet input.



---

## Overview

The simulation models the aircraft's motion in the vertical plane: forward and vertical velocity, pitch rate and pitch attitude. It uses the aircraft's aerodynamic, inertial and geometric data.

The project has two main parts:

1. **Trim analysis:** trim conditions (angle of attack, elevator deflection, thrust) at **5000 ft** for true airspeeds from **30 to 80 m/s**.
2. **Dynamic response:** a **30-second elevator doublet** simulation starting from trimmed flight at **60 m/s**, showing the aircraft's short-period and phugoid behavior.

---

## Model

**State vector**

| State | Description |
|-------|-------------|
| `u` | Body-axis forward velocity |
| `w` | Body-axis vertical velocity |
| `q` | Pitch rate |
| `θ` | Pitch angle |
| `x`, `h` | Horizontal position and altitude |

**Control inputs:** elevator deflection `δe`, throttle `δT`

**Equations of motion (body axes)**

```
u̇ = X/m − g·sinθ − q·w
ẇ = Z/m + g·cosθ + q·u
q̇ = M / Iyy
θ̇ = q
```

Aerodynamic forces and moments are computed from stability and control derivatives. Atmospheric properties come from the International Standard Atmosphere (ISA).

---

## Workflow

The XDSM diagram below shows how data flows between the model components (atmosphere, aerodynamics, propulsion, equations of motion, trim solver and integrator).



## Results

### Trim envelope (5000 ft, TAS 30–80 m/s)


[Add 1–2 sentences on what the trim curves show, e.g. how angle of attack and elevator deflection change with airspeed.]

### Elevator doublet response (60 m/s trim)



[Add 1–2 sentences: e.g. the short-period mode damps out within X s, and the phugoid oscillation has a period of about Y s.]

---





## Acknowledgments

The XDSM (eXtended Design Structure Matrix) diagram in this project was created with **pyXDSM**, an open-source tool developed by the **MDO Lab at the University of Michigan**, led by **Prof. Joaquim R. R. A. Martins**.

The XDSM notation, which gives a clear and standardized way to describe how data and processes connect in multidisciplinary design, analysis and optimization, was introduced by Prof. Martins and Andrew B. Lambe:

> A. B. Lambe and J. R. R. A. Martins, "Extensions to the design structure matrix for the description of multidisciplinary design, analysis, and optimization processes," *Structural and Multidisciplinary Optimization*, vol. 46, no. 2, pp. 273–284, 2012.

Many thanks to Prof. Martins and the MDO Lab for making this tool freely available to the engineering community.
---

## Author

**Özge İşler**, Aerospace Engineer


---

© 2026 Özge İşler. All rights reserved.

📌 **Using this work?** You're welcome to learn from it — just cite it.
Copying it without credit is not trim-stable behavior. 🛩️
