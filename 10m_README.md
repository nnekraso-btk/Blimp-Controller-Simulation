# blimpFormation — 10 m Configuration
### WVU Custom Airship Leader-Follower Formation Control
*Based on Sponaugle 2025 MSME Thesis — extended from the 4 m lab-scale baseline*

---

## Overview

This file implements a 6-DOF nonlinear simulation of two WVU custom airships (Park 250 thrusters) performing leader-follower formation control. The follower (B) autonomously tracks a position 5.5 m behind the leader (A) using a CLF-CBF quadratic program safety filter, a layered communication-zone architecture, and a disturbance model.

The 10 m configuration scales the communication zones to match the WVU Vicon lab footprint and introduces a series of controller improvements validated against outward, inward, and randomized disturbances.

---

## Physical Model

All parameters sourced from Sponaugle 2025, Tables 6.2, 6.3, 6.7.

| Parameter | Value | Source |
|---|---|---|
| Rigid body mass | 0.460 kg | Table 6.3 |
| Surge added mass | m_surge = 0.5691 kg | Table 6.3 |
| Sway/heave added mass | m_sway = 0.7720 kg | Table 6.3 |
| Yaw inertia | Iz = 0.1070 kg·m² | Table 6.3 |
| Surge drag | Xu = 0.190 kg/s | Table 6.7 |
| Sway drag | Yv = 0.752 kg/s | Table 6.7 |
| Yaw drag | Nr = 0.245 kg·m²/s | Table 6.7 |
| Motor separation | ly = 0.461 m | Table 6.2 |
| Max thrust (Park 250 @ PWM 1800) | F_max = 1.07 N | Table 6.2 |
| Semi-major axis | a = 0.918 m | Table 6.2 |

> **Note:** Semi-axes `P.a = 0.918 m`, `P.b = P.c = 0.372 m` per Table 6.2. Visual only — physics uses added-mass parameters directly.

**Equations of motion** (body frame, state `[x, y, z, ψ, u, v, w, r]`):
```
du = (Fx + m·v·r + dist_surge − Xu·u) / m_surge
dv = (−m·u·r + dist_sway  − Yv·v) / m_sway
dw = (Fz − Zw·w) / m_heave
dr = (Mz − Nr·r) / Iz
dx = u·cos(ψ) − v·sin(ψ)
dy = u·sin(ψ) + v·cos(ψ)
```

---

## Communication Zone Architecture (10 m scale)

| Zone | Range | Label | Behaviour |
|---|---|---|---|
| DANGER CLOSE | d < 3.0 m | CRIT | Inner CBF hard boundary |
| TOO CLOSE | 3.0–4.0 m | WARN | Inner warning |
| IDEAL | 4.0–6.25 m | OK | Normal formation following |
| TOO FAR | 6.25–7.5 m | WARN | Outer warning; arc correction fades |
| OUTER WARN | 7.5–10.0 m | WARN | Layer 2 active; ldr_scale reduces A's speed |
| COMM LOST | d > 10.0 m | CRIT | Layer 3 active; A stops; B navigates to last known A |

---

## Controller Architecture

### CLF Target Selection
The desired position `(xd, yd)` for the CLF is selected in priority order:

1. **Layer 3 (comm lost):** B navigates to the last known A position frozen at comm loss.
2. **Outer zone + outside cone:** B is redirected to the nearest cone-edge point at formation distance (`follow_dist = 5.5 m` from A at ±60°), avoiding the path-through-A geometry that caused tangential latching at the yellow ring.
3. **Inner CBF circumnavigation:** When d < 4.5 m, the CLF target blends toward an orbit point (4.5 m from A) advancing 30% per tick toward the behind-tail angle. CBF pushes radially outward; CLF pulls tangentially — cooperative curved escape.
4. **Normal:** Fixed behind-tail point at `A − follow_dist·(cos(ψ_A), sin(ψ_A))`.

### Heading PD with Coriolis Feedforward
```
β       = atan2(v, max(0.05, |u|))          % crab angle
ψ_des   = atan2(ey, ex) − β                 % crab-adjusted bearing
ε_ψ     = wrap(ψ_des − ψ)
β_dot   = v_dot_ff / max(0.05, |u|)         % Coriolis feedforward
Mz      = Kp_yaw·ε_ψ − Kd_yaw·r − Kff_yaw·β_dot
```

**Mz alignment scale** (outer zone only): When `d > comm_ideal_hi`, Mz is scaled by `(1 − |nose_r|)` where `nose_r = (dx·cos(ψ) + dy·sin(ψ)) / d`.
- Aligned (|nose_r| → 1): Mz → 0; full motor authority goes to surge for inward recovery.
- Perpendicular (|nose_r| → 0): full Mz to turn nose toward A before surging.

### Motor Allocation
```
F_surge  = Fx_safe·cos(ψ) + Fy_safe·sin(ψ)    % CBF inertial force projected onto nose
F1 = clamp((F_surge + Mz/ly)/2, −F_max, F_max)
F2 = clamp((F_surge − Mz/ly)/2, −F_max, F_max)
inp.Fx = F1 + F2
```

### Controller Gains

| Gain | Value | Role |
|---|---|---|
| Kp | 3.50 N/m | Position proportional |
| Kd | 3.40 N/(m/s) | Velocity derivative |
| Kff | 0.55 N/(m/s) | Velocity feedforward (fades via kff_scale) |
| Kp_yaw | 4.50 | Heading proportional |
| Kd_yaw | 0.60 | Yaw rate damping |
| Kff_yaw | 0.30 | Coriolis yaw feedforward |

---

## CLF-CBF QP Safety Filter

**Variables:** `[ax, ay, δ_clf]` — 2D inertial acceleration + CLF slack.  
**Solver:** MATLAB `quadprog`, active-set algorithm, `MaxIterations = 200`, z0 perturbed 0.01 inside bounds to prevent bound-face degenerate start.

### Barrier Functions
```
h1 = d² − d_min²          (inner: prevents collision, d_min = 3.0 m)
h2 = d_max² − d²           (outer: prevents comm loss, d_max = 10.0 m)
```

### Inner Barrier RHS
The inner CBF constraint is **only activated when `h1 < cbf_h1_thresh`** (d < 4.5 m). When d ≥ 4.5 m, `rhs1 = 1e6` (no constraint). This prevents the `dist_margin_inner` term — which scales linearly with d — from generating a spurious outward force at large separations where the inner boundary poses no threat.

```
rhs1 = 2·dv² − 2·(dx·aAx + dy·aAy) − 2·a_unc_dp
       − dist_margin_inner + (α1+α2)·ḣ1 + α1·α2·h1
```

**Alpha coordination:** `α_eff = α_base × outer_margin_frac²` where  
`outer_margin_frac = (d_max − d) / (d_max − d_min)`.  
Inner correction energy fades quadratically toward zero as B approaches the outer boundary, preventing inner CBF ejection from overshooting the outer boundary.

### Outer Barrier RHS
```
rhs2 = −2·dv² + 2·(dx·aAx + dy·aAy) + 2·a_unc_dp
       − dist_margin_outer − tang_pen + (β1+β2)·ḣ2 + β1·β2·h2
```

### Infeasibility Fallback (direction-aware)
| Condition | Action |
|---|---|
| h2 < 0 (outside outer boundary) | Max inward thrust: `ax = −a_max·dx_u` |
| h1 < 0 (inside inner boundary) | Max outward thrust: `ax = +a_max·dx_u` |
| Both satisfied (numerical failure) | Apply nominal `ax_nom, ay_nom` |

### Passthrough
When `h1 > h1_thresh` AND `h2 > h2_thresh` AND `~broadside_threat`: return nominal force unmodified.

**Broadside threat:** `|nose_r| < 0.50` AND `d > 5.0 m` — CBF activates early when motor alignment with the radial is poor.

---

## Layered Architecture

### Layer 1 — Follower handles alone
Normal CBF + CLF operation. kff_scale, Mz alignment, arc correction, circumnavigation.

### Layer 2 — Predictive leader speed scaling
```
d_pred = d_now + v_out·T2 + 0.5·a_wc·T2²      (T2 = 1.0 s, a_wc = F_dist/m_sway)
ldr_scale = clamp(1 − (d_pred − d_yellow) / (d_max − d_yellow), 0, 1)
inp_A.Fx = ldr_scale·inp_A.Fx + (1−ldr_scale)·(−Kbrake·u_A)
```
A brakes proportionally as B's predicted position approaches the outer boundary. Full stop when `d_pred ≥ d_max`.

### Layer 3 — Comm lost
- **Entry:** `d > d_max`. Timer `t_comm_lost` stamped once per event.
- **Hysteresis exit:** `comm_lost` only clears when `d < d_max − 1.5 m` (prevents timer reset from ±0.01 m boundary oscillation).
- **HOLD (only mode currently implemented):** A stops. B navigates to frozen `last_A_x/y/ψ`.
- **Timeout (10 s):** A resumes at `layer3_resume_scale = 0.30`.

> **Pending:** `P.recovery_mode` parameter with HOLD / CONTINUE_REDUCED / DEADRECKON pilot-selectable modes not yet implemented.

---

## Disturbance Model

```
P.dist_mag       = 0.40 N       % force magnitude
P.dist_direction = 1            % +1 outward, -1 inward, 0 randomized
P.dist_t_on      = 5.0 s        % active duration per cycle
P.dist_t_off     = 4.0 s        % calm recovery window
P.dist_t_start   = 8.0 s        % delay before first disturbance
```

**Randomized direction (`dist_direction = 0`):** A random unit vector is drawn once at the rising edge of each ON cycle and held constant for its duration — consistent with a real gust having a fixed direction during its event. The HUD DIST field shows the angle in degrees (e.g., `ON −31°`).

---

## Improvements from 4 m Baseline

The following were developed and validated in the 10 m version. Items marked † are scale-independent bugs/improvements applicable to the 4 m file as well.

| # | Change | Impact |
|---|---|---|
| 1 | **Inner CBF constraint deactivated at h1 ≥ h1_thresh** † | Root-cause fix: `dist_margin_inner` scaling with d was generating a spurious outward force at all separations, latching B at the yellow ring and preventing inward recovery |
| 2 | **Mz alignment scale `(1 − \|nose_r\|)`** † | Restores full surge authority when nose is aligned with radial; full yaw authority when perpendicular — eliminates tangential deadlock at outer boundary |
| 3 | **Cone-edge CLF target** † | When B is outside the ±60° cone AND outside the ideal zone, redirects CLF to nearest cone-edge point at formation distance, avoiding path-through-A geometry |
| 4 | **Arc correction hard cutoff at `comm_ideal_hi`** † | Zero arc correction when d > 6.25 m; gradual fade was creating a partial-active boundary that latched B on a tangential arc at the yellow ring |
| 5 | **Randomized disturbance with cycle-persistent direction** | `dist_direction = 0`; direction drawn once per ON cycle, held constant; HUD displays angle |
| 6 | **Perturbed z0 for QP solver** † | Prevents active-set from starting on a bound face, eliminating spurious infeasibility when `ax_nom` is at the motor limit |
| 7 | **OUTER WARN zone label** | 7.5–10 m range was incorrectly showing COMM LOST in the status banner |
| 8 | **Layer 3 hysteresis (`layer3_reentry_m = 1.5 m`)** | Timer no longer resets from ±0.01 m boundary oscillation; prevents Layer 2/3 handoff deadlock |
| 9 | **Inner-outer CBF alpha coordination (quadratic fade)** | `α_eff = α_base × outer_margin_frac²`; inner correction energy proportional to square of remaining outer runway |
| 10 | **`comm_min_red` increased 0.8 m → 3.0 m** | Physical clearance at inner boundary: 3.0 − 2×0.918 = 1.16 m (was 0.16 m) |
| 11 | **`cbf_alpha` reduced 0.80 → 0.35** | Prevents inner CBF ejection from building enough outward velocity to overshoot the outer boundary |
| 12 | **`r_soft_cap` tightened 1.5 → 1.0 rad/s** | Reduces Coriolis sway input `m·u·r/m_sway` during recovery manoeuvres |

---

## Pending Items

| Item | Notes |
|---|---|
| `P.recovery_mode` | HOLD is permanent. CONTINUE_REDUCED and DEADRECKON pilot-selectable modes not yet coded |
| Dead reckoning | B extrapolating A's future position using last known velocity — deferred pending design discussion of divergence risk and mission profile |
| HUD h3 display | Cone barrier value and CONE RELAXED status indicator absent from system column |
| 4 m backport | Items 1–7 above are scale-independent and should be applied to the 4 m configuration file |

---

## Validation Status

| Test | Status |
|---|---|
| Outward disturbance (fixed radial) | ✅ Validated |
| Inward disturbance (fixed radial) | ✅ Validated |
| Randomized disturbance (all angles) | ✅ Validated |
| Layer 2 ldr_scale reliability | ✅ Validated |
| Layer 3 HOLD + timeout + hysteresis | ✅ Validated |
| Tight leader turns without comm exit | ✅ Validated |
| Recovery from tangential boundary position | ✅ Validated |
| Recovery modes (CONTINUE_REDUCED, DEADRECKON) | ⏳ Pending |

---

## File Structure

```
blimpFormation.m
├── PARAMETERS          Physical model, zones, gains, CBF, disturbance, Layer 3
├── ELLIPSOID MESH      3D blimp geometry for visualisation
├── COLOURS             Display palette
├── FIGURE + AXES       Layout: top-view, 3D perspective, right HUD panel
├── GRAPHIC OBJECTS     Blimp markers, zone rings, cone arc, velocity quivers
├── TIMER SETUP         25 ms tick; keyboard handler
├── TICK FUNCTION       Mode dispatch, Layer 2/3 logic, rk4step integration
├── autoLeader          Figure-8 path generator (Mode 1)
├── autoFollower        CLF target, heading PD, CBF filter call, motor allocation
├── clf_cbf_filter      QP safety filter: inner/outer CBF + CLF soft constraint
├── rk4step             4th-order Runge-Kutta 6-DOF integrator
├── applyDisturbance    Fixed/randomized disturbance injection
├── updateDisplay       HUD, zone rings, 3D perspective refresh
└── helpers             fmState, iif, clamp
```

---

*Developed iteratively against the WVU Vicon lab hardware specification. Parameter values from Sponaugle 2025 MSME Thesis unless noted.*
