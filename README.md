# Self-Commissioning Drive for PMSM using CCS-MPC with Prognostics using AI

Standard PI-based FOC relies on linear controllers that struggle to maintain optimal dynamic response, decoupling, and strict constraint handling across the highly non-linear, multi-variable operation of interior permanent magnet synchronous motors (IPMSMs).

This repository contains the complete simulation models, automated self-commissioning algorithms, continuous-control-set model predictive control (CCS-MPC), space vector pulse-width modulation (SVPWM), and Model-in-the-Loop (MIL) / Software-in-the-Loop (SIL) verification suite.

---

## Project Structure

- **[`Review_2/`](./Review_2/)**: Complete Review 2 standalone models, scripts, LaTeX presentation deck, and verified MIL/SIL co-simulation results.
  - **`Review_2/Models/`**: Standalone AutoTuner, Speed MPC, and CCS-MPC controller `.slx` models.
  - **`Review_2/Scripts/`**: BMW i3 motor parameter definitions, MTPA/MTPV trajectory generators, and test harnesses.
  - **`Review_2/Presentation/`**: 90-slide Beamer presentation deck (`first_review.tex`).
  - **`Review_2/Images/`**: High-resolution MIL vs. SIL verification plots and control envelopes.
- **[`Model/`](./Model/)**: Baseline and Review 1 comparative simulation models (FOC vs. CCS-MPC).

---

## Target Motor Parameters (BMW i3 IPMSM)

| Parameter | Symbol | Value | Unit |
|---|:---:|:---:|:---:|
| Pole Pairs | $p$ | **6** | — |
| Stator Resistance | $R_s$ | **5.30** | $\text{m}\Omega$ |
| $d$-axis Inductance | $L_d$ | **71.20** | $\mu\text{H}$ |
| $q$-axis Inductance | $L_q$ | **141.30** | $\mu\text{H}$ |
| PM Flux Linkage | $\Psi_{PM}$ | **0.0436** | $\text{Wb}$ |
| DC Bus Voltage | $V_{dc}$ | **400.0** | $\text{V}$ |
| Peak Current Limit | $I_{max}$ | **565.685** | $\text{A}$ ($400\text{ A}_{rms}$) |
| Rated Base Speed | $n_{base}$ | **4000** | $\text{RPM}$ |
| Maximum Speed | $n_{max}$ | **11400** | $\text{RPM}$ |

