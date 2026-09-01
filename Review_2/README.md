# Review 2: Cascaded MPC & AutoTuner MIL/SIL Verification for BMW i3 IPMSM

## Overview
This directory contains the complete Model-in-the-Loop (MIL) and Software-in-the-Loop (SIL) verification suite, mathematical models, controller algorithms, presentation materials, and high-resolution result plots for **Project Review 2**.

## Directory Structure
- **Models/**:
  - `PMSM_CascadedMPC.slx`: Complete cascaded simulation model (AutoTuner + Speed MPC + CCS-MPC + Inverter + IPMSM).
  - `Standalone_AutoTuner.slx`: Standalone supervisory parameter identification module.
  - `Standalone_Speed_MPC.slx`: Standalone outer speed MPC module (1 kHz rate).
  - `Standalone_CCSMPC_Controller.slx`: Standalone inner current CCS-MPC & Space Vector PWM module (10 kHz rate, 5 µs carrier).
- **Scripts/**:
  - `params_pmsm_inverter.m`: Parameter definition file (BMW i3 IPMSM, Inverter, Controller gains).
  - `plot_mtpa_mtpv_envelope.m`: Generates 2-panel MTPA / Field-Weakening trajectory & capability envelopes.
  - `run_master_mil_sil_and_codegen.m`: Automated master MIL vs. SIL co-simulation and verification script.
  - `configure_models_sil.m`: Embedded Coder SIL target configuration script.
- **Presentation/**:
  - `first_review.tex`: Complete 90-slide Beamer presentation deck with block diagrams and MIL/SIL comparisons.
- **Images/**:
  - High-resolution verification plots for AutoTuner parameter estimation, Speed MPC tracking, CCS-MPC deadbeat response, PWM gate pulses, and MTPA/MTPV envelopes.

## Summary of Key Verification Metrics
| Subsystem | Metric / Signal | MIL vs. SIL Error | Verification Status |
|---|---|:---:|:---:|
| **AutoTuner** | \, L_d, L_q, \Psi_{PM}, J, B\$ | **0.00e+00** | Bit-Exact C99 Match |
| **Speed MPC** | \{q,ref}, i_{d,ref}\$ Reference Currents | **1.14e-13 A** | Machine Epsilon Match |
| **CCS-MPC** | \{d,ref}, v_{q,ref}, d_a, d_b, d_c, S_{abc}\$ | **0.00e+00** | Bit-Exact Gate Match |
