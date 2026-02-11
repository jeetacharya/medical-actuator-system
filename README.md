# Digital Twin–Based Medical Actuator System

## Overview
This project implements a software-only simulation of a medical device actuator subsystem with
closed-loop control, digital twin monitoring, fault detection, safety interlocks, and automated
verification and risk analysis.

The architecture mirrors how regulated medical robotic subsystems are designed, monitored,
and validated in industry environments.

---

## System Architecture

The system consists of the following layers:

- Physics-based actuator model
- Closed-loop PID motion controller
- Digital twin model with parameter mismatch
- Residual-based fault detection
- Safety state machine (Normal/ Degraded/ Shutdown)
- Verification and validation (V&V)
- Software Failure Modes and Effects Analysis (FMEA)

---

## Key Features

### Closed-Loop Control
- Physics-based rotational actuator model
- PID based position control
- Performance metrics: overshoot, settling time, RMS error

### Digital Twin
- Parallel model with parameter mismatch
- Used as predictive reference for diagnostics

### Fault Detection & Safety
- Residual-based fault detection for position and velocity
- Sensor drift fault injection
- Safety state transitions:
  - NORMAL: full performance
  - DEGRADED: current limiting
  - SHUTDOWN: motion disabled

### Automated Verification
- Requirements-based verification
- Pass/Fail evaluation of performance and safety metrics
- Fault detection latency measurement

### Risk Analysis
- Software FMEA generation
- Risk Priority Number (RPN) calculation
- Ranked failure modes

---

## Project Structure

*   `medical_actuator-system/`
    *   `models/`: Actuator and digital twin models
    *   `control/`: PID controller
    *   `metrics/`: Performance metrics of response
    *   `diagnostics/`: Fault injection and detection
    *   `safety/`: Safety state machine
    *   `simulation/`: System integration
    *   `verification/`: Automated V&V
    *   `risk/`: Software FMEA
    *   `results/`: Plots and reports
    *   `phase*_main.py`: Execution scripts

---

## Verification Results (Summary)

| Requirement              | Status |
|--------------------------|--------|
| Overshoot < 10%          | PASS   |
| Settling Time < 1.0 s    | PASS   |
| RMS Error < 17.15 deg    | PASS   |
| Fault Detection < 200 ms | PASS   |

---
