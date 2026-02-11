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
- Digital twin with parameter mismatch
- Residual-based fault detection
- Safety state machine (NORMAL / DEGRADED / SHUTDOWN)
- Automated verification and validation (V&V)
- Software Failure Modes and Effects Analysis (FMEA)
