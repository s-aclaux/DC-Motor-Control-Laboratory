# DC-Motor-Control-Laboratory
Design and implementation of two controllers into a DC motor inertia output setup, and comparison of results with theoretical models. 

## Overview
This project investigates position control of a DC motor with an inertial load using two controllers:
- **Proportional Controller**
- **Phase-Lead Compensator**

The work combines analytical modeling, MATLAB simulation, and hardware implementation on a National Instruments myRIO. The goal is to compare theoretical predictions with experimental results and evaluate controller performance in terms of settling time, overshoot, and current limits.

---

## Objectives
1. Develop a transfer function model for the DC motor and inertial load system.
2. Design a proportional controller and a phase-lead compensator to meet performance criteria.
3. Implement both controllers on the myRIO hardware platform.
4. Compare and analyse experimental results against analytical solutions and MATLAB simulations.

---

## Repository Contents
- **Report: `docs` folder**  
  Full LaTeX-written report detailing the derivation, controller design, simulation results, and experimental analysis.
  
- **Figures: `figs` folder**  
  Diagrams of the physical setup, block diagrams, linear graphs, and MATLAB plots.

- **MATLAB Code: `scripts` folder**  
  Contains all calculations, transfer function definitions, controller design, Bode plots, and step response simulations.

---

## How to Use
1. **Run MATLAB Simulations**  
   - Open the provided `.m` script in MATLAB.
   - Execute the script to reproduce:
     - Transfer function derivations
     - Controller design values
     - Bode plots
     - Step responses for angle and D/A voltage

2. **Hardware Implementation**  
   - Controllers are implemented on NI myRIO using LabVIEW - simply enter the gain, zero and pole magnitudes in the correct fields.
   - Compare hardware results with MATLAB predictions.

---

## Key Results
- **Proportional Controller:**  
  Theoretically meets overshoot and settling time requirements, but experimentally insufficient due to friction and current limits.

- **Phase-Lead Compensator:**  
  Achieves fast settling and acceptable overshoot. Experimental results closely match simulation, with minor differences in D/A voltage oscillations due to noise and unmodeled dynamics.

---

## Conclusion
This project demonstrates the importance of advanced compensator design in motor control. While proportional control is simple, it is limited under real-world constraints. The phase-lead compensator provides robust performance, validating the analytical design process and highlighting the gap between ideal models and physical systems.

---

## Author
**Alexandre Claux**  
University of Washington – Mechanical Engineering 
