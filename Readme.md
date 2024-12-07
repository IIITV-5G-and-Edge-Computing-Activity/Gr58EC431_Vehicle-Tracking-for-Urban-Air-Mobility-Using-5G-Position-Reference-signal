# Vehicle Tracking for Urban Air Mobility Using 5G PRS

This project demonstrates the use of *5G New Radio (NR) Positioning Reference Signals (PRS)* for tracking autonomous vehicles in Urban Air Mobility (UAM) scenarios. The example leverages MATLAB's 5G Toolbox to simulate a realistic urban environment and evaluate the performance of PRS-based positioning. The project highlights how ToA (Time of Arrival) measurements from multiple gNodeBs improve positioning accuracy and robustness under challenging urban conditions.

---

## Features

- *Urban Mobility Simulation*: Simulates an urban environment with realistic vehicle trajectories.
- *Integration of 5G PRS*: Demonstrates the generation, transmission, and reception of PRS signals.
- *Positioning Accuracy*: Evaluates performance using multi-gNodeB ToA measurements.
- *Advanced Tracking*: Implements tracking algorithms (e.g., Extended Kalman Filter) for trajectory refinement.
- *Comprehensive Visualization*: Includes 2D/3D trajectory plots and positioning error metrics.

---

## Repository Contents

- *MATLAB Code*:
  - generateUrbanScenario.m: Creates the urban environment and vehicle trajectory.
  - configurePRS.m: Configures the 5G PRS signals as per 3GPP standards.
  - simulateChannel.m: Simulates the urban multipath propagation channel.
  - processToA.m: Processes ToA measurements from received PRS signals.
  - applyTracking.m: Implements Kalman filtering for position tracking.
- *Results*:
  - trajectory.png: 3D visualization of the vehicle trajectory with uncertainty regions.
  - errorMetrics.png: Positioning error metrics and cumulative distribution.
- *Documentation*:
  - README.md: This file provides an overview of the project.
  - references.md: List of references and related 3GPP specifications.

---

## Requirements

- *MATLAB R2023a or newer*: The project requires MATLAB with the following toolboxes:
  - 5G Toolbox
  - Signal Processing Toolbox
  - Statistics and Machine Learning Toolbox
- *System Requirements*: Ensure your system meets MATLAB's minimum hardware requirements.

---

## How to Run

1. Clone or download the repository.
2. Open MATLAB and set the project directory as the current folder.
3. Run the main script:
   ```matlab
   mainSimulation.m
4. View Results, Including Trajectory Plots and Error Metrics


## Key Steps

### Urban Scenario Generation:
- Define vehicle trajectory and gNodeB positions in a 3D urban layout.
- Simulate the environment using the generateUrbanScenario.m script.

### PRS Configuration:
- Configure the PRS signals for transmission using 3GPP-compliant settings.
- Use configurePRS.m to set subcarrier spacing, periodicity, and bandwidth.

### Channel Simulation:
- Simulate urban multipath propagation using a clustered delay line (CDL) model.
- Incorporate realistic impairments like NLoS (Non-Line-of-Sight) and Doppler shifts.

### ToA Measurement:
- Process received PRS signals to estimate ToA.
- Use processToA.m to extract and refine ToA measurements.

### Tracking:
- Fuse multi-gNodeB ToA data using Kalman filtering.
- Apply applyTracking.m to generate a smooth and accurate trajectory.

---

## Outputs

- *Trajectory Visualization*: 3D and 2D plots of vehicle path, estimated trajectory, and uncertainty regions.
- *Error Metrics*: Quantitative evaluation of positioning accuracy, including RMSE and cumulative error distributions.

---

## References

1. *3GPP TS 38.211*: "NR; Physical channels and modulation"
2. *3GPP TS 38.855*: "Study on NR positioning support"
3. [*MathWorks Documentation*: Vehicle Tracking for Urban Air Mobility Using 5G PRS](https://in.mathworks.com/help/5g/ug/vehicle-tracking-for-urban-air-mobility-using-5g-prs.html)
