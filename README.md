# Bayesian-Optimization-for-Resilient-Traffic-Signal-Timing
This project explores the use of Bayesian Optimization (BO) to find a single, static traffic signal timing plan that is inherently resilient to a variety of network conditions, including heavy congestion and network disruptions. The project is built using Python with **`scikit-optimize`** for the optimization algorithm and **`SUMO (Simulation of Urban MObility)`** for the traffic simulation.

## File Descriptions

This repository contains all the necessary code and configuration files to reproduce the experiment.

- **`grid_.net.xml`**: Defines the 3x3 road network, including junctions and lanes.
- **`grid_normal.rou.xml` & `grid_highstress.rou.xml`**: Defines the 'Normal' and 'High-Stress' traffic demand scenarios.
- **`disruption.add.xml`**: Defines the "City Center Shutdown" event that closes the central intersection mid-simulation.
- **`grid_normal.sumocfg`**, **`grid_highstress.sumocfg`**, **`grid_disrupted.sumocfg`**: The configuration files that combine the network, route, and disruption files to create the simulation.
- **`bayesian_optimization.py`**: The main optimization script that runs the Bayesian Optimization process across all three scenarios to find and save the **'Optimized-for-Resilience'** traffic plan.
- **`evaluation.py`**: The final evaluation script that systematically runs 9 simulations (3 plans x 3 scenarios) to gather performance data for the final comparison and generates the final results.

## Prerequisites

- **SUMO:** You must have SUMO installed on your system. Please follow the official installation guide at [sumo.dlr.de](https://sumo.dlr.de/docs/Installing.html).
