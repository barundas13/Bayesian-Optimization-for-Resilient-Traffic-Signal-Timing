import os
import sys
import traci
from sumolib import checkBinary
import numpy as np
import xml.etree.ElementTree as ET
import pandas as pd
import matplotlib.pyplot as plt

# --- Configuration ---
GUI = False
MAX_SIM_STEPS = 3600
SUMO_BINARY = checkBinary('sumo-gui') if GUI else checkBinary('sumo')

SCENARIOS = {
    "Normal": "grid_normal.sumocfg",
    "High-Stress": "grid_highstress.sumocfg",
    "Disrupted": "grid_disrupted.sumocfg"
}

PLANS = {
    "Default-SUMO": None,  # Special case: no additional file
    "Optimized-for-Normal": "plan_normal_day.add.xml",
    "Optimized-for-Resilience": "plan_resilient.add.xml"
}


def run_single_evaluation(scenario_name, scenario_cfg, plan_name, plan_file):
    # Runs a single SUMO simulation for one plan and one scenario

    sumo_cmd = [SUMO_BINARY, "-c", scenario_cfg, "--tripinfo-output", "temp_tripinfo.xml", "--no-warnings", "true"]

    # Add the plan file only if it's not the default SUMO plan
    if plan_file:
        sumo_cmd.extend(["-a", plan_file])

    traci.start(sumo_cmd)
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0 and step < MAX_SIM_STEPS:
        traci.simulationStep()
        step += 1
    traci.close()

    # Parse the results
    try:
        tree = ET.parse("temp_tripinfo.xml")
        root = tree.getroot()
        wait_times = [float(trip.get('waitingTime')) for trip in root.findall('tripinfo')]
        return np.mean(wait_times) if wait_times else MAX_SIM_STEPS
    except (FileNotFoundError, ET.ParseError):
        return MAX_SIM_STEPS  # High penalty for failure


if __name__ == "__main__":
    # Ensure prerequisites are met
    if 'SUMO_HOME' not in os.environ:
        sys.exit("please declare environment variable 'SUMO_HOME'")

    results = []
    print("Starting Final Evaluation")

    for plan_name, plan_file in PLANS.items():
        for scenario_name, scenario_cfg in SCENARIOS.items():
            avg_wait_time = run_single_evaluation(scenario_name, scenario_cfg, plan_name, plan_file)
            results.append({
                "Plan": plan_name,
                "Scenario": scenario_name,
                "Avg. Wait Time (s)": avg_wait_time
            })

    # --- Process and Save the Results ---
    results_df = pd.DataFrame(results)
    print("\nEvaluation Complete")
    print(results_df)

    # Pivot the table for easier plotting and save to CSV
    pivot_df = results_df.pivot(index='Scenario', columns='Plan', values='Avg. Wait Time (s)')
    pivot_df.to_csv('final_comparison_results.csv')

    # --- Plotting ---
    pivot_df.plot(kind='bar', figsize=(12, 8), rot=0)
    plt.title('Performance Comparison of Signal Plans Across Scenarios', fontsize=16)
    plt.ylabel('Average Vehicle Wait Time (s)')
    plt.xlabel('Scenario')
    plt.tight_layout()
    plt.savefig('final_performance_comparison.png')
    plt.show()