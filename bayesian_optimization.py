import os
import sys
import traci
from sumolib import checkBinary
import numpy as np
import xml.etree.ElementTree as ET
from skopt import gp_minimize
from skopt.space import Real, Integer
from skopt.utils import use_named_args
import skopt.plots as skplt
import matplotlib.pyplot as plt

# --- Configuration ---
GUI = False  # For faster simulation
MAX_SIM_STEPS = 3600  # Each simulation runs for 1 hour

# List of scenarios to evaluate for each set of parameters
SCENARIOS = ["grid_normal.sumocfg", "grid_highstress.sumocfg", "grid_disrupted.sumocfg"]

# --- SUMO Setup ---
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

sumo_binary = checkBinary('sumo-gui') if GUI else checkBinary('sumo')


def generate_tl_program(cycle_length, ns_green_ratio, file_path="temp_tls.add.xml"):
    # Calculate phase durations
    green_time = cycle_length - 6  # 3s yellow for each of the 2 main directions
    ns_green_time = int(green_time * ns_green_ratio)
    ew_green_time = green_time - ns_green_time

    # Create the XML structure
    root = ET.Element("additional")

    # Assuming all 9 intersections in a 3x3 grid have the same logic
    for i in range(3):
        for j in range(3):
            tl_id = f"J_{i}_{j}"
            tl_logic = ET.SubElement(root, "tlLogic", id=tl_id, type="static", programID="bo_plan", offset="0")
            # North-South Green Phase
            ET.SubElement(tl_logic, "phase", duration=str(ns_green_time), state="GGggrrrrGGggrrrr")
            # North-South Yellow Phase
            ET.SubElement(tl_logic, "phase", duration="3", state="yyyyrrrryyyyrrrr")
            # East-West Green Phase
            ET.SubElement(tl_logic, "phase", duration=str(ew_green_time), state="rrrrGGggrrrrGGgg")
            # East-West Yellow Phase
            ET.SubElement(tl_logic, "phase", duration="3", state="rrrryyyyrrrryyyy")

    # Write to file
    tree = ET.ElementTree(root)
    tree.write(file_path)


def objective_function(params):
    # The main objective function to be minimized by Bayesian Optimization
    cycle_length, ns_green_ratio = params

    # Generate the traffic light program file for this set of parameters
    tl_program_file = "temp_tls.add.xml"
    generate_tl_program(cycle_length, ns_green_ratio, tl_program_file)

    scenario_scores = []

    # Evaluate this traffic plan against each scenario
    for scenario_cfg in SCENARIOS:
        sumo_cmd = [
            sumo_binary,
            "-c", scenario_cfg,
            "-a", tl_program_file,  # Use the generated traffic light plan
            "--tripinfo-output", "temp_tripinfo.xml",  # Temporary output file
            "--no-warnings", "true"
        ]

        traci.start(sumo_cmd)

        step = 0
        while step < MAX_SIM_STEPS:
            traci.simulationStep()
            # Break early if simulation is empty (for lighter scenarios)
            if traci.simulation.getMinExpectedNumber() == 0 and step > 100:
                break
            step += 1

        traci.close()

        # Parse results and calculate score for this scenario
        try:
            tree = ET.parse("temp_tripinfo.xml")
            root = tree.getroot()
            wait_time = [float(trip.get('waitingTime')) for trip in root.findall('tripinfo')]
            # If no cars finished, it's a failure -> very high penalty
            if not wait_time:
                scenario_score = MAX_SIM_STEPS * 10  # High penalty
            else:
                scenario_score = np.mean(wait_time)
        except (FileNotFoundError, ET.ParseError):
            scenario_score = MAX_SIM_STEPS * 10  # High penalty for simulation crash

        scenario_scores.append(scenario_score)

    # Calculate the final resilience score
    resilience_score = max(scenario_scores)

    return resilience_score


if __name__ == "__main__":
    # Define the parameter search space
    search_space = [
        Integer(low=20, high=120, name='cycle_length'),
        Real(low=0.3, high=0.7, name='ns_green_ratio')
    ]


    # Create the wrapper function for skopt
    @use_named_args(dimensions=search_space)
    def objective_wrapper(cycle_length, ns_green_ratio):
        # This function just formats the parameters and calls the main objective function
        return objective_function([cycle_length, ns_green_ratio])

    # Run the Bayesian Optimization
    print("Starting Bayesian Optimization")
    result = gp_minimize(
        func=objective_wrapper,
        dimensions=search_space,
        n_calls=30,  # Total number of iterations (parameter sets to test)
        n_random_starts=10,  # Number of initial random guesses before optimizing
        random_state=123,
        verbose=True
    )

    # Print and save the final results
    print("\nOptimization Finished")
    print("Best parameters found:")
    print(f"Optimal Cycle Length: {result.x[0]} s")
    print(f"Optimal N-S Green Ratio: {result.x[1]:.3f}")
    print(f"Best resilience score: {result.fun:.2f} s")

    # Visualize the optimization process
    skplt.plot_convergence(result)
    plt.savefig('bo_convergence.png')

    # This shows how the score varies with each parameter
    skplt.plot_objective(result, dimensions=['cycle_length', 'ns_green_ratio'])
    plt.savefig('bo_objective_landscape.png')

    plt.show()

    # --- Save the best found parameters to a dedicated file ---
    best_cycle_length = result.x[0]
    best_ns_green_ratio = result.x[1]

    generate_tl_program(best_cycle_length, best_ns_green_ratio, file_path="plan_resilient.add.xml")
    print("Resilient plan saved.")