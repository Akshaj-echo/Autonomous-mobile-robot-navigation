import random

from simulation import Simulation


def benchmark_environment(seed):

    random.seed(seed)

    base_simulation = Simulation()
    base_simulation.generate_new_warehouse("Medium")

    warehouse = base_simulation.warehouse
    robot_position = base_simulation.robot.position

    # Remove dynamic obstacles for a pure planner benchmark.
    warehouse.dynamic_obstacles = []

    results = {}

    for algorithm in ["Dijkstra", "A*"]:

        simulation = Simulation()

        simulation.warehouse = warehouse

        simulation.robot = type(base_simulation.robot)(
            robot_position[0],
            robot_position[1],
            max_velocity=base_simulation.robot.max_velocity,
            max_acceleration=base_simulation.robot.max_acceleration,
            collision_radius=base_simulation.robot.collision_radius,
            max_angular_velocity=base_simulation.robot.max_angular_velocity,
            max_angular_acceleration=base_simulation.robot.max_angular_acceleration
        )

        simulation.calculate_path(algorithm)

        results[algorithm] = {
            "planning_time": simulation.planning_time,
            "nodes_explored": simulation.cells_explored,
            "path_length": (
                len(simulation.path)
                if simulation.path is not None
                else 0
            ),
            "path_cost": simulation.path_cost,
            "success": simulation.path is not None
        }

    return results


SEEDS = list(range(1, 21))

all_results = []


for seed in SEEDS:

    results = benchmark_environment(seed)

    dijkstra = results["Dijkstra"]
    a_star = results["A*"]

    assert dijkstra["success"], (
        f"Dijkstra failed on seed {seed}."
    )

    assert a_star["success"], (
        f"A* failed on seed {seed}."
    )

    assert dijkstra["path_cost"] == a_star["path_cost"], (
        f"Path costs differ on seed {seed}: "
        f"Dijkstra={dijkstra['path_cost']}, "
        f"A*={a_star['path_cost']}"
    )

    all_results.append(results)

    print(
        f"Seed {seed}: "
        f"Dijkstra={dijkstra['planning_time']:.6f}s, "
        f"A*={a_star['planning_time']:.6f}s, "
        f"Dijkstra nodes={dijkstra['nodes_explored']}, "
        f"A* nodes={a_star['nodes_explored']}, "
        f"path cost={dijkstra['path_cost']}"
    )


average_dijkstra_time = sum(
    result["Dijkstra"]["planning_time"]
    for result in all_results
) / len(all_results)

average_a_star_time = sum(
    result["A*"]["planning_time"]
    for result in all_results
) / len(all_results)

average_dijkstra_nodes = sum(
    result["Dijkstra"]["nodes_explored"]
    for result in all_results
) / len(all_results)

average_a_star_nodes = sum(
    result["A*"]["nodes_explored"]
    for result in all_results
) / len(all_results)

average_dijkstra_path_length = sum(
    result["Dijkstra"]["path_length"]
    for result in all_results
) / len(all_results)

average_a_star_path_length = sum(
    result["A*"]["path_length"]
    for result in all_results
) / len(all_results)

average_path_cost = sum(
    result["Dijkstra"]["path_cost"]
    for result in all_results
) / len(all_results)

speedup = (
    average_dijkstra_time
    / average_a_star_time
)

node_reduction = (
    (
        average_dijkstra_nodes
        - average_a_star_nodes
    )
    / average_dijkstra_nodes
) * 100


print("\nBENCHMARK SUMMARY")

print(
    f"Success rate: "
    f"{len(all_results)}/{len(SEEDS)}"
)

print(
    f"Average Dijkstra time: "
    f"{average_dijkstra_time:.6f}s"
)

print(
    f"Average A* time: "
    f"{average_a_star_time:.6f}s"
)

print(
    f"Average Dijkstra nodes: "
    f"{average_dijkstra_nodes:.2f}"
)

print(
    f"Average A* nodes: "
    f"{average_a_star_nodes:.2f}"
)

print(
    f"Average Dijkstra path length: "
    f"{average_dijkstra_path_length:.2f}"
)

print(
    f"Average A* path length: "
    f"{average_a_star_path_length:.2f}"
)

print(
    f"Average path cost: "
    f"{average_path_cost:.2f}"
)

print(
    f"A* speedup: "
    f"{speedup:.2f}x"
)

print(
    f"A* node reduction: "
    f"{node_reduction:.2f}%"
)

print("\nBENCHMARK: PASS")