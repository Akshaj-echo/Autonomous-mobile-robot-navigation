from simulation import Simulation
from pathfinding import AStarPlanner


def zero_heuristic(position_a, position_b):
    return 0


simulation = Simulation()

simulation.generate_new_warehouse("Medium")

simulation.planner = AStarPlanner(
    heuristic=zero_heuristic
)

path, explored = simulation.planner.plan(
    simulation.get_robot_planning_position(),
    simulation.warehouse.goal_position,
    simulation.warehouse.obstacles,
    simulation.warehouse
)

print("path found:", path is not None)
print("nodes explored:", explored)

assert path is not None, "A* failed with configurable heuristic."

assert len(path) > 0, "Returned path is empty."

print("CONFIGURABLE HEURISTIC TEST: PASS")