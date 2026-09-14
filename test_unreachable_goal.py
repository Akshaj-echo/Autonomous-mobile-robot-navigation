import random

from simulation import Simulation
from warehouse import DynamicObstacle


random.seed(42)

simulation = Simulation()

simulation.generate_new_warehouse("Medium")

simulation.calculate_path("A*")

assert simulation.path is not None, (
    "Initial planner failed to find a path."
)

print("start:", simulation.robot.position)
print("goal:", simulation.warehouse.goal_position)
print("initial path:", len(simulation.path))

goal = simulation.warehouse.goal_position

simulation.warehouse.dynamic_obstacles.clear()

simulation.warehouse.dynamic_obstacles.append(
    DynamicObstacle(
        goal[0],
        goal[1]
    )
)

simulation.path_invalidated = not simulation.is_path_valid()

print("path invalidated:", simulation.path_invalidated)

assert simulation.path_invalidated, (
    "Goal blockage did not invalidate the path."
)

replanned = simulation.replan_if_needed()

print("replanned:", replanned)
print("replan count:", simulation.replan_count)
print("path after replan:", simulation.path)

assert not replanned, (
    "Planner unexpectedly found a path to the blocked goal."
)

print("UNREACHABLE GOAL TEST: PASS")