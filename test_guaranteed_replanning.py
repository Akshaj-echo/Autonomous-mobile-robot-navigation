import random

from simulation import Simulation


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

obstacle_index = len(simulation.path) // 2

blocked_position = simulation.path[obstacle_index]
print("full path:", simulation.path)

simulation.warehouse.dynamic_obstacles[0].position = (
    blocked_position[0],
    blocked_position[1]
)

simulation.update_dynamic_obstacles(0)

print("blocked position:", blocked_position)
print("path invalidated:", simulation.path_invalidated)

assert simulation.path_invalidated, (
    "Dynamic obstacle failed to invalidate the path."
)

old_path = simulation.path.copy()

replanned = simulation.replan_if_needed()

print("replanned:", replanned)
print("replan count:", simulation.replan_count)
print("new path:", len(simulation.path) if simulation.path else None)
print("path changed:", simulation.path != old_path)

assert replanned, (
    "Planner failed to replan."
)

assert simulation.replan_count == 1, (
    "Expected exactly one replan."
)

assert simulation.path is not None, (
    "New path does not exist."
)

assert simulation.is_path_valid(), (
    "New path is still blocked."
)

assert simulation.path != old_path, (
    "Replanned path did not change."
)

print("GUARANTEED REPLANNING TEST: PASS")