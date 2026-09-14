from simulation import Simulation
from warehouse import DynamicObstacle


simulation = Simulation()

simulation.generate_new_warehouse("Medium")

simulation.calculate_path("A*")

assert simulation.path is not None

original_path = simulation.path.copy()

print("original path length:", len(original_path))


# Place an obstacle directly on the current path.
appearance_position = original_path[len(original_path) // 2]

dynamic_obstacle = DynamicObstacle(
    appearance_position[0],
    appearance_position[1],
    velocity_x=0.0,
    velocity_y=0.0,
    collision_radius=0.5
)

simulation.warehouse.add_dynamic_obstacle(
    dynamic_obstacle
)


# The obstacle should invalidate the current path.
assert simulation.is_path_valid() is False

simulation.path_invalidated = True

replanned = simulation.replan_if_needed()

assert replanned is True
assert simulation.path is not None

blocked_path = simulation.path.copy()

print("replanned path length:", len(blocked_path))


# Remove the obstacle from the environment.
simulation.warehouse.dynamic_obstacles.remove(
    dynamic_obstacle
)


# The environment is now clear again.
assert simulation.is_path_valid() is True


# The robot should be able to calculate a path through
# the newly available space.
simulation.calculate_path("A*")

assert simulation.path is not None

print("path after obstacle disappears:", len(simulation.path))

print("OBSTACLE DISAPPEARANCE TEST: PASS")