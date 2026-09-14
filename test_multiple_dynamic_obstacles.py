from simulation import Simulation
from warehouse import DynamicObstacle


simulation = Simulation()

simulation.generate_new_warehouse("Medium")

simulation.calculate_path("A*")

assert simulation.path is not None

original_path = simulation.path.copy()

print("original path length:", len(original_path))


# Select several different cells from the existing path.
path_positions = [
    original_path[len(original_path) // 3],
    original_path[(2 * len(original_path)) // 3]
]


dynamic_obstacles = []

for position in path_positions:

    dynamic_obstacle = DynamicObstacle(
        position[0],
        position[1],
        velocity_x=0.0,
        velocity_y=0.0,
        collision_radius=0.5
    )

    simulation.warehouse.add_dynamic_obstacle(
        dynamic_obstacle
    )

    dynamic_obstacles.append(dynamic_obstacle)


# The newly appearing obstacles should invalidate the path.
assert simulation.is_path_valid() is False

simulation.path_invalidated = True

replanned = simulation.replan_if_needed()

print("replanned:", replanned)
print("replan count:", simulation.replan_count)

assert replanned is True
assert simulation.path is not None
assert simulation.path != original_path


# Remove the first obstacle.
simulation.warehouse.dynamic_obstacles.remove(
    dynamic_obstacles[0]
)


simulation.calculate_path("A*")

assert simulation.path is not None

print("path after first obstacle disappears:", len(simulation.path))


# Remove the second obstacle.
simulation.warehouse.dynamic_obstacles.remove(
    dynamic_obstacles[1]
)


simulation.calculate_path("A*")

assert simulation.path is not None

print("path after second obstacle disappears:", len(simulation.path))


print("MULTIPLE DYNAMIC OBSTACLES TEST: PASS")