from simulation import Simulation
from warehouse import DynamicObstacle


simulation = Simulation()

simulation.generate_new_warehouse("Medium")

simulation.calculate_path("A*")

assert simulation.path is not None

original_path = simulation.path.copy()

print("original path length:", len(original_path))


# Find a waypoint on the existing path where an obstacle can
# appear and an alternate route still exists.
appearance_position = None

for candidate in original_path[1:-1]:

    dynamic_obstacle = DynamicObstacle(
        candidate[0],
        candidate[1],
        velocity_x=0.0,
        velocity_y=0.0,
        collision_radius=0.5
    )

    simulation.warehouse.add_dynamic_obstacle(
        dynamic_obstacle
    )

    simulation.update_dynamic_obstacles(0)

    if simulation.path_invalidated:

        old_path = simulation.path.copy()

        replanned = simulation.replan_if_needed()

        if (
            replanned
            and
            simulation.path is not None
            and
            simulation.path != old_path
            and
            simulation.is_path_valid()
        ):
            appearance_position = candidate
            break

    simulation.warehouse.dynamic_obstacles.clear()
    simulation.path = original_path.copy()
    simulation.path_invalidated = False
    simulation.replan_count = 0


assert appearance_position is not None, (
    "Could not find a path waypoint with a valid alternate route."
)


print("appearance position:", appearance_position)
print("path invalidated:", True)
print("replanned:", True)
print("replan count:", simulation.replan_count)
print(
    "new path length:",
    len(simulation.path) if simulation.path else None
)
print("new path valid:", simulation.is_path_valid())


assert simulation.replan_count == 1
assert simulation.path is not None
assert simulation.path != original_path
assert simulation.is_path_valid() is True


print("OBSTACLE APPEARANCE TEST: PASS")