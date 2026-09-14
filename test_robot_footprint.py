from simulation import Simulation


simulation = Simulation()

simulation.generate_new_warehouse("Medium")

simulation.robot.collision_radius = 0.5


# Pick a static obstacle.
obstacle = next(iter(simulation.warehouse.obstacles))

obstacle_position = (
    obstacle[0],
    obstacle[1]
)


# Remove dynamic obstacles so this test isolates
# static robot-footprint collision.
simulation.warehouse.dynamic_obstacles.clear()


# Robot center exactly on the obstacle.
assert simulation.is_robot_position_safe(
    obstacle_position
) is False


# Robot center inside the robot's collision radius.
near_obstacle_position = (
    obstacle_position[0] + 0.4,
    obstacle_position[1]
)

assert simulation.is_robot_position_safe(
    near_obstacle_position
) is False


# Find a genuinely safe position.
safe_position = None

for candidate in simulation.warehouse.walkable_cells:

    candidate = (
        float(candidate[0]),
        float(candidate[1])
    )

    if simulation.is_robot_position_safe(candidate):

        safe_position = candidate
        break


assert safe_position is not None

assert simulation.is_robot_position_safe(
    safe_position
) is True


print("obstacle:", obstacle_position)
print("safe position:", safe_position)
print("ROBOT FOOTPRINT TEST: PASS")