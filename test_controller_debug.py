from simulation import Simulation


simulation = Simulation()

simulation.generate_new_warehouse("Medium")
simulation.calculate_path("Dijkstra")

print("start:", simulation.robot.position)
print("goal:", simulation.warehouse.goal_position)
print("first 10 path points:", simulation.path[:10])

for step in range(300):

    if simulation.path is None:
        print("PATH IS NONE")
        break

    if len(simulation.path) == 0:
        print("PATH EMPTY")
        break

    target = simulation.path[0]

    old_position = simulation.robot.position
    old_heading = simulation.robot.heading
    old_velocity = simulation.robot.velocity

    simulation.execute_path_step(0.1)

    if step % 10 == 0:

        distance_x = target[0] - simulation.robot.position[0]
        distance_y = target[1] - simulation.robot.position[1]

        distance = (
            distance_x ** 2 +
            distance_y ** 2
        ) ** 0.5

        print(
            "step:", step,
            "position:", tuple(
                round(value, 3)
                for value in simulation.robot.position
            ),
            "target:", target,
            "distance:", round(distance, 3),
            "heading:", round(simulation.robot.heading, 2),
            "velocity:", round(simulation.robot.velocity, 3),
            "old_heading:", round(old_heading, 2),
            "old_velocity:", round(old_velocity, 3),
            "path_remaining:", len(simulation.path)
        )