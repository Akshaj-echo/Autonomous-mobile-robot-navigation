from simulation import Simulation


simulation = Simulation()

simulation.generate_new_warehouse("Medium")
simulation.calculate_path("A*")

print("start:", simulation.robot.position)
print("goal:", simulation.warehouse.goal_position)
print("initial path:", len(simulation.path) if simulation.path else None)

initial_path = simulation.path.copy()

dt = 0.1
injected = False
success = False

for step in range(10000):

    if (
        not injected
        and simulation.path is not None
        and len(simulation.path) > 10
        and step == 50
    ):
        obstacle_index = min(10, len(simulation.path) - 1)

        simulation.warehouse.dynamic_obstacles[0].position = (
            simulation.path[obstacle_index][0],
            simulation.path[obstacle_index][1]
        )

        injected = True

        print("dynamic obstacle injected at:", step)

    if simulation.controller.is_goal_reached(
        simulation.robot,
        simulation.warehouse.goal_position
    ):
        success = True
        break

    simulation.update_dynamic_obstacles(dt)

    if simulation.path_invalidated:
            
        old_path = simulation.path.copy()

        replanned = simulation.replan_if_needed()

        if replanned:
            print("replanned at step:", step)
            print("old path length:", len(old_path))
            print("new path length:", len(simulation.path))
            print("path changed:", simulation.path != old_path)


            if step % 100 == 0:
                print(
                    "DEBUG:",
                    "step=", step,
                    "position=", simulation.robot.position,
                    "target=", simulation.path[0] if simulation.path else None,
                    "velocity=", simulation.robot.velocity,
                    "heading=", simulation.robot.heading,
                    "path_invalidated=", simulation.path_invalidated
                )


    simulation.execute_path_step(dt)

print("success:", success)
print("final:", simulation.robot.position)
print("replans:", simulation.replan_count)
print("collisions:", simulation.collision_count)
print("steps:", step)
print("path remaining:", len(simulation.path) if simulation.path else 0)
assert success, "Robot failed to reach the goal."

assert simulation.replan_count >= 1, (
    "Expected at least one replan."
)

assert simulation.collision_count == 0, (
    "Robot collided with a dynamic obstacle."
)
print("AUTOMATED TEST: PASS")