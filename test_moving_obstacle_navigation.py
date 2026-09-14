import random

from simulation import Simulation
random.seed(42)


simulation = Simulation()

simulation.generate_new_warehouse("Medium")
simulation.calculate_path("A*")

print("start:", simulation.robot.position)
print("goal:", simulation.warehouse.goal_position)
print("initial path:", len(simulation.path) if simulation.path else None)

dt = 0.1
success = False

for step in range(10000):

    if simulation.controller.is_goal_reached(
        simulation.robot,
        simulation.warehouse.goal_position
    ):
        success = True
        break

    simulation.update_dynamic_obstacles(dt)

    if simulation.path_invalidated:
        simulation.replan_if_needed()

    simulation.execute_path_step(dt)

print("success:", success)
print("final:", simulation.robot.position)
print("replans:", simulation.replan_count)
print("collisions:", simulation.collision_count)
print("steps:", step)

assert success, "Robot failed to reach the goal."

assert simulation.collision_count == 0, (
    "Robot collided with a dynamic obstacle."
)

print("MOVING OBSTACLE TEST: PASS")