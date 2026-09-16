from simulation import Simulation


simulation = Simulation()

simulation.generate_new_warehouse("Medium")




simulation.calculate_path("A*")

print("start:", simulation.robot.position)
print("goal:", simulation.warehouse.goal_position)
print("initial path:", len(simulation.path) if simulation.path else None)

success = simulation.run_navigation(
    dt=0.1,
    max_steps=10000
)

print("success:", success)
print("final:", simulation.robot.position)
print("replans:", simulation.replan_count)
print("collisions:", simulation.collision_count)
print("path remaining:", len(simulation.path) if simulation.path else 0)

assert success, "Robot failed to reach the goal."

assert simulation.collision_count == 0, (
    "Robot collided with a dynamic obstacle."
)

print("FULL NAVIGATION TEST: PASS")