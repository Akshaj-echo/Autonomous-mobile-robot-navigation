import random

from simulation import Simulation


random.seed(42)

simulation = Simulation()

simulation.generate_new_warehouse("Medium")
simulation.calculate_path("Dijkstra")

assert simulation.path is not None, (
    "Dijkstra failed to find an initial path."
)

print("start:", simulation.robot.position)
print("goal:", simulation.warehouse.goal_position)
print("initial path:", len(simulation.path))
print("algorithm:", simulation.algorithm)

success = simulation.run_navigation(
    dt=0.1,
    max_steps=10000
)

print("success:", success)
print("final:", simulation.robot.position)
print("replans:", simulation.replan_count)
print("collisions:", simulation.collision_count)
print("nodes explored:", simulation.cells_explored)
print("planning time:", simulation.planning_time)

assert success, (
    "Dijkstra navigation failed to reach the goal."
)

assert simulation.collision_count == 0, (
    "Dijkstra navigation caused a collision."
)

assert simulation.algorithm == "Dijkstra", (
    "Simulation did not use Dijkstra."
)

print("DIJKSTRA NAVIGATION TEST: PASS")