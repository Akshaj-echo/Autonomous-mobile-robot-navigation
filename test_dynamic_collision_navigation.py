import random

from simulation import Simulation
from warehouse import DynamicObstacle

random.seed(42)
simulation = Simulation()

simulation.generate_new_warehouse("Medium")
simulation.calculate_path("A*")

assert simulation.path is not None

initial_path = simulation.path.copy()

# Use a point on the existing path, away from the start and goal.
path_index = len(initial_path) // 2
target = initial_path[path_index]

# Place the obstacle above that point and move it downward.
obstacle = DynamicObstacle(
    target[0],
    target[1] + 2,
    velocity_x=0.0,
    velocity_y=-1.0,
    collision_radius=0.5
)

simulation.warehouse.add_dynamic_obstacle(obstacle)

# The obstacle will naturally move toward the planned route.
success = simulation.run_navigation(
    dt=0.1,
    max_steps=10000
)

print("initial path length:", len(initial_path))
print("success:", success)
print("replans:", simulation.replan_count)
print("collisions:", simulation.collision_count)
print("simulation time:", simulation.simulation_time)
print("final:", simulation.robot.position)

assert success is True
assert simulation.replan_count >= 1
assert simulation.collision_count == 0

print("DYNAMIC COLLISION NAVIGATION TEST: PASS")