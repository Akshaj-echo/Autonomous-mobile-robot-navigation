from simulation import Simulation
from warehouse import DynamicObstacle


simulation = Simulation()

simulation.generate_new_warehouse("Medium")

start = simulation.robot.position

# Robot moves 4 cells to the right.
end = (
    start[0] + 4,
    start[1]
)

# Obstacle starts above the robot's path
# and moves downward across it.
obstacle = DynamicObstacle(
    start[0] + 2,
    start[1] + 2,
    velocity_x=0.0,
    velocity_y=-4.0,
    collision_radius=0.5
)

simulation.warehouse.add_dynamic_obstacle(
    obstacle
)

safe = simulation.is_robot_path_safe_from_dynamic_obstacles(
    start,
    end
)

print("robot start:", start)
print("robot end:", end)
print("obstacle start:", obstacle.position)
print("obstacle velocity:", obstacle.velocity)
print("path safe:", safe)

assert safe is False

print("MOVING SWEPT COLLISION TEST: PASS")