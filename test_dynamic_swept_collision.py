from simulation import Simulation
from warehouse import DynamicObstacle


simulation = Simulation()

simulation.generate_new_warehouse("Medium")

start = simulation.robot.position

# Place a dynamic obstacle directly across the robot's movement path.
obstacle_position = (
    start[0] + 2,
    start[1]
)

dynamic_obstacle = DynamicObstacle(
    obstacle_position[0],
    obstacle_position[1],
    velocity_x=0.0,
    velocity_y=0.0,
    collision_radius=0.5
)

simulation.warehouse.add_dynamic_obstacle(
    dynamic_obstacle
)

end = (
    start[0] + 4,
    start[1]
)

safe = simulation.is_robot_path_safe_from_dynamic_obstacles(
    start,
    end
)

print("start:", start)
print("obstacle:", obstacle_position)
print("end:", end)
print("path safe:", safe)

assert safe is False

print("DYNAMIC SWEPT COLLISION TEST: PASS")