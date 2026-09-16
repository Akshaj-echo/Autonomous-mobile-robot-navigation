from simulation import Simulation


simulation = Simulation()

simulation.generate_new_warehouse("Medium")

robot_position = simulation.robot.position

obstacle_position = None

for cell in simulation.warehouse.walkable_cells:

    if cell != robot_position:
        obstacle_position = cell
        break

assert obstacle_position is not None
simulation.warehouse.obstacles.add(
    obstacle_position
)

start_position = (
    obstacle_position[0] - 2.0,
    obstacle_position[1]
)

end_position = (
    obstacle_position[0] + 2.0,
    obstacle_position[1]
)

simulation.robot.position = start_position

assert simulation.is_robot_position_safe(start_position)

assert simulation.is_robot_position_safe(end_position)

assert not simulation.is_robot_path_safe(
    start_position,
    end_position
)

print("start:", start_position)
print("obstacle:", obstacle_position)
print("end:", end_position)
print("SWEPT COLLISION TEST: PASS")