from simulation import Simulation

print("Starting program...")

simulation = Simulation()

print("Generating warehouse...")

simulation.generate_new_warehouse("Medium")

print("Warehouse generated.")

print(
    "Warehouse size:",
    simulation.warehouse.width,
    "x",
    simulation.warehouse.height
)

print(
    "Robot:",
    simulation.robot.position
)

print(
    "Goal:",
    simulation.warehouse.goal_position
)

simulation.path = simulation.calculate_path("A*")

print(
    "Path length:",
    len(simulation.path) - 1
)

print(
    "Path cost:",
    simulation.path_cost
)

print(
    "Cells explored:",
    simulation.cells_explored
)