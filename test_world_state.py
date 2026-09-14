from simulation import Simulation


simulation = Simulation()
simulation.generate_new_warehouse("Medium")

observed_obstacles = simulation.sensor.get_observed_obstacles(
    simulation.robot.position
)

simulation.world_state.update_obstacles(
    observed_obstacles
)

stored_obstacles = simulation.world_state.get_obstacles()

assert stored_obstacles == observed_obstacles

print("observed obstacles:", len(observed_obstacles))
print("stored obstacles:", len(stored_obstacles))
print("WORLD STATE TEST: PASS")