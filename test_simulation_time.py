from simulation import Simulation


simulation = Simulation()

simulation.generate_new_warehouse("Medium")

simulation.calculate_path("A*")

dt = 0.1

success = simulation.run_navigation(
    dt=dt,
    max_steps=10000
)

print("success:", success)
print("simulation time:", simulation.simulation_time)

assert success
assert simulation.simulation_time > 0

expected_time_steps = round(
    simulation.simulation_time / dt
)

assert expected_time_steps > 0

print("SIMULATION TIME TEST: PASS")