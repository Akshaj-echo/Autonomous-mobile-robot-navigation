from world_state import WorldState
from obstacle_representation import ObstacleRepresentation


world_state = WorldState()

test_obstacles = {
    (5, 5),
    (10, 10)
}

world_state.update_obstacles(test_obstacles)

representation = ObstacleRepresentation(world_state)

assert representation.get_obstacles() == test_obstacles

print("OBSTACLE REPRESENTATION TEST: PASS")