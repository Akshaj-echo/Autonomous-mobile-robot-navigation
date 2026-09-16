class ObstacleRepresentation:

    def __init__(self, world_state):

        self.world_state = world_state

    def get_obstacles(self):

        return self.world_state.get_obstacles()