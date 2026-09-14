class WorldState:

    def __init__(self):

        self.observed_obstacles = set()

    def update_obstacles(self, observed_obstacles):

        self.observed_obstacles = set(observed_obstacles)

    def get_obstacles(self):

        return self.observed_obstacles