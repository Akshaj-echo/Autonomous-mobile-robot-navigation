import math


class SimulatedSensor:

    def __init__(self, warehouse, sensing_radius=10.0):

        self.warehouse = warehouse
        self.sensing_radius = sensing_radius

    def get_observed_obstacles(self, robot_position):

        observed_obstacles = set()

        for obstacle in self.warehouse.obstacles:

            distance = math.sqrt(
                (
                    obstacle[0]
                    - robot_position[0]
                ) ** 2
                +
                (
                    obstacle[1]
                    - robot_position[1]
                ) ** 2
            )

            if distance <= self.sensing_radius:

                observed_obstacles.add(obstacle)

        for dynamic_obstacle in self.warehouse.dynamic_obstacles:

            distance = math.sqrt(
                (
                    dynamic_obstacle.position[0]
                    - robot_position[0]
                ) ** 2
                +
                (
                    dynamic_obstacle.position[1]
                    - robot_position[1]
                ) ** 2
            )

            if distance <= self.sensing_radius:

                observed_obstacles.add(
                    (
                        round(dynamic_obstacle.position[0]),
                        round(dynamic_obstacle.position[1])
                    )
                )

        return observed_obstacles