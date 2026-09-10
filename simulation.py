print("SIMULATION.PY LOADED")


import random

from warehouse import (
    Warehouse,
    generate_warehouse_shape,
    generate_terrain,
    generate_obstacles
)

from robot import Robot

from pathfinding import (
    find_path,
    calculate_path_cost
)


class Simulation:

    def __init__(self):

        self.warehouse = None
        self.robot = None

        self.difficulty = None
        self.algorithm = None

        self.path = None
        self.path_cost = 0
        self.cells_explored = 0

        self.moves = 0
        self.map_generation_attempts = 0

    def generate_new_warehouse(self, difficulty):

        self.difficulty = difficulty

        difficulty_density = {
            "Easy": 0.10,
            "Medium": 0.20,
            "Hard": 0.30
        }

        obstacle_density = difficulty_density[difficulty]

        while True:

            width = random.randint(40, 60)
            height = random.randint(25, 40)

            warehouse = Warehouse(width, height)

            generate_warehouse_shape(warehouse)

            floor_cells = [
                cell
                for cell in warehouse.walkable_cells
                if cell not in warehouse.boundary
            ]

            if len(floor_cells) < 20:
                continue

            generate_terrain(
                warehouse,
                floor_cells
            )

            robot_position = random.choice(floor_cells)

            robot = Robot(
                robot_position[0],
                robot_position[1]
            )

            goal_position = max(
                floor_cells,
                key=lambda cell:
                abs(cell[0] - robot.position[0])
                +
                abs(cell[1] - robot.position[1])
            )

            warehouse.goal_position = goal_position

            number_of_obstacles = int(
                len(floor_cells) * obstacle_density
            )

            attempts = 0

            while True:

                warehouse.obstacles = generate_obstacles(
                    warehouse,
                    number_of_obstacles,
                    floor_cells,
                    robot.position  
                )

                attempts += 1

                print("Obstacle attempt:", attempts)

                test_path, _ = find_path(
                    "BFS",
                    robot.position,
                    warehouse.goal_position,
                    warehouse.obstacles,
                    warehouse
                )

                if test_path is not None:
                    break

            self.warehouse = warehouse
            self.robot = robot
            self.map_generation_attempts = attempts

            self.path = None
            self.path_cost = 0
            self.cells_explored = 0
            self.moves = 0

            return

    def calculate_path(self, algorithm):

        self.algorithm = algorithm

        self.path, self.cells_explored = find_path(
            algorithm,
            self.robot.position,
            self.warehouse.goal_position,
            self.warehouse.obstacles,
            self.warehouse
        )

        if self.path is not None:

            self.path_cost = calculate_path_cost(
                self.path,
                self.warehouse
            )

        else:

            self.path_cost = 0

        return self.path

    def move_robot(self, new_position):

        if new_position not in self.warehouse.walkable_cells:
            return False

        if new_position in self.warehouse.boundary:
            return False

        if new_position in self.warehouse.obstacles:
            return False

        self.robot.position = new_position
        self.moves += 1

        return True