
import math
import random


class Warehouse:

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.obstacles = []
        self.goal_position = None
        self.boundary = []
        self.walkable_cells = []
        self.terrain_costs = {}





def generate_warehouse_shape(warehouse):

    center_x = warehouse.width / 2
    center_y = warehouse.height / 2

    number_of_points = random.randint(5, 10)

    angles = sorted(
        [
            random.uniform(0, 2 * math.pi)
            for _ in range(number_of_points)
        ]
    )

    points = []

    for angle in angles:

        radius_x = random.uniform(
            warehouse.width * 0.40,
            warehouse.width * 0.49
        )

        radius_y = random.uniform(
            warehouse.height * 0.40,
            warehouse.height * 0.49
        )

        x = center_x + radius_x * math.cos(angle)
        y = center_y + radius_y * math.sin(angle)

        points.append((x, y))

    walkable_cells = set()

    for y in range(1, warehouse.height - 1):

        for x in range(1, warehouse.width - 1):

            inside = False
            j = len(points) - 1

            for i in range(len(points)):

                xi, yi = points[i]
                xj, yj = points[j]

                if (
                    ((yi > y) != (yj > y))
                    and
                    (x < (xj - xi) * (y - yi) / (yj - yi) + xi)
                ):
                    inside = not inside

                j = i

            if inside:
                walkable_cells.add((x, y))

    warehouse.walkable_cells = list(walkable_cells)

    boundary_cells = set()

    for cell in walkable_cells:

        x, y = cell

        neighbors = [
            (x, y + 1),
            (x, y - 1),
            (x - 1, y),
            (x + 1, y)
        ]

        for neighbor in neighbors:

            if neighbor not in walkable_cells:

                boundary_cells.add(cell)
                break

    warehouse.boundary = list(boundary_cells)


def generate_terrain(warehouse, floor_cells):

    terrain_types = {
        "normal": 1,
        "rough": 3,
        "very_rough": 5
    }

    for cell in floor_cells:

        terrain = random.choices(
            list(terrain_types.keys()),
            weights=[0.60, 0.25, 0.15]
        )[0]

        warehouse.terrain_costs[cell] = terrain_types[terrain]






def generate_obstacles(
        warehouse,
        number_of_obstacles,
        floor_cells,
        robot_position
):

    obstacles = set()

    max_possible = len(floor_cells) - 2

    number_of_obstacles = min(
        number_of_obstacles,
        max_possible
    )

    number_of_obstacles = max(
        number_of_obstacles,
        0
    )

    while len(obstacles) < number_of_obstacles:

        obstacle_position = random.choice(floor_cells)

        if (
            obstacle_position != robot_position
            and
            obstacle_position != warehouse.goal_position
        ):

            obstacles.add(obstacle_position)

    return obstacles



