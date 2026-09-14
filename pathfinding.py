import heapq
from collections import deque


class Planner:

    def plan(
        self,
        robot_position,
        goal_position,
        obstacles,
        warehouse,
        robot_radius=0.5
    ):
        raise NotImplementedError


class DijkstraPlanner(Planner):

    def plan(
        self,
        robot_position,
        goal_position,
        obstacles,
        warehouse,
        robot_radius=0.5
    ):
        return dijkstra(
            robot_position,
            goal_position,
            obstacles,
            warehouse,
             robot_radius=robot_radius
        )


def manhattan_distance(position_a, position_b):

    return (
        abs(position_a[0] - position_b[0])
        +
        abs(position_a[1] - position_b[1])
    )


class AStarPlanner(Planner):

    def __init__(self, heuristic=None):

        self.heuristic = heuristic or manhattan_distance

    def plan(
        self,
        robot_position,
        goal_position,
        obstacles,
        warehouse,
        robot_radius=0.5
    ):
        return a_star(
            robot_position,
            goal_position,
            obstacles,
            warehouse,
            heuristic=self.heuristic,
            robot_radius=robot_radius
        )


def bfs(
    robot_position,
    goal_position,
    obstacles,
    warehouse
):

    positions_to_check = deque([robot_position])
    visited = {robot_position}

    previous_position = {}

    while positions_to_check:

        current_position = positions_to_check.popleft()

        if current_position == goal_position:

            path = []
            current = goal_position

            while current != robot_position:

                path.append(current)
                current = previous_position[current]

            path.append(robot_position)
            path.reverse()

            return path, len(visited)

        x = current_position[0]
        y = current_position[1]

        neighbor_positions = [
            (x, y + 1),
            (x, y - 1),
            (x - 1, y),
            (x + 1, y)
        ]

        for neighbor in neighbor_positions:

            if (
                neighbor in warehouse.walkable_cells
                and neighbor not in warehouse.boundary
                and neighbor not in obstacles
                and neighbor not in visited
            ):

                positions_to_check.append(neighbor)
                visited.add(neighbor)

                previous_position[neighbor] = current_position

    return None, len(visited)



def is_position_valid_for_robot(
    position,
    obstacles,
    warehouse,
    robot_radius
):

    for obstacle in obstacles:

        obstacle_center_x = obstacle[0]
        obstacle_center_y = obstacle[1]

        closest_x = max(
            obstacle_center_x - 0.5,
            min(position[0], obstacle_center_x + 0.5)
        )

        closest_y = max(
            obstacle_center_y - 0.5,
            min(position[1], obstacle_center_y + 0.5)
        )

        distance_x = position[0] - closest_x
        distance_y = position[1] - closest_y

        distance_squared = (
            distance_x ** 2
            +
            distance_y ** 2
        )

        if distance_squared < robot_radius ** 2:
            return False

    return True

# ============================================================
# Dijkstra
# ============================================================

def dijkstra(
    robot_position,
    goal_position,
    obstacles,
    warehouse,
    robot_radius=0.5
):

    priority_queue = [(0, robot_position)]

    distances = {
        robot_position: 0
    }

    previous_position = {}

    while priority_queue:

        current_cost, current_position = heapq.heappop(
            priority_queue
        )

        if (
            current_position in distances
            and current_cost > distances[current_position]
        ):
            continue

        if current_position == goal_position:

            path = []
            current = goal_position

            while current != robot_position:

                path.append(current)
                current = previous_position[current]

            path.append(robot_position)
            path.reverse()

            return path, len(distances)

        x = current_position[0]
        y = current_position[1]

        neighbor_positions = [
            (x, y + 1),
            (x, y - 1),
            (x - 1, y),
            (x + 1, y)
        ]

        for neighbor in neighbor_positions:
            if (
                neighbor in warehouse.walkable_cells
                and neighbor not in warehouse.boundary
                and neighbor not in obstacles
                and is_position_valid_for_robot(
                    neighbor,
                    obstacles,
                    warehouse,
                    robot_radius
                )
            ):

                new_cost = (
                    current_cost
                    + warehouse.terrain_costs[neighbor]
                )

                if (
                    neighbor not in distances
                    or new_cost < distances[neighbor]
                ):

                    distances[neighbor] = new_cost
                    previous_position[neighbor] = current_position

                    heapq.heappush(
                        priority_queue,
                        (new_cost, neighbor)
                    )

    return None, len(distances)


# ============================================================
# A*
# ============================================================

def a_star(
    robot_position,
    goal_position,
    obstacles,
    warehouse,
    heuristic=None,
    robot_radius=0.5
):

    if heuristic is None:
        heuristic = manhattan_distance

    priority_queue = [(0, 0, robot_position)]

    distances = {
        robot_position: 0
    }

    previous_position = {}

    while priority_queue:
        current_f_cost, current_g_cost, current_position = heapq.heappop(
            priority_queue
        )

        if current_g_cost > distances[current_position]:
            continue

        if current_position == goal_position:

            path = []
            current = goal_position

            while current != robot_position:

                path.append(current)
                current = previous_position[current]

            path.append(robot_position)
            path.reverse()

            return path, len(distances)

        x = current_position[0]
        y = current_position[1]

        neighbor_positions = [
            (x, y + 1),
            (x, y - 1),
            (x - 1, y),
            (x + 1, y)
        ]

        for neighbor in neighbor_positions:

            if (
                neighbor in warehouse.walkable_cells
                and neighbor not in warehouse.boundary
                and neighbor not in obstacles
                and is_position_valid_for_robot(
                    neighbor,
                    obstacles,
                    warehouse,
                    robot_radius
                )
            ):

                g_cost = (
                    distances[current_position]
                    + warehouse.terrain_costs[neighbor]
                )

                h_cost = heuristic(
                    neighbor,
                    goal_position
                )

                f_cost = g_cost + h_cost

                if (
                    neighbor not in distances
                    or g_cost < distances[neighbor]
                ):

                    distances[neighbor] = g_cost
                    previous_position[neighbor] = current_position

                    heapq.heappush(
                        priority_queue,
                        (f_cost, g_cost, neighbor)
                    )

    return None, len(distances)


def calculate_path_cost(path, warehouse):

    total_cost = 0

    for position in path[1:]:

        total_cost += warehouse.terrain_costs[position]

    return total_cost

