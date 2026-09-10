import heapq
from collections import deque


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





#============================================================   
# Dijkstra 
#============================================================

def dijkstra(
        robot_position,
        goal_position,
        obstacles,
        warehouse
):

    priority_queue = [(0, robot_position)]

    distances = {robot_position: 0}

    previous_position = {}

    while priority_queue:

        current_cost, current_position = heapq.heappop(priority_queue)

        if current_position in distances and current_cost > distances[current_position]:
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
            ):

                distance = (
                    current_cost
                    + warehouse.terrain_costs[neighbor]
                )

                if (
                    neighbor not in distances
                    or distance < distances[neighbor]
                ):

                    distances[neighbor] = distance
                    previous_position[neighbor] = current_position

                    heapq.heappush(priority_queue, (distance, neighbor))

    return None, len(distances)



#============================================================
# A* 
#============================================================

def a_star(
        robot_position,
        goal_position,
        obstacles,
        warehouse
):

    priority_queue = [(0, robot_position)]

    distances = {robot_position: 0}
    previous_position = {}

    while priority_queue:

        current_cost, current_position = heapq.heappop(priority_queue)

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
            ):
                g_cost = (
                    distances[current_position]
                    + warehouse.terrain_costs[neighbor]
                )

                h_cost = (
                    abs(neighbor[0] - goal_position[0])
                    +
                    abs(neighbor[1] - goal_position[1])
                )


                f_cost = g_cost + h_cost


                if (
                    neighbor not in distances
                    or g_cost < distances[neighbor]
                ):
                    distances[neighbor] = g_cost
                    previous_position[neighbor] = current_position
                    heapq.heappush(priority_queue, (f_cost, neighbor))

    return None, 0




def find_path(
        algorithm,
        robot_position,
        goal_position,
        obstacles,
        warehouse
):

    if algorithm == "BFS":

        return bfs(
            robot_position,
            goal_position,
            obstacles,
            warehouse
        )

    elif algorithm == "Dijkstra":

        return dijkstra(
            robot_position,
            goal_position,
            obstacles,
            warehouse
        )

    elif algorithm == "A*":

        return a_star(
            robot_position,
            goal_position,
            obstacles,
            warehouse
        )

    else:

        return None, 0




def calculate_path_cost(path, warehouse):

    total_cost = 0

    for position in path[1:]:

        total_cost += warehouse.terrain_costs[position]

    return total_cost

