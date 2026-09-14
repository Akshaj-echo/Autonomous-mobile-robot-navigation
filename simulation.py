import math
import random
import time
from world_state import WorldState
from controller import Controller
from sensor import SimulatedSensor
from warehouse import (
    Warehouse,
    DynamicObstacle,
    generate_warehouse_shape,
    generate_terrain,
    generate_obstacles
)

from robot import Robot

from pathfinding import (
    bfs,
    calculate_path_cost,
    DijkstraPlanner,
    AStarPlanner
)


import warehouse


class Simulation:

    def __init__(self):

        self.warehouse = None
        self.robot = None
        self.controller = Controller()
        self.sensor = None
        self.world_state = WorldState()
        self.difficulty = None
        self.algorithm = None
        self.planner = None

        self.path = None
        self.previous_waypoint = None
        self.path_cost = 0
        self.cells_explored = 0

        self.moves = 0
        self.map_generation_attempts = 0
        self.collision_count = 0
        self.was_colliding = False
        self.path_invalidated = False
        self.replan_count = 0
        self.navigation_failed = False
        self.planning_time = 0.0
        self.replanning_time = 0.0
        self.simulation_time = 0.0






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
            max_obstacle_attempts = 100

            while attempts < max_obstacle_attempts:

                attempts += 1
                warehouse.dynamic_obstacles = []
                environment_generated = False

                warehouse.obstacles = generate_obstacles(
                    warehouse,
                    number_of_obstacles,
                    floor_cells,
                    robot.position
                )

                static_path = bfs(
                    robot.position,
                    warehouse.goal_position,
                    warehouse.obstacles,
                    warehouse
                )

                if static_path is None:
                    continue

                dynamic_obstacle = None

                for _ in range(100):

                    dynamic_x, dynamic_y = random.choice(
                        warehouse.walkable_cells
                    )

                    candidate_position = (
                        dynamic_x,
                        dynamic_y
                    )

                    if (
                        candidate_position == robot.position
                        or
                        candidate_position == warehouse.goal_position
                        or
                        candidate_position in warehouse.obstacles
                    ):
                        continue

                    distance_x = (
                        candidate_position[0]
                        - robot.position[0]
                    )

                    distance_y = (
                        candidate_position[1]
                        - robot.position[1]
                    )

                    distance_squared = (
                        distance_x ** 2
                        +
                        distance_y ** 2
                    )

                    combined_radius = (
                        robot.collision_radius
                        + 0.5
                    )

                    if distance_squared > combined_radius ** 2:

                        dynamic_obstacle = DynamicObstacle(
                            candidate_position[0],
                            candidate_position[1],
                            velocity_x=0.5,
                            velocity_y=0,
                            collision_radius=0.5
                        )

                        break

                if dynamic_obstacle is None:
                    continue

                warehouse.add_dynamic_obstacle(dynamic_obstacle)

                all_obstacles = set(warehouse.obstacles)

                dynamic_position = (
                    round(dynamic_obstacle.position[0]),
                    round(dynamic_obstacle.position[1])
                )

                all_obstacles.add(dynamic_position)

                initial_path, _ = AStarPlanner().plan(
                    robot.position,
                    warehouse.goal_position,
                    all_obstacles,
                    warehouse,
                    robot_radius=robot.collision_radius
                )

                if initial_path is None:
                    warehouse.dynamic_obstacles = []
                    continue

                environment_generated = True
                break

            if not environment_generated:
                continue

            self.warehouse = warehouse
            self.sensor = SimulatedSensor(self.warehouse)
            self.robot = robot

            self.map_generation_attempts = attempts

            self.path = None
            self.path_cost = 0
            self.cells_explored = 0
            self.moves = 0
            self.collision_count = 0
            self.path_invalidated = False
            self.replan_count = 0
            self.planning_time = 0.0
            self.replanning_time = 0.0
            self.simulation_time = 0.0
            
            return


    def get_robot_planning_position(self):

        return (
            round(self.robot.position[0]),
            round(self.robot.position[1])
        )




    


    def calculate_path(self, algorithm, is_replan=False):

        self.algorithm = algorithm

        all_obstacles = set(self.warehouse.obstacles)

        for dynamic_obstacle in self.warehouse.dynamic_obstacles:

            dynamic_x = dynamic_obstacle.position[0]
            dynamic_y = dynamic_obstacle.position[1]

            combined_radius = (
                self.robot.collision_radius
                + dynamic_obstacle.collision_radius
            )

            center_x = round(dynamic_x)
            center_y = round(dynamic_y)

            radius_in_cells = math.ceil(combined_radius)

            for dx in range(
                -radius_in_cells,
                radius_in_cells + 1
            ):
                for dy in range(
                    -radius_in_cells,
                    radius_in_cells + 1
                ):

                    candidate = (
                        center_x + dx,
                        center_y + dy
                    )

                    distance_squared = (
                        (candidate[0] - dynamic_x) ** 2
                        +
                        (candidate[1] - dynamic_y) ** 2
                    )

                    if distance_squared <= combined_radius ** 2:
                        all_obstacles.add(candidate)
        if algorithm == "Dijkstra":
            self.planner = DijkstraPlanner()

        elif algorithm == "A*":
            self.planner = AStarPlanner()

        else:
            self.planner = None
            return None

        start_time = time.perf_counter()

        self.path, self.cells_explored = self.planner.plan(
            self.get_robot_planning_position(),
            self.warehouse.goal_position,
            all_obstacles,
            self.warehouse,
            robot_radius=self.robot.collision_radius
        )

        elapsed_time = time.perf_counter() - start_time

        if is_replan:
            self.replanning_time += elapsed_time
        else:
            self.planning_time = elapsed_time

        if self.path is not None:
            self.path_cost = calculate_path_cost(
                self.path,
                self.warehouse
            )

            if len(self.path) >= 2:
                self.previous_waypoint = self.path[0]

        else:
            self.path_cost = 0
            self.previous_waypoint = None

        return self.path



   




    def is_path_valid(self):

        if self.path is None:
            return False

        # Check static obstacles.
        for position in self.path:

            if position in self.warehouse.obstacles:
                return False

        # Check whether any dynamic obstacle currently overlaps
        # a waypoint on the planned path.
        for dynamic_obstacle in self.warehouse.dynamic_obstacles:

            obstacle_x = dynamic_obstacle.position[0]
            obstacle_y = dynamic_obstacle.position[1]

            combined_radius = (
                self.robot.collision_radius
                + dynamic_obstacle.collision_radius
            )

            for position in self.path:

                distance_x = (
                    position[0]
                    - obstacle_x
                )

                distance_y = (
                    position[1]
                    - obstacle_y
                )

                distance_squared = (
                    distance_x ** 2
                    +
                    distance_y ** 2
                )

                if distance_squared < combined_radius ** 2:
                    return False

        return True








    def is_dynamic_obstacle_blocking_path(self):

        if self.path is None:
            return False

        if len(self.path) == 0:
            return False

        for dynamic_obstacle in self.warehouse.dynamic_obstacles:

            dynamic_position = (
                round(dynamic_obstacle.position[0]),
                round(dynamic_obstacle.position[1])
            )

            if dynamic_position in self.path:

                return True

        return False










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


    def is_dynamic_obstacle_path_colliding_with_robot(
        self,
        start_position,
        end_position,
        obstacle_radius
    ):

        robot_position = self.robot.position

        combined_radius = (
            self.robot.collision_radius
            + obstacle_radius
        )

        distance_x = (
            end_position[0]
            - start_position[0]
        )

        distance_y = (
            end_position[1]
            - start_position[1]
        )

        distance = math.sqrt(
            distance_x ** 2
            +
            distance_y ** 2
        )

        if distance == 0:
            return (
                (
                    robot_position[0]
                    - start_position[0]
                ) ** 2
                +
                (
                    robot_position[1]
                    - start_position[1]
                ) ** 2
            ) <= combined_radius ** 2

        sample_step = self.robot.collision_radius / 2

        sample_count = max(
            1,
            math.ceil(distance / sample_step)
        )

        for i in range(sample_count + 1):

            t = i / sample_count

            obstacle_x = (
                start_position[0]
                + distance_x * t
            )

            obstacle_y = (
                start_position[1]
                + distance_y * t
            )

            distance_to_robot_squared = (
                (
                    robot_position[0]
                    - obstacle_x
                ) ** 2
                +
                (
                    robot_position[1]
                    - obstacle_y
                ) ** 2
            )

            if distance_to_robot_squared <= combined_radius ** 2:
                return True

        return False


    def update_dynamic_obstacles(self, dt):

        is_colliding = False

        for obstacle in self.warehouse.dynamic_obstacles:

            old_position = obstacle.position

            proposed_position = (
                old_position[0]
                + obstacle.velocity[0] * dt,
                old_position[1]
                + obstacle.velocity[1] * dt
            )

            if not self.warehouse.is_valid_dynamic_obstacle_position(
                proposed_position
            ):
                obstacle.velocity = (
                    -obstacle.velocity[0],
                    -obstacle.velocity[1]
                )

                proposed_position = (
                    old_position[0]
                    + obstacle.velocity[0] * dt,
                    old_position[1]
                    + obstacle.velocity[1] * dt
                )

            if self.is_dynamic_obstacle_path_colliding_with_robot(
                old_position,
                proposed_position,
                obstacle.collision_radius
            ):
                obstacle.velocity = (
                    -obstacle.velocity[0],
                    -obstacle.velocity[1]
                )

                proposed_position = (
                    old_position[0]
                    + obstacle.velocity[0] * dt,
                    old_position[1]
                    + obstacle.velocity[1] * dt
                )

                if self.is_dynamic_obstacle_path_colliding_with_robot(
                    old_position,
                    proposed_position,
                    obstacle.collision_radius
                ):
                    proposed_position = old_position

            obstacle.position = proposed_position

        if self.path is not None:
            self.path_invalidated = (
                not self.is_path_valid()
                or self.is_dynamic_obstacle_blocking_path()
            )

        return is_colliding



        
    def replan_if_needed(self):

        if not self.path_invalidated:
            return False

        if self.algorithm is None:
            return False

        old_path = self.path

        new_path = self.calculate_path(
            self.algorithm,
            is_replan=True
        )

        if new_path is None:
            self.path = old_path
            return False

        if not self.is_path_valid():

            print("ERROR: PLANNER RETURNED INVALID PATH")
            print("robot position:", self.robot.position)
            print("planning position:", self.get_robot_planning_position())
            print("new path length:", len(new_path))
            print("first 5 waypoints:", new_path[:5])

            self.path = old_path

            return False

        self.path_invalidated = False

        self.replan_count += 1

        return True




    


    def execute_path_step(self, dt):

        if self.path is None:
            return False

        if len(self.path) == 0:
            self.robot.update_velocity(0.0, dt)
            return False

        target_position = self.path[0]

        distance_x = (
            target_position[0]
            - self.robot.position[0]
        )

        distance_y = (
            target_position[1]
            - self.robot.position[1]
        )

        distance = math.sqrt(
            distance_x ** 2
            +
            distance_y ** 2
        )

        # Current waypoint reached.
        if self.controller.is_waypoint_reached(
            self.robot,
            target_position
        ):

            if not self.is_robot_position_safe(
                target_position
            ):
                self.path_invalidated = True
                self.robot.update_velocity(0.0, dt)
                return False

            self.robot.position = target_position

            # Goal reached.
            if len(self.path) == 1:
                self.path.pop(0)
                self.robot.update_velocity(0.0, dt)
                return True

            self.previous_waypoint = self.path[0]
            self.path.pop(0)

            return False

        # The robot is travelling from previous_waypoint
        # toward the current target waypoint.
        if self.previous_waypoint is not None:

            direction_x = (
                target_position[0]
                - self.previous_waypoint[0]
            )

            direction_y = (
                target_position[1]
                - self.previous_waypoint[1]
            )

            target_heading = math.degrees(
                math.atan2(
                    direction_y,
                    direction_x
                )
            )

        else:

            target_heading = math.degrees(
                math.atan2(
                    distance_y,
                    distance_x
                )
            )


        target_velocity = self.controller.get_target_velocity(
            self.robot,
            target_position,
            dt,
            target_heading
        )

        self.robot.update_velocity(
            target_velocity,
            dt
        )
    
        heading_error = (
            target_heading
            - self.robot.heading
            + 180
        ) % 360 - 180

        if abs(heading_error) > 5.0:
            self.robot.velocity = 0.0

        old_position = self.robot.position


        movement_distance = (
            self.robot.velocity * dt
        )

        direction_x = math.cos(
            math.radians(target_heading)
        )

        direction_y = math.sin(
            math.radians(target_heading)
        )
        new_position = (
                    old_position[0]
                    + movement_distance * direction_x,
                    old_position[1]
                    + movement_distance * direction_y
                )
        
        if not self.is_robot_path_safe_from_dynamic_obstacles(
            old_position,
            new_position
        ):

            self.robot.position = old_position
            self.robot.update_velocity(0.0, dt)
            self.path_invalidated = True

            return False

        if not self.is_robot_path_safe(
            old_position,
            new_position
        ):

            self.robot.update_velocity(0.0, dt)
            self.path_invalidated = True

            return False

        if not self.is_robot_position_safe(
            new_position
        ):

            self.robot.update_velocity(0.0, dt)
            self.path_invalidated = True

            return False

        self.robot.position = new_position
        
        return False

        

        







                    
    def run_navigation(self, dt=0.1, max_steps=10000):

        if self.path is None:
            return False

        start_time = time.perf_counter()

        for _ in range(max_steps):

            self.simulation_time += dt

            observed_obstacles = self.sensor.get_observed_obstacles(
                self.robot.position
            )

            self.world_state.update_obstacles(
                observed_obstacles
            )

            if self.controller.is_goal_reached(
                self.robot,
                self.warehouse.goal_position
            ):
                self.robot.update_velocity(0.0, dt)
                return True

            self.update_dynamic_obstacles(dt)

            if self.path_invalidated:

                if not self.replan_if_needed():

                    self.robot.update_velocity(0.0, dt)

                    if self.navigation_failed:
                        return False

                    continue
            self.execute_path_step(dt)

        return False




    




    def is_robot_position_safe(self, position):

        robot_radius = self.robot.collision_radius

        # Check collision with static obstacles.
        for obstacle in self.warehouse.obstacles:

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

        # Check collision with dynamic obstacles.
        if self.warehouse.is_robot_colliding_with_dynamic_obstacle(
            position,
            robot_radius
        ):
            return False

        return True


    def is_robot_path_safe(self, start_position, end_position):

        distance_x = end_position[0] - start_position[0]
        distance_y = end_position[1] - start_position[1]

        distance = (
            distance_x ** 2
            +
            distance_y ** 2
        ) ** 0.5

        if distance == 0:
            return self.is_robot_position_safe(start_position)

        step_size = self.robot.collision_radius / 2

        steps = max(
            1,
            math.ceil(distance / step_size)
        )

        for step in range(steps + 1):

            progress = step / steps

            position = (
                start_position[0] + distance_x * progress,
                start_position[1] + distance_y * progress
            )

            if not self.is_robot_position_safe(position):
                return False

        return True



    def is_robot_path_safe_from_dynamic_obstacles(
    self,
    start_position,
    end_position
):

        robot_radius = self.robot.collision_radius

        for obstacle in self.warehouse.dynamic_obstacles:

            start_obstacle = obstacle.position

            end_obstacle = (
                obstacle.position[0] + obstacle.velocity[0],
                obstacle.position[1] + obstacle.velocity[1]
            )

            steps = max(
                1,
                int(
                    max(
                        abs(end_position[0] - start_position[0]),
                        abs(end_position[1] - start_position[1]),
                        abs(end_obstacle[0] - start_obstacle[0]),
                        abs(end_obstacle[1] - start_obstacle[1])
                    ) / (robot_radius / 2)
                )
            )

            for step in range(steps + 1):

                ratio = step / steps

                robot_position = (
                    start_position[0]
                    + (
                        end_position[0]
                        - start_position[0]
                    ) * ratio,

                    start_position[1]
                    + (
                        end_position[1]
                        - start_position[1]
                    ) * ratio
                )

                obstacle_position = (
                    start_obstacle[0]
                    + (
                        end_obstacle[0]
                        - start_obstacle[0]
                    ) * ratio,

                    start_obstacle[1]
                    + (
                        end_obstacle[1]
                        - start_obstacle[1]
                    ) * ratio
                )

                distance_x = (
                    robot_position[0]
                    - obstacle_position[0]
                )

                distance_y = (
                    robot_position[1]
                    - obstacle_position[1]
                )

                distance_squared = (
                    distance_x ** 2
                    +
                    distance_y ** 2
                )

                combined_radius = (
                    robot_radius
                    + obstacle.collision_radius
                )

                if distance_squared <= combined_radius ** 2:
                    return False

        return True