import math

import robot


class Controller:

    def __init__(
    self,
    waypoint_tolerance=0.1,
    goal_tolerance=0.2
):
        self.waypoint_tolerance = waypoint_tolerance
        self.goal_tolerance = goal_tolerance






























    def get_target_velocity(
        self,
        robot,
        target_position,
        dt,
        target_heading=None
    ):

        distance_x = target_position[0] - robot.position[0]
        distance_y = target_position[1] - robot.position[1]

        distance = math.sqrt(
            distance_x ** 2 +
            distance_y ** 2
        )

        if distance <= self.waypoint_tolerance:
            return 0.0

        if target_heading is None:
            target_heading = math.degrees(
                math.atan2(distance_y, distance_x)
            )

        robot.update_heading(
            target_heading,
            dt
        )

        heading_error = (
            target_heading
            - robot.heading
            + 180
        ) % 360 - 180

        # Turn in place until sufficiently aligned.
        if abs(heading_error) > 5.0:
            return 0.0

        braking_velocity = math.sqrt(
            2 * robot.max_acceleration * distance
        )

        return min(
            robot.max_velocity,
            braking_velocity
        )








    def is_waypoint_reached(self, robot, target_position):

        distance_x = target_position[0] - robot.position[0]
        distance_y = target_position[1] - robot.position[1]

        distance = math.sqrt(
            distance_x ** 2
            +
            distance_y ** 2
        )

        return distance <= self.waypoint_tolerance


    



    def is_goal_reached(self, robot, goal_position):

        distance_x = goal_position[0] - robot.position[0]
        distance_y = goal_position[1] - robot.position[1]

        distance = math.sqrt(
            distance_x ** 2
            +
            distance_y ** 2
        )

        return distance <= self.goal_tolerance    