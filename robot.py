import math

class Robot:

    def __init__(
        self,
        x,
        y,
        max_velocity=1.0,
        max_acceleration=0.5,
        collision_radius=0.5,
        max_angular_velocity=90.0,
        max_angular_acceleration=180.0
    ):
        self.position = (x, y)

        self.angular_velocity = 0.0
        self.heading = 0.0
        self.velocity = 0.0

        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.collision_radius = collision_radius

        self.max_angular_velocity = max_angular_velocity
        self.max_angular_acceleration = max_angular_acceleration







    def update_velocity(self, target_velocity, dt):
        velocity_change = self.max_acceleration * dt

        if target_velocity > self.velocity:
            self.velocity = min(
                self.velocity + velocity_change,
                target_velocity,
                self.max_velocity
            )

        elif target_velocity < self.velocity:
            self.velocity = max(
                self.velocity - velocity_change,
                target_velocity,
                0
            )





    def update_heading(self, target_heading, dt):

        angle_difference = (
            target_heading - self.heading + 180
        ) % 360 - 180

        angular_velocity_change = (
            self.max_angular_acceleration * dt
        )

        # Stop exactly at the target if the remaining angle
        # can be reached within this timestep.
        if abs(angle_difference) <= abs(self.angular_velocity * dt):

            self.heading = target_heading % 360
            self.angular_velocity = 0.0
            return

        if angle_difference > 0:

            self.angular_velocity = min(
                self.angular_velocity + angular_velocity_change,
                self.max_angular_velocity
            )

        elif angle_difference < 0:

            self.angular_velocity = max(
                self.angular_velocity - angular_velocity_change,
                -self.max_angular_velocity
            )

        else:

            if self.angular_velocity > 0:
                self.angular_velocity = max(
                    0,
                    self.angular_velocity - angular_velocity_change
                )

            elif self.angular_velocity < 0:
                self.angular_velocity = min(
                    0,
                    self.angular_velocity + angular_velocity_change
                )

        self.heading += self.angular_velocity * dt
        self.heading %= 360



    def update_position(self, dt):
        angle = math.radians(self.heading)

        self.position = (
            self.position[0] + self.velocity * math.cos(angle) * dt,
            self.position[1] + self.velocity * math.sin(angle) * dt
        )

    def move_up(self):
        return (self.position[0], self.position[1] + 1)

    def move_down(self):
        return (self.position[0], self.position[1] - 1)

    def move_left(self):
        return (self.position[0] - 1, self.position[1])

    def move_right(self):
        return (self.position[0] + 1, self.position[1])