from robot import Robot


robot = Robot(
    0,
    0,
    max_angular_velocity=90.0,
    max_angular_acceleration=180.0
)

robot.update_heading(90.0, 0.1)

assert robot.heading == 1.8
assert robot.angular_velocity == 18.0

for _ in range(100):
    robot.update_heading(90.0, 0.1)

assert abs(robot.heading - 90.0) < 0.001
assert robot.angular_velocity == 0.0

robot.heading = 359.0

robot.update_heading(1.0, 0.1)

assert robot.heading > 359.0 or robot.heading < 1.0

print("final heading:", robot.heading)
print("angular velocity:", robot.angular_velocity)
print("ROBOT TURNING TEST: PASS")