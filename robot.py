
class Robot:

    def __init__(self, x, y):
        self.position = (x, y)

    def move_up(self):
        return (self.position[0], self.position[1] + 1)

    def move_down(self):
        return (self.position[0], self.position[1] - 1)

    def move_left(self):
        return (self.position[0] - 1, self.position[1])

    def move_right(self):
        return (self.position[0] + 1, self.position[1])
