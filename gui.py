import pygame

from simulation import Simulation


class GUI:

    def __init__(self):

        pygame.init()

        self.simulation = Simulation()

        self.simulation.generate_new_warehouse("Medium")

        self.cell_size = 20

        self.screen_width = (
            self.simulation.warehouse.width
            * self.cell_size
        )

        self.screen_height = (
            self.simulation.warehouse.height
            * self.cell_size
        )

        self.screen = pygame.display.set_mode(
            (self.screen_width, self.screen_height)
        )

        pygame.display.set_caption(
            "V4 Autonomous Robot Navigation"
        )

        self.clock = pygame.time.Clock()

    def run(self):

        running = True

        while running:

            for event in pygame.event.get():

                if event.type == pygame.QUIT:
                    running = False

            self.screen.fill((30, 30, 30))

            pygame.display.flip()

            self.clock.tick(60)

        pygame.quit()