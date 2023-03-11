# Imports
import pygame
import math
import numpy as np
import time

class Dijkstra:
    def __init__(self,screen_size):
        pygame.init()
        self.screen = pygame.display.set_mode(screen_size)

        # Define Colors
        self.background=(255,255,255)

        # Define Grid Size
        self.gridsize=1
    
    # Main Loop
    def game():
        running=True
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running=False
        # quit pygame
        pygame.quit()