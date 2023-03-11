# Imports
import pygame
import math
import numpy as np
import time

class Dijkstra:
    def __init__(self,width,height,scale,start,goal):
        pygame.init()
        
        # Define constants
        self.START=start
        self.GOAL=goal
        self.SCALE=scale
        self.WIDTH=width
        self.HEIGHT=height
        self.screen_size=self.get_screen_size(width,height,scale)
        self.screen = pygame.display.set_mode(self.screen_size)

        # Define Colors
        self.background=(255,255,255)
        self.obst_color=(0,0,0)
        self.clear_color=(243, 228, 20, 0.8)
        self.grid_color=(233, 226, 230, 0.2)

        # Define Grid Size
        self.grid_size=5

        self.H_PX=self.WIDTH*self.SCALE/self.grid_size
        self.V_PX=self.HEIGHT*self.SCALE/self.grid_size

        
    
    # Main Loop
    def game(self):
        self.running=True
        self.drawgrid()
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running=False
        # quit pygame
        pygame.quit()

    def get_screen_size(self,width,height,scale):
    #     10pixel =1mm
        screen_size=(width*scale,height*scale)
        
        return screen_size
    def drawgrid(self):
        for row in range(int(self.V_PX)):
            for col in range(int(self.H_PX)):
                # Calculate the position and size of the current cell
                x = col * self.grid_size
                y = row * self.grid_size
                cell_rect = pygame.Rect(x, y, self.grid_size, self.grid_size)

                # Draw the cell as a black rectangle with a white border
                pygame.draw.rect(self.screen, self.grid_color, cell_rect, 1)
                pygame.draw.rect(self.screen, self.background, cell_rect.inflate(-1, -1))
        pygame.display.update()



if __name__ == "__main__":
    # Create an instance of Dijkstra
    d_algo = Dijkstra(600,250,2,(0,0),(20,20))
    
    # Call the game method
    d_algo.game()