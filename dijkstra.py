# Imports
import pygame
import math
import numpy as np
import time
from queue import PriorityQueue


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
        self.draw_obstacles()
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

    def draw_triangle(self,p1,p2,p3, color):
        # Define the vertices of the triangle
        vertices = [p1, p2, p3]
        # Draw the triangle
        pygame.draw.polygon(self.screen, color , vertices)
    
    def draw_heaxagon(self, cx, cy, sl, color):
        # Calculate the points of the hexagon
        center_x = cx*self.SCALE
        center_y = cy*self.SCALE
        side_length = sl*self.SCALE
        angle_deg = 60
        angle_rad = math.radians(angle_deg)
        angle_deg = 60
        angle_rad = math.radians(angle_deg)
        points = []
        for i in range(6):
            x = round(center_x + side_length * math.cos(angle_rad * i))
            y = round(center_y + side_length * math.sin(angle_rad * i))
            points.append((x, y))

        # Draw and fill the hexagon
        pygame.draw.polygon(self.screen, color, points, 0)
        
    def draw_rectangle(self, x, y, w, h, color):
        x,y,w,h=x*self.SCALE,y*self.SCALE,w*self.SCALE,h*self.SCALE
        rect = pygame.Rect(x, y, w, h)
        pygame.draw.rect(self.screen, color, rect)

    def draw_obstacles(self):
        # Draw Clearance
        
        self.draw_heaxagon(300,125,60, self.clear_color)
        # Triangle
        p1=(455*self.SCALE,(250-20)*self.SCALE)
        p2=(455*self.SCALE,20*self.SCALE)
        p3=(465*self.SCALE,(250-20)*self.SCALE)
        p4=(465*self.SCALE,20*self.SCALE)
        p5=(515*self.SCALE,125*self.SCALE)
        vertices = [p1, p2, p4, p5,p3]
        pygame.draw.polygon(self.screen, self.clear_color, vertices)

        self.draw_rectangle(100-5,0-5,50+10,100+10, self.clear_color)
        self.draw_rectangle(100-5,250-100-5,50+10,100+10, self.clear_color)

        # Draw obstacles
        self.draw_heaxagon(300,125,50, self.obst_color)
        p1=(460*self.SCALE,(250-25)*self.SCALE)
        p2=(460*self.SCALE,25*self.SCALE)
        p3=(510*self.SCALE,125*self.SCALE)
        self.draw_triangle(p1,p2,p3,self.obst_color)
        self.draw_rectangle(100,0,50,100, self.obst_color)
        self.draw_rectangle(100,250-100,50,100, self.obst_color)

        pygame.display.update()

    def check_grid(self,x,y):
        for i in range(y*self.grid_size+1,y*self.grid_size+self.grid_size):
            for j in range(x*self.grid_size,x*self.grid_size+self.grid_size):
                color = self.screen.get_at((i, j)) 
                if color==self.clear_color or color==self.obst_color :
                    return True
        self.change_color((245, 40, 145, 0.8),x,y,self.grid_size)
        return False
            
    def change_color(self, color, x, y):
        cell_rect = pygame.Rect(y*self.grid_size, x*self.grid_size, self.grid_size, self.grid_size)
        pygame.draw.rect(self.screen, self.grid_color, cell_rect, 1)
        pygame.draw.rect(self.screen, color, cell_rect.inflate(-1, -1))
        pygame.display.update()
    
    def dijkstra(self):
        idx=1
        start=(0,0,0,self.START)
        self.change_color(self.start_color,self.START[0],self.START[1])
        goal=(self.H_PX+self.V_PX,0,0,self.GOAL)
        
        # Define action sets
        actions=[[0,1],[1,0],[0,-1],[-1,0],[1,1],[-1,-1],[-1,1],[1,-1]]
        # Create a open list
        open_list = PriorityQueue()
        # Create a close list
        close_list=set()
        # Create a tracker for path node with compatible data type
        tracker=[]
        current=start
        open_list.put(start)
        
        while open_list and not current[3]==goal[3]:
            current=open_list.get()
            tracker.append(current)
            if current[3] in close_list:
                    continue
            if current[3]==goal[3]:
                    break
            close_list.add(current[3])
            for a in actions:
                    cost=current[0]
                    parent_idx=current[2]
                    neighbor=(current[3][0]+a[0],current[3][1]+a[1])
                    x,y=neighbor
                    if x>=0 and y>=0 and x<self.V_PX and y<self.H_PX and not self.check_grid(x,y):
                            if neighbor in close_list:
                                continue
                            if a[0]==0 or a[1]==0:
                                cost=cost+1
                            else:
                                cost=cost+1.4
                            idx=idx+1
                            open_list.put((cost,parent_idx,idx,neighbor))
        self.change_color(self.goal_color,self.GOAL[0],self.GOAL[1])
    #     print(current)
        g_node=current
        path=[]
        while not g_node[2]==0:
            for c in tracker:
                if c[2] == g_node[1]:
    #                 print(c)
                    change_color((25, 252, 255, 1),c[3][0],c[3][1],grid_size)
                    path.append([c[3][0],c[3][1]])
                    g_node=c
        print("Complete")
        path.reverse()
        for x,y in path:
            time.sleep(0.05)
            pygame.draw.circle(screen, (34, 203, 203, 1), (y*grid_size,x*grid_size), 5)
            pygame.display.update()

if __name__ == "__main__":
    # Create an instance of Dijkstra
    d_algo = Dijkstra(600,250,2,(0,0),(20,20))
    
    # Call the game method
    d_algo.game()