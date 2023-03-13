# Imports
import pygame
import math
import numpy as np
import time
from queue import PriorityQueue
import argparse
import cv2

class Dijkstra:
    def __init__(self,width,height,scale,start,goal):
        # Get the video dimensions and FPS
        
        self.result = cv2.VideoWriter("Dijkstra01.avi", cv2.VideoWriter_fourcc(*'MJPG'), 300, (width, height))
        
        # Define constants
        self.START=start
        self.GOAL=goal
        self.SCALE=scale
        self.WIDTH=width
        self.HEIGHT=height
        # self.screen_size=self.get_screen_size(width,height,scale)
        # self.screen = pygame.display.set_mode(self.screen_size)

        # Define Colors
        self.background=(255,255,255)
        self.obst_color=(0,0,0)
        self.clear_color=(255, 255, 245)
        self.grid_color=(233, 226, 230, 0.2)
        self.start_color=(132, 222, 15, 0.8)
        self.goal_color=(254, 0, 0, 0.8)
        self.explored_color= (252, 230, 108)
        self.track_color= (252, 184, 100)
        self.robot_color= (168, 230, 53, 1)

        # Define Grid Size
        self.grid_size=2

        self.H_PX=self.WIDTH*self.SCALE/self.grid_size
        self.V_PX=self.HEIGHT*self.SCALE/self.grid_size

        self.HEX=self.hexagon(125,300,60)
        # Triangle
        p1=((250-20),455)
        p2=(20, 455)
        p3=((250-20),465)
        p4=(20, 465)
        p5=(125,515)
        self.TRI = [p1, p2, p4, p5,p3]
    
    def line_equation(self,p,p1,p2):
        try:
            m=(p2[1]-p1[1])/(p2[0]-p1[0])
            value=p[1]-p1[1]-(m*p[0])+(m*p1[0])
        except:
            value=p[0]-p1[0]
            
        return value
    
    def hexagon(self,cx, cy, sl):
            # Calculate the points of the hexagon
            center_x = cx
            center_y = cy
            side_length = sl
            angle_deg = 60
            angle_rad = math.radians(angle_deg)
            angle_deg = 60
            angle_rad = math.radians(angle_deg)
            points = []
            for i in range(6):
                x = round(center_x + side_length * math.cos(angle_rad * i))
                y = round(center_y + side_length * math.sin(angle_rad * i))
                points.append((x, y))
            return points
    
    def obstacle_detect_triangle(self,p,v1,v2,v3):
        h1=self.line_equation(p,v2,v3)>=0 if self.line_equation(v1,v2,v3)>0 else self.line_equation(p,v2,v3)<=0
        h2=self.line_equation(p,v1,v3)>=0 if self.line_equation(v2,v1,v3)>0 else self.line_equation(p,v1,v3)<=0
        h3=self.line_equation(p,v1,v2)>=0 if self.line_equation(v3,v1,v2)>0 else self.line_equation(p,v1,v2)<=0
        print(h1,h2,h3)
        return h1 and h2 and h3

    def obstacle_detect_rectangle(self,point,x,y,width,height):
        if point[0]>=x and point[1]>=y and point[0]<=(x+width) and point[1]<=(y+height):
            return True
        else:
            return False
    
    def obstacle_detect_boundary(self,point,x,y,width,height):
        if point[0]<=x or point[1]<=y or point[0]>=(x+width) or point[1]>=(y+height):
            return True
        else:
            return False
        
    def obstacle_detect_polygon(self,p,points):
        points=points+points[:3]
        for i in range(len(points)-3):
            v1,v2,v3=points[i],points[i+1],points[i+2]
            h=self.line_equation(p,v1,v2)>=0 if self.line_equation(v3,v1,v2)>0 else self.line_equation(p,v1,v2)<=0
            if not h:
                return False
        return True


    

    # self.draw_rectangle(100-5,0-5,50+10,100+10, self.clear_color)
    # self.draw_self.obstacle_detect_rectangle(100-5,250-100-5,50+10,100+10, self.clear_color)
            
    def check_obstacle(self,p):
        h1=self.obstacle_detect_rectangle(p,0,100,100,50)
        h2=self.obstacle_detect_rectangle(p,150,100,100,50)
        h3=self.obstacle_detect_polygon(p,self.HEX)
        h4=self.obstacle_detect_polygon(p,self.TRI)
        h5= self.obstacle_detect_boundary(p,5,5,240,590)
        
        return (h1 or h2 or h3 or h4 or h5)

    def make_obstacle_space(self):
        grid=np.ones((self.HEIGHT,self.WIDTH,3),np.uint8)
        nodes=[]
        for i in range(self.HEIGHT):
            row=[]
            for j in  range(self.WIDTH):
                if self.check_obstacle((i,j)):
                    row.append([-1,None,None,(i,j)])
                    grid[i][j]=np.array(self.obst_color)
                else:
                    row.append([float('inf'),None,None,(i,j)])
                    grid[i][j]=np.array(self.background)
            nodes.append(row)
        return grid,nodes
    
    def game(self):
        self.running=True
        if self.START[0]>=0 and self.START[0]<self.HEIGHT and self.START[1]>=0 and self.START[1]<self.WIDTH:
            if self.check_obstacle(self.START):
                print("Invalid Start")
                self.running=False
            else:
                print("Valid Start")
        else:
            self.running=False
            print(" Start Out of Bounds")

        if self.GOAL[0]>=0 and self.GOAL[0]<self.HEIGHT and self.GOAL[1]>=0 and self.GOAL[1]<self.WIDTH:
            if self.check_obstacle(self.GOAL):
                print("Invalid")
                self.running=False
            else:
                print("Valid")
        else:
            self.running=False
            print(" Goal Out of Bounds")
        if self.running:
            # Create a blank canvas with size 512x512 and 3 channels (RGB)
            self.img,self.node_grid = self.make_obstacle_space()
            print(len(self.node_grid),len(self.node_grid[0]))
            self.t,self.c= self.dijkstra()
            print(self.c)
            print(self.t[0])
            # img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
            # Show the canvas in a window named "Canvas"
            self.img=self.back_track(self.t,self.c,self.img)
            self.img = cv2.flip(self.img, 0)
            for i in range(5000):
                self.result.write(self.img)
            cv2.imshow("Canvas", self.img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    def back_track(self,tracker,current,img):
        g_node=current
        path=[]
        while not g_node[2]==0:
            for c in tracker:
                if c[2] == g_node[1]:
                    path.append([c[3][0],c[3][1]])
                    g_node=c
        print("Complete")
        path.reverse()
        for x,y in path:
            img[x][y]=np.array(self.track_color)
        return img

    def move(self,current,action):
        i=current[0]+action[0]
        j=current[1]+action[1]
        if i>=0 and j>=0 and i<self.HEIGHT and j<self.WIDTH:
            cost=0
            if action[0]==0 or action[1]==0:
                cost=1
            else:
                cost=1.4
            return self.node_grid[i][j],cost
        else:
            return None,None
    
    
    
    def dijkstra(self):
        start_time = time.time()
        idx=1
        start=(0,0,0,self.START)
#         self.change_color(self.start_color,self.START[0],self.START[1])
        goal=(float('inf'),None,None,self.GOAL)
        
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
            close_list.add(current[3])
            tracker.append(current)
#             if current[3] in close_list:
#                     continue
            if current[3]==goal[3]:
                    break
            
            else:
                for a in actions:
                    neighbor,cost_of_action=self.move(current[3],a)
                    if not neighbor==None:
                        
                        if neighbor[3] not in close_list and not neighbor[0]==-1:
                            if tuple(neighbor) not in open_list.queue or neighbor[0]==float('inf'):
                                self.img[neighbor[3][0]][neighbor[3][1]]=self.explored_color
                                flip_img = cv2.flip(self.img, 0)
                                # cv2.imshow("Out",flip_img)
                                # cv2.waitKey(1)
                                self.result.write(flip_img)
                        
                                neighbor[0]=current[0]+cost_of_action
                                neighbor[1]=current[2]
                                idx=idx+1
                                neighbor[2]=idx
                                # print(neighbor)
                                open_list.put(tuple(neighbor))
                            else:
                                if neighbor[0]>current[0]+cost_of_action:
                                    neighbor[1]=current[2]
                                    neighbor[0]=current[0]+cost_of_action
                        
        end_time = time.time()
        elapsed_time = end_time - start_time
        print(f"Execution time of algorithm: {elapsed_time:.2f} seconds")
        return tracker,current

if __name__ == "__main__":
    # Define start and goal
    # Inverted co ordinate system compensation
    # goal=(230,500)
    start=(6,6)
    goal=(50,50)
    # start=(0,0)
    # Create an instance of Dijkstra
    d_algo = Dijkstra(600,250,1,start,goal)
    
    # Call the game method
    d_algo.game()