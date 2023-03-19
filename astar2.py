# Imports
import math
import numpy as np
import time
from queue import PriorityQueue
import cv2

class Dijkstra:
    def __init__(self,width,height,scale,start,goal):
        # Get the video dimensions and FPS
        
        self.result = cv2.VideoWriter("Dijkstra01.avi", cv2.VideoWriter_fourcc(*'MJPG'), 600, (width, height))
        
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
        self.clear_color= (145, 194, 245)
        self.grid_color=(233, 226, 230, 0.2)
        self.start_color=(132, 222, 15, 0.8)
        self.goal_color=(254, 0, 0, 0.8)
        self.explored_color= (64, 188, 237)
        self.track_color= (64, 64, 237)
        self.robot_color= (168, 230, 53)

        # Define Grid Size
        self.grid_size=2

        self.H_PX=self.WIDTH*self.SCALE/self.grid_size
        self.V_PX=self.HEIGHT*self.SCALE/self.grid_size

        self.HEX=self.hexagon(125,300,50)
        self.HEX_CLR=self.hexagon(125,300,60)
        # Triangle
        p1=((250-20),455)
        p2=(20, 455)
        p3=((250-20),465)
        p4=(20, 465)
        p5=(125,515)
        self.TRI_CLR = [p1, p2, p4, p5,p3]
        p1=(25,460)
        p2=(225, 460)
        p3=(125,510)
        self.TRI=[p1,p2,p3]
    
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

            
    def check_obstacle(self,p):
        h1=self.obstacle_detect_rectangle(p,0,100,100,50)
        h2=self.obstacle_detect_rectangle(p,150,100,100,50)
        h3=self.obstacle_detect_polygon(p,self.HEX)
        h4=self.obstacle_detect_polygon(p,self.TRI)

        return (h1 or h2 or h3 or h4 )

    
    def check_clearance(self,p):
        h1=self.obstacle_detect_rectangle(p,0,95,105,60)
        h2=self.obstacle_detect_rectangle(p,145,95,105,60)
        h3=self.obstacle_detect_polygon(p,self.HEX_CLR)
        h4=self.obstacle_detect_polygon(p,self.TRI_CLR)
        h5= self.obstacle_detect_boundary(p,5,5,240,590)
        
        return (h1 or h2 or h3 or h4 or h5)

    def make_obstacle_space(self):
        grid=np.ones((self.HEIGHT,self.WIDTH,3),np.uint8)
        nodes=[]
        flag=0
        for i in range(self.HEIGHT):
            row=[]
            for j in  range(self.WIDTH):
                theta=[]
                for t in range(12):
                    if self.check_clearance((i,j)):
                        theta.append([-1,None,None,(i,j,t),None])
                        grid[i][j]=np.array(self.clear_color)
                        if self.check_obstacle((i,j)):
                            grid[i][j]=np.array(self.obst_color)
                    else:
                        theta.append([float('inf'),None,None,(i,j,t),float('inf')])
                        grid[i][j]=np.array(self.background)
                row.append(theta)
            nodes.append(row)
        return grid,nodes

    def game(self):
        self.running=True
        if self.START[0]>=0 and self.START[0]<self.HEIGHT and self.START[1]>=0 and self.START[1]<self.WIDTH:
            if self.check_clearance(self.START):
                print("Invalid Start")
                self.running=False
            else:
                print("Valid Start")
        else:
            self.running=False
            print(" Start Out of Bounds")

        if self.GOAL[0]>=0 and self.GOAL[0]<self.HEIGHT and self.GOAL[1]>=0 and self.GOAL[1]<self.WIDTH:
            if self.check_clearance(self.GOAL):
                print("Invalid")
                self.running=False
            else:
                print("Valid")
        else:
            self.running=False
            print(" Goal Out of Bounds")
        if self.running:
            self.img,self.node_grid = self.make_obstacle_space()
            self.t,self.c= self.astar()
            
            self.img=self.back_track(self.t,self.c,self.img)
            self.img = cv2.flip(self.img, 0)
            for i in range(3000):
                self.result.write(self.img)
            cv2.imshow("Final Output", self.img)
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
        for i in range(len(path)-1):
            x1,y1=path[i]
            x2,y2=path[i+1]
            print(path[i],path[i+1])
            # draw the line on the image
            self.img=cv2.line(self.img, (x2,y2), (x1,y1), (0, 0, 255), 1)
        return img
    
    def action_set(self,current,action):
        print(current)
        xg=action[1]*np.cos((action[0]+current[2])*np.pi/6)
        yg=action[1]*np.sin((action[0]+current[2])*np.pi/6)
        xg=current[1]+xg
        yg=current[0]+yg
#         print(xg,yg,)
        if xg>=0 and yg>=0 and xg<self.WIDTH and yg<self.HEIGHT:
                cost=action[1]
                k=(action[0]+current[2])%12
                print(xg,yg,k)
                return self.node_grid[int(yg)][int(xg)][int(k)],cost
        else:
            return None,None
    
    def draw_vector(self,current,neighbor):
        # define the start and end points of the line
        start_point = (current[3][1],current[3][0])
        end_point = (neighbor[3][1],neighbor[3][0])

        # draw the line on the image
        self.img=cv2.line(self.img, start_point, end_point, (0, 255, 0), 1)
        
    def cost_to_goal(self,x,y):
        return math.sqrt((self.GOAL[0]-x)**2 + (self.GOAL[1]-y)**2)
    
    def check_goal(self, current):
        # Calculate the distance between the point and the center of the circle using the Pythagorean theorem
        distance = ((current[0] - self.GOAL[0]) ** 2 + (current[1] - self.GOAL[1]) ** 2) ** 0.5
        # If the distance is less than or equal to the radius of the circle, the point lies within the circle
#         print(current,distance)
        if distance <= 1.5:
            return True
        else:
            return False
    def check_dup(self,current, open_list):
        for _,_,_,node,_ in open_list:
            if ((current[0] - node[0]) ** 2 + (current[1] - node[1]) ** 2)<0.25:
                return True
        return False

    def astar(self):
        print(len(self.node_grid),len(self.node_grid[0]),len(self.node_grid[0][0]))
        start_time = time.time()
        idx=1
        start=(0,0,0,self.START,0)
#         self.change_color(self.start_color,self.START[0],self.START[1])
        goal=(float('inf'),None,None,self.GOAL,None)
        
         # Define action sets
#         n = 12
        L = 10
        actions = [[k, L] for k in range(-2,3)]
        
        # Create a open list
        open_list = PriorityQueue()
        # Create a close list
        close_list=[]
        # Create a tracker for path node with compatible data type
        tracker=[]
        current=start
        open_list.put(start)
#         [Cost,Idx,PrIdx,State,CostToCome]
#         theta.append([float('inf'),None,None,(i,j,t),float('inf')])
        while open_list and not self.check_goal(current[3]):
#             print('.',end='')
            current=open_list.get()
            print("Current",current)
            close_list.append(current)
            tracker.append(current)
#             if current[3] in close_list:
#                     continue
            if self.check_goal(current[3]):
                    break
            
            else:
                for a in actions:
                    neighbor,cost_of_action=self.action_set(current[3],a)
#                     print(neighbor)
                    if not neighbor==None:
                        
                        if not self.check_dup(neighbor[3],close_list) and not neighbor[0]==-1:
                            if not self.check_dup(neighbor[3],open_list.queue) or neighbor[0]==float('inf'):
                                self.draw_vector(current,neighbor)
                                flip_img = cv2.flip(self.img, 0)
                                cv2.imshow("Out",flip_img)
                                cv2.waitKey(1)
                                if cv2.waitKey(20) & 0xFF == ord('q'):
                                    break
#                                 self.result.write(flip_img)
#         [Cost,CostToGoal,PrIdx,Idx,State,CostToCome]
#                                 Parent
                                neighbor[1]=current[2]
#                                 Cost to Come
                                neighbor[4]=current[4]+cost_of_action
#                                 Cost
                                neighbor[0]=neighbor[4]+self.cost_to_goal(neighbor[3][0],neighbor[3][1])
                                
                                idx=idx+1
                                neighbor[2]=idx
                                print("Neigbor",neighbor)
                                open_list.put(tuple(neighbor))
                            else:
                                if neighbor[4]>current[4]+cost_of_action:
                                    neighbor[1]=current[2]
                                    neighbor[0]=neighbor[4]+self.cost_to_goal(neighbor[3][0],neighbor[3][1])
        tracker.append(current)     
        end_time = time.time()
        elapsed_time = end_time - start_time
        print(f"\nExecution time of algorithm: {elapsed_time:.2f} seconds")
        return tracker,current
    

if __name__ == "__main__":
    # Define start and goal
    # Inverted co ordinate system compensation
    
    # goal=(230,500)
#     sx=int(input("Enter X co ordinate of start point: "))
#     sy=int(input("Enter Y co ordinate of start point: "))
#     st=int(input("Enter Theta of start point: "))
#     start=(sy,sx,st)
#     gx=int(input("Enter X co ordinate of goal point: "))
#     gy=int(input("Enter Y co ordinate of goal point: "))
#     gt=int(input("Enter theta of goal point: "))
#     goal=(gy,gx,gt)
    start=(6,6,0)
#     goal=(240,550,0)
    goal=(30,30,0)
    # start=(0,0)
    # Create an instance of Dijkstra
    d_algo = Dijkstra(600,250,1,start,goal)
    
    # Call the game method
    d_algo.game()