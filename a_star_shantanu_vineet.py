# Project 3 
# Submission By: Shantanu Parab (sparab), Vineet Singh (vsingh03)

# GITHUB Link: https://github.com/shantanuparabumd/ASTAR.git

# Imports
import math
import numpy as np
import time
from queue import PriorityQueue
import cv2
import argparse

# Creating Node Class
class Astar:
    def __init__(self,width,height,scale,start,goal,robot_clear,obj_clear,step_size):
        
        # Get the video dimensions and FPS
        self.result = cv2.VideoWriter("Astar01.avi", cv2.VideoWriter_fourcc(*'MJPG'), 600, (width, height))
        
        # Define constants
        self.START=start
        self.GOAL=goal
        self.SCALE=scale
        self.WIDTH=width
        self.HEIGHT=height
        self.L=step_size
        self.frame_info=[]
        self.ROBOT=robot_clear
        self.OBJ=obj_clear

        # Define Colors
        self.background=(255,255,255)
        self.obst_color=(0,0,0)
        self.clear_color= (12, 253, 240)
        self.grid_color=(233, 226, 230, 0.2)
        self.start_color=(132, 222, 15, 0.8)
        self.goal_color=(254, 0, 0, 0.8)
        self.explored_color= (64, 188, 237)
        self.track_color= (16, 141, 16)
        self.robot_color= (62, 169, 119)
        self.robot_clear_color= (255, 255, 255)

        # Define Grid Size
        self.grid_size=2

        self.H_PX=self.WIDTH*self.SCALE/self.grid_size
        self.V_PX=self.HEIGHT*self.SCALE/self.grid_size

        self.HEX=self.hexagon(125,300,50)
        self.HEX_CLR=self.hexagon(125,300,50+(self.OBJ))
        self.HEX_ROBO_CLR=self.hexagon(125,300,50+(self.OBJ+self.ROBOT))
        # Triangle
        p1=(25-self.OBJ, 460-self.OBJ)
        p2=(25-self.OBJ, 460+self.OBJ)
        p3=(125,510+self.OBJ)
        p4=(225+self.OBJ, 460+self.OBJ)
        p5=(225+self.OBJ,460-self.OBJ)
        self.TRI_CLR = [p1, p2, p3, p4, p5]
        
        p1=(25-(self.OBJ+self.ROBOT), 460-(self.OBJ+self.ROBOT))
        p2=(25-(self.OBJ+self.ROBOT), 460+(self.OBJ+self.ROBOT))
        p3=(125,510+(self.OBJ+self.ROBOT))
        p4=(225+(self.OBJ+self.ROBOT), 460+(self.OBJ+self.ROBOT))
        p5=(225+(self.OBJ+self.ROBOT),460-(self.OBJ+self.ROBOT))
        self.TRI_ROBO_CLR = [p1, p2, p3, p4, p5]
        
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
        h1=self.obstacle_detect_rectangle(p,0,100-self.OBJ,100+self.OBJ,50+2*self.OBJ)
        h2=self.obstacle_detect_rectangle(p,150-self.OBJ,100-self.OBJ,100+self.OBJ,50+2*self.OBJ)
        h3=self.obstacle_detect_polygon(p,self.HEX_CLR)
        h4=self.obstacle_detect_polygon(p,self.TRI_CLR)
        h5= self.obstacle_detect_boundary(p,self.OBJ,self.OBJ,250-2*self.OBJ,600-2*self.OBJ)
        
        return (h1 or h2 or h3 or h4 or h5)
    
    def check_robot(self,p):
        h1=self.obstacle_detect_rectangle(p,0,100-(self.OBJ+self.ROBOT),100+(self.OBJ+self.ROBOT),50+2*(self.OBJ+self.ROBOT))
        h2=self.obstacle_detect_rectangle(p,150-(self.OBJ+self.ROBOT),100-(self.OBJ+self.ROBOT),100+(self.OBJ+self.ROBOT),50+2*(self.OBJ+self.ROBOT))
        h3=self.obstacle_detect_polygon(p,self.HEX_ROBO_CLR)
        h4=self.obstacle_detect_polygon(p,self.TRI_ROBO_CLR)
        h5= self.obstacle_detect_boundary(p,(self.OBJ+self.ROBOT),(self.OBJ+self.ROBOT),250-2*(self.OBJ+self.ROBOT),600-2*(self.OBJ+self.ROBOT))
        
        return (h1 or h2 or h3 or h4 or h5)

    def make_obstacle_space(self):
        grid=np.ones((self.HEIGHT,self.WIDTH,3),np.uint8)
        nodes=[]
        for i in range(self.HEIGHT*2):
            row=[]
            for j in  range(self.WIDTH*2):
                theta=[]
                for t in range(12):
                    if self.check_robot((i/2,j/2)):
                        theta.append([-1,None,None,None,(i/2,j/2,t),None])
                        grid[int(np.floor(i/2))][int(np.floor(j/2))]=np.array(self.robot_clear_color)
                        if self.check_clearance((i/2,j/2)):
                            grid[int(np.floor(i/2))][int(np.floor(j/2))]=np.array(self.clear_color)
                            if self.check_obstacle((i/2,j/2)):
                                grid[int(np.floor(i/2))][int(np.floor(j/2))]=np.array(self.obst_color)
                    else:
                        theta.append([float('inf'),float('inf'),None,None,(i/2,j/2,t),float('inf')])
                        grid[int(np.floor(i/2))][int(np.floor(j/2))]=np.array(self.background)
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
                print("Invalid Goal")
                self.running=False
            else:
                print("Valid Goal")
        else:
            self.running=False
            print(" Goal Out of Bounds")
        if self.running:
            start_time = time.time()
            self.img,self.node_grid = self.make_obstacle_space()
            end_time = time.time()
            elapsed_time = end_time - start_time
            print(f"\nGrid formation Time: {elapsed_time:.2f} seconds")
            self.t,self.c= self.astar()
            self.video()
            self.img=self.back_track(self.t,self.c,self.img)
            for i in range(3000):
                flip_img = cv2.flip(self.img, 0)
                self.result.write(flip_img)
            flip_img = cv2.flip(self.img, 0)
            cv2.imshow("Out", flip_img)
            cv2.waitKey(10)
            cv2.destroyAllWindows()

    def back_track(self,tracker,current,img):
        g_node=current
        path=[]
        while not g_node[3]==0:
            for c in tracker:
                if c[3] == g_node[2]:
                    path.append([c[4][1],c[4][0]])
                    g_node=c
        print("Complete")
        path.reverse()
        for i in range(len(path)-1):
            x1,y1=path[i]
            x2,y2=path[i+1]
            print(path[i],path[i+1])
            # draw the line on the image
            img=cv2.line(img, (int(x2),int(y2)), (int(x1),int(y1)), self.track_color, 4)
        return img
    
    def roundn(self,num):
        if num>=np.floor(num)+0.5:
            rounded_num = np.ceil(num) if 0.75 < (num%1) else num-(num%0.5)
        else:
            rounded_num = np.floor(num) if  0.25 < (num%1) else num-(num%0.5)
        return  rounded_num
    
    def action_set(self,current,action):
        xg=action[1]*np.cos((action[0]+current[2])*np.pi/6)
        yg=action[1]*np.sin((action[0]+current[2])*np.pi/6)
        xg=current[1]+xg
        yg=current[0]+yg
        xg=self.roundn(xg)
        yg=self.roundn(yg)
        if xg>=0 and yg>=0 and xg<self.WIDTH and yg<self.HEIGHT:
                cost=action[1]
                k=(action[0]+current[2])%12
                return self.node_grid[int(yg*2)][int(xg*2)][int(k)],cost
        else:
            return None,None
    
    def draw_vector(self,current,neighbor):
        # define the start and end points of the line
        start_point = (int(np.floor(current[4][1])),int(np.floor(current[4][0])))
        end_point = (int(np.floor(neighbor[4][1])),int(np.floor(neighbor[4][0])))

        # draw the line on the image
        self.img=cv2.line(self.img, start_point, end_point, self.robot_color, 1)
        
    def cost_to_goal(self,x,y):
        return math.sqrt((self.GOAL[0]-x)**2 + (self.GOAL[1]-y)**2)
    
    def check_goal(self, current):
        # Calculate the distance between the point and the center of the circle using the Pythagorean theorem
        distance = ((current[0] - self.GOAL[0]) ** 2 + (current[1] - self.GOAL[1]) ** 2) ** 0.5
        # If the distance is less than or equal to the radius of the circle, the point lies within the circle
        if distance <= 1.5:
            return True
        else:
            return False

    def check_dup(self,current, open_list):
        for _,_,_,_,node,_ in open_list:
            if ((current[0] - node[0]) ** 2 + (current[1] - node[1]) ** 2)<=0.5:
                return True
        return False
    
    def video(self):
        for current,neighbor in self.frame_info:
            self.draw_vector(current,neighbor)
            flip_img = cv2.flip(self.img, 0)
            self.result.write(flip_img)
                 
    def astar(self):
        start_time = time.time()
        idx=1
        start_cost_to_goal=math.sqrt((self.GOAL[0]-self.START[0])**2 + (self.GOAL[1]-self.START[1])**2)
        start=(start_cost_to_goal,start_cost_to_goal,0,0,self.START,0)
        goal=(float('inf'),0,None,None,self.GOAL,None)
        
         # Define action sets
        self.L = 10
        actions = [[k, self.L] for k in range(-2,3)]
        
        # Create an open list
        open_list = PriorityQueue()
        # Create a close list
        close_list=[]
        # Create a tracker for path node with compatible data type
        tracker=[]
        current=start
        open_list.put(start)
        while open_list and not self.check_goal(current[4]):
            current=open_list.get()
            close_list.append(current[4])
            tracker.append(current)
            if self.check_goal(current[4]):
                    break
            else:
                for a in actions:
                    neighbor,cost_of_action=self.action_set(current[4],a)
                    if not neighbor==None:
                        
                        if neighbor[4] not in close_list and not neighbor[0]==-1:
                            if  neighbor[0]==float('inf'):
                                self.frame_info.append([current,neighbor])
                                # self.draw_vector(current,neighbor)
                                # flip_img = cv2.flip(self.img, 0)
                                # # cv2.imshow("Out",flip_img)
                                # # cv2.waitKey(1)
                                # # if cv2.waitKey(20) & 0xFF == ord('q'):
                                # #     break
                                # self.result.write(flip_img)
#                                 Parent
                                neighbor[2]=current[3]
#                                 Cost to Come
                                neighbor[5]=current[5]+cost_of_action
#                                 Cost
                                neighbor[0]=neighbor[5]+self.cost_to_goal(neighbor[4][0],neighbor[4][1])
                                neighbor[1]=self.cost_to_goal(neighbor[4][0],neighbor[4][1])
                                idx=idx+1
                                neighbor[3]=idx
                                open_list.put(tuple(neighbor))
                            else:
                                if neighbor[0]>current[5]+cost_of_action:
                                    neighbor[2]=current[3]
                                    neighbor[5]=current[5]+cost_of_action
                                    neighbor[0]=neighbor[5]+self.cost_to_goal(neighbor[4][0],neighbor[4][1])
                                    
        tracker.append(current)          
        end_time = time.time()
        elapsed_time = end_time - start_time
        print(f"\nExecution time of algorithm: {elapsed_time:.2f} seconds")
        return tracker,current
    
if __name__ == "__main__":
    
    # Parameters to accept start and goal
    parser = argparse.ArgumentParser()
    parser.add_argument("--InitState",nargs='+', type=int, help = 'Initial state for the matrix')
    parser.add_argument("--GoalState",nargs='+', type=int, help = 'Goal state for the matrix')
    parser.add_argument("--StepSize",nargs='+', type=int, help = 'Step size for the robot')
    parser.add_argument("--RobotClearance",nargs='+', type=int, help = 'Clearance for the robot')
    parser.add_argument("--ObjectClearance",nargs='+', type=int, help = 'Clearance for the object')
    Args = parser.parse_args()
    initial_point = Args.InitState
    goal_point = Args.GoalState
    step_size = Args.StepSize
    robot_clear = Args.RobotClearance
    object_clear = Args.RobotClearance

    # Converting initial and final goals to multiple of 30 
    init_deg = int(initial_point[2]/30)
    goal_deg = int(goal_point[2]/30)
    # Converting inputs from list to tuple and integer 
    start=(initial_point[1], initial_point[0],init_deg)
    goal=(goal_point[1], goal_point[0],goal_deg)
    robot_clear = robot_clear[0]
    object_clear = object_clear[0]
    step_size = step_size[0]

    #Creating an instance of A*
    d_algo = Astar(600,250,1,start,goal,robot_clear,object_clear,step_size)

    # Call the game method
    d_algo.game()