
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
import numpy as np
import math



class World:
    def __init__(self):
        self.obstacle_x = 70
        self.obstacle_y  = 90
        self.obstacle_width = 60
        self.obstacle_height = 60
        self.vehicle1_x = 30
        self.vehicle1_y = 10
        self.vehicle2_x = 130
        self.vehicle2_y = 10
        self.robot_x = 0
        self.robot_y = 180
        self.robot_theta = 0
        self.vehicle_width = 30
        self.vehicle_height = 20
        self.padding = 5
        self.robot_start = [self.robot_x + 10, self.robot_y + self.vehicle_height/2,0]
        self.robot_goal = [self.vehicle1_x + self.vehicle_width + 30, 10 + self.vehicle_height/2,0]
        self.robot_wheelRadius = 1
        self.robot_wheelBase = 28
        self.robot_streering_angle = 50
        self.velocity = 1
        self.vel_L = [5,2,1,0,-1,-2,-5]
        self.vel_R = [5,2,1,0,-1,-2,-5]
        self.park_point = False
        self.robot_boundary = [[self.robot_x,self.robot_y,1],[self.robot_x+self.vehicle_width,self.robot_y,1],[self.robot_x+self.vehicle_width,self.robot_y+self.vehicle_height,1],[self.robot_x,self.robot_y+self.vehicle_height,1]]
        self.obstacle = [[self.obstacle_x-self.padding,self.obstacle_y-self.padding],[self.obstacle_x+self.obstacle_width+self.padding,self.obstacle_y-self.padding],[self.obstacle_x+self.obstacle_width+self.padding,self.obstacle_y+self.obstacle_height+self.padding],[self.obstacle_x-self.padding,self.obstacle_y+self.obstacle_height+self.padding]]
        self.vehicle1 = [[self.vehicle1_x-self.padding,self.vehicle1_y-self.padding],[self.vehicle1_x+self.vehicle_width+self.padding,self.vehicle1_y-self.padding],[self.vehicle1_x+self.vehicle_width+self.padding,self.vehicle1_y+self.vehicle_height+self.padding],[self.vehicle1_x-self.padding,self.vehicle1_y+self.vehicle_height+self.padding]]
        self.vehicle2 = [[self.vehicle2_x-self.padding,self.vehicle2_y-self.padding],[self.vehicle2_x+self.vehicle_width+self.padding,self.vehicle2_y-self.padding],[self.vehicle2_x+self.vehicle_width+self.padding,self.vehicle2_y+self.vehicle_height+self.padding],[self.vehicle2_x-self.padding,self.vehicle2_y+self.vehicle_height+self.padding]]
        self.robot_bound = [[-1,29,29,-10],[-10,-10,10,10],[1,1,1,1]]


    def construct_world(self,x,y,theta):
        fig = plt.figure("Animation")
        ax = fig.add_subplot(111)
        obstacle_draw = Rectangle((self.obstacle_x, self.obstacle_y),self.obstacle_width,self.obstacle_height,color= "black")
        vehicle1_draw = Rectangle((self.vehicle1_x,self.vehicle1_y),self.vehicle_width,self.vehicle_height,color='green')
        vehicle2_draw = Rectangle((self.vehicle2_x,self.vehicle2_y),self.vehicle_width,self.vehicle_height,color='green')
        robot_draw = Rectangle((self.robot_x,self.robot_y),self.vehicle_width,self.vehicle_height,color='grey')
        parking_position = Rectangle((self.vehicle1_x+self.vehicle_width+10,5),self.vehicle_width+10,self.vehicle_height+10,fc='none',ec='g',lw=2)
        ax.add_patch(robot_draw)
        ax.add_patch(vehicle1_draw)
        ax.add_patch(vehicle2_draw)
        ax.add_patch(obstacle_draw)
        ax.add_patch(parking_position)
        plt.plot(x,y,"sk")
        boundary = self.boundry_getter(x,y,theta)
        X = []
        Y = []
        for x,y in boundary:
            X.append(x)
            Y.append(y)
        plt.plot(X,Y)
        plt.xlim([0,200])
        plt.ylim([-20,200])
        return

    def boundry_getter(self,x,y,theta):
        theta_new = (theta - self.robot_start[2])*(math.pi/180)
        homo_matrix = [[math.cos(theta_new),-math.sin(theta_new),x],[math.sin(theta_new),math.cos(theta_new),y]]
        matrix = np.dot(homo_matrix,self.robot_bound)
        updated_boundary =  [[matrix[0][0],matrix[1][0]],[matrix[0][1],matrix[1][1]],[matrix[0][2],matrix[1][2]],[matrix[0][3],matrix[1][3]]]
        return updated_boundary

    def neighbour_finder(self,x,y,theta):
        neighbours = []
        for i in range(-self.robot_streering_angle,self.robot_streering_angle+1,5):
            x_dot = self.velocity * math.cos(theta*(math.pi/180))
            y_dot = self.velocity * math.sin(theta*(math.pi/180))
            theta_dot = (self.velocity*math.tan(i*(math.pi/180))/self.robot_wheelBase)*(180/math.pi)
            if(self.valid_point(x+x_dot,y+y_dot,theta+theta_dot)):
                neighbours.append([round(x+x_dot,2),round(y+y_dot,2),(round(theta+theta_dot,2))%360,1,i])
            if(self.valid_point(x-x_dot,y-y_dot,theta-theta_dot)):
                neighbours.append([round(x-x_dot,2),round(y-y_dot,2),(round(theta-theta_dot,2))%360,-1,i])
        return neighbours
    
    def valid_point(self,x,y,theta):
        boundary = self.boundry_getter(x,y,theta)
        if x < 1 or y < self.vehicle_height or x > 200 - self.vehicle_width or y > 200 - self.vehicle_height/2.0:
            return False
        collision_check1 = self.check_polygon_intersection(boundary,self.obstacle)
        collision_check2 = self.check_polygon_intersection(boundary,self.vehicle1)
        collision_check3 = self.check_polygon_intersection(boundary,self.vehicle2)
        if (collision_check1 or collision_check2 or collision_check3):
            return False
        return True
    
    def check_polygon_intersection(self,polygonA, polygonB):
        polygons = [polygonA,polygonB]
        minA, minB, maxA, maxB, projected, i, j, k = None, None, None, None, None, None, None, None
        for i in range(len(polygons)):
            polygon = polygons[i]
            for j in range(len(polygon)):
                vertice_1 = j
                vertice_2 = (vertice_1 + 1) % len(polygon)
                p1 = polygon[vertice_1]
                p2 = polygon[vertice_2]
                normal = {'x':p2[1] - p1[1],'y':p1[0]-p2[0]}
                minA, maxA = None, None 
                for k in range(len(polygonA)):
                    projected = normal['x'] * polygonA[k][0] + normal['y']*polygonA[k][1]
                    if (minA is None) or (projected < minA):
                        minA = projected
                    if (maxA is None) or (projected > maxA):
                        maxA = projected

                minB, maxB = None, None 
                for k in range(len(polygonB)):
                    projected = normal['x'] * polygonB[k][0] + normal['y']*polygonB[k][1]
                    if (minB is None) or (projected < minB):
                        minB = projected
                    if (maxB is None) or (projected > maxB):
                        maxB = projected
                if (maxA < minB) or (maxB < minA):
                    return False
        return True
                
    def cost_function(self,x1,y1,x2,y2):
        distance = math.sqrt((pow(x1-x2,2)+pow(y1-y2,2)))
        return distance
    
    def hurestic_function(self,x,y,theta):
        _theta = 0
        theta = (theta+360)%360
        distance = math.sqrt((pow(self.robot_goal[0]-x,2)+pow(self.robot_goal[1]-y,2)))
        distance += math.sqrt(((pow((self.robot_goal[0]+self.vehicle_width)-(x+self.vehicle_width*math.cos(theta*(math.pi/180))),2)+pow((self.robot_goal[1]+self.vehicle_height)-(y+self.vehicle_height*math.sin(theta*(math.pi/180))),2))))
        if self.straight_available(x,y) and not(x > self.robot_goal[0]-1 and y > self.robot_goal[1]-1 and x <self.robot_goal[0]+1 and y <self.robot_goal[1]+1):
            _theta = abs((360 + (math.atan2(y-self.robot_goal[1],x-self.robot_goal[0]))*(180/math.pi))%360 - theta+180)
        hurestic = distance + _theta 
        return hurestic

    def straight_available(self,x,y):
        boundary_line = [[x,y],[self.robot_goal[0],self.robot_goal[1]],[self.robot_goal[0]+1,self.robot_goal[1]],[x+1,y]]
        if (self.check_polygon_intersection(boundary_line,self.obstacle)) or (self.check_polygon_intersection(boundary_line,self.vehicle1)):
            return False
        return True

    def priority(self,queue):
        min = math.inf
        index = 0
        for i in range(len(queue)):
            _,value,_,_ = queue[i]
            if value < min:
                min = value
                index = i
        return index
    
    def visited_checker(self,curr_node,visited):
        for x,y,theta in visited:
            if curr_node[0] == x and curr_node[1] == y and curr_node[2] == theta:
                return True
        return False

    def A_star(self):
        queue = []
        visited = []
        start = self.robot_start
        f = 0
        g = 0
        path = [start]
        queue.append((start,f,g,path))
        while len(queue) > 0:
            index = self.priority(queue)
            (shortest,_,g_value,path) = queue[index]
            queue.pop(index)
            if not (self.visited_checker((round(shortest[0]),round(shortest[1]),round(shortest[2])),visited)):
                visited.append([round(shortest[0]),round(shortest[1]),round(shortest[2])])
                if round(shortest[0]) <= self.robot_goal[0]+5 and round(shortest[0]) >= self.robot_goal[0]-5 and round(shortest[1]) <= self.robot_goal[1]+5 and round(shortest[1]) >= self.robot_goal[1]-5 and shortest[2] <= self.robot_goal[2]+15 and shortest[2] >= self.robot_goal[2]-15:
                    return path
                neighbours = self.neighbour_finder(shortest[0],shortest[1],shortest[2])
                for neighbour in neighbours:
                    t_g = g_value + (0.1*self.cost_function(shortest[0],shortest[1],neighbour[0],neighbour[1]))
                    t_f = t_g +(0.9*self.hurestic_function(neighbour[0],neighbour[1],neighbour[2]))
                    queue.append((neighbour,t_f,t_g,path + [neighbour]))
        return path

if __name__ == "__main__":
    diff_drive_world = World()
    trajectory = diff_drive_world.A_star()
    print("Reached")
    print(trajectory[-1])

    for point in trajectory:
        plt.cla()
        diff_drive_world.construct_world(point[0],point[1],point[2])
        plt.pause(0.00001)


plt.figure("Axle Path")
plt.xlim([0,200])
plt.ylim([-20,200])
for point in trajectory:
    plt.scatter(point[0],point[1],color="black",s=1)
plt.show()
