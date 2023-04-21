
from matplotlib import get_backend
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
import numpy as np
import math







class World:
    def __init__(self):
        self.obstacle_x = 70
        self.obstacle_y  = 90
        self.obstacle_width = 50
        self.obstacle_height = 50
        self.vehicle1_x = 30
        self.vehicle1_y = 10
        self.robot_x = 49
        self.robot_y = 180
        self.trailer_x = self.robot_x - 23
        self.trailer_y = self.robot_y
        self.robot_theta = 0
        self.trailer_theta = 0
        self.vehicle_width = 22
        self.vehicle_height = 17.5
        self.padding = 2
        self.robot_start = [self.robot_x, self.robot_y + self.vehicle_height/2,0]
        self.trailer_start = [self.trailer_x,self.trailer_y +self.vehicle_height/2,0]
        self.robot_goal = [self.vehicle1_x + 2*self.vehicle_width + 30, 10 + self.vehicle_height/2,180]
        self.robot_wheelRadius = 1
        self.robot_wheelBase = 15
        self.robot_streering_angle = 40
        self.hitching_lenght = 25
        self.velocity = 1
        self.vel_L = [5,2,1,0,-1,-2,-5]
        self.vel_R = [5,2,1,0,-1,-2,-5]
        self.park_point = False
        self.robot_boundary = [[self.robot_x,self.robot_y,1],[self.robot_x+self.vehicle_width,self.robot_y,1],[self.robot_x+self.vehicle_width,self.robot_y+self.vehicle_height,1],[self.robot_x,self.robot_y+self.vehicle_height,1]]
        self.obstacle = [[self.obstacle_x-self.padding,self.obstacle_y-self.padding],[self.obstacle_x+self.obstacle_width+self.padding,self.obstacle_y-self.padding],[self.obstacle_x+self.obstacle_width+self.padding,self.obstacle_y+self.obstacle_height+self.padding],[self.obstacle_x-self.padding,self.obstacle_y+self.obstacle_height+self.padding]]
        self.vehicle1 = [[self.vehicle1_x-self.padding,self.vehicle1_y-self.padding],[self.vehicle1_x+self.vehicle_width+self.padding,self.vehicle1_y-self.padding],[self.vehicle1_x+self.vehicle_width+self.padding,self.vehicle1_y+self.vehicle_height+self.padding],[self.vehicle1_x-self.padding,self.vehicle1_y+self.vehicle_height+self.padding]]
        self.robot_bound = [[-1,21,21,-1],[-7.5,-7.5,7.5,7.5],[1,1,1,1]]
        self.trailer_bound = [[-1,11,11,-1],[-7.5,-7.5,7.5,7.5],[1,1,1,1]]


    def construct_world(self,x,y,theta,xt,yt,theta_t):
        fig = plt.figure("Animation")
        ax = fig.add_subplot(111)
        obstacle_draw = Rectangle((self.obstacle_x, self.obstacle_y),self.obstacle_width,self.obstacle_height,color= "black")
        vehicle1_draw = Rectangle((self.vehicle1_x,self.vehicle1_y),self.vehicle_width,self.vehicle_height,color='red')
        robot_draw = Rectangle((self.robot_x,self.robot_y),self.vehicle_width,self.vehicle_height,color='green')
        trailer_draw = Rectangle((self.trailer_x,self.trailer_y),10,self.vehicle_height,color='green')
        parking_position = Rectangle((self.vehicle1_x+self.vehicle_width+15,0),(2*self.vehicle_width)+30,(self.vehicle_height*2),fc='none',ec='g',lw=2)
        ax.add_patch(robot_draw)
        ax.add_patch(trailer_draw)
        ax.add_patch(vehicle1_draw)
        ax.add_patch(obstacle_draw)
        ax.add_patch(parking_position)
        plt.plot(x,y,"sk")
        boundary = self.boundry_get(x,y,theta)
        boundary_t = self.boundry_get(xt,yt,theta_t,'trailer')
        X = []
        Y = []
        for x,y in boundary:
            X.append(x)
            Y.append(y)
        plt.plot(X,Y,color='green')
        X = []
        Y = []
        for x,y in boundary_t:
            X.append(x)
            Y.append(y)
        plt.plot(X,Y,color='green')
        plt.xlim([0,200])
        plt.ylim([-20,200])
        return

    def boundry_get(self,x,y,theta,vehicle_type='car'):
        theta_new = (theta - self.robot_start[2])*(math.pi/180)
        homo_matrix = [[math.cos(theta_new),-math.sin(theta_new),x],[math.sin(theta_new),math.cos(theta_new),y]]
        if vehicle_type == 'car':
            matrix = np.dot(homo_matrix,self.robot_bound)
        else:
            matrix = np.dot(homo_matrix,self.trailer_bound)
        updated_boundary =  [[matrix[0][0],matrix[1][0]],[matrix[0][1],matrix[1][1]],[matrix[0][2],matrix[1][2]],[matrix[0][3],matrix[1][3]]]
        return updated_boundary

    def neighbours_getter(self,x,y,theta,xt,yt,theta_t):
        neighbours = []
        for i in [-self.robot_streering_angle,0,self.robot_streering_angle] :
            x_dot = self.velocity * math.cos(theta*(math.pi/180))
            y_dot = self.velocity * math.sin(theta*(math.pi/180))
            theta_dot = (self.velocity*math.tan(i*(math.pi/180))/self.robot_wheelBase)*(180/math.pi)
            xt_dot = self.velocity * math.cos(theta_t*(math.pi/180))
            yt_dot = self.velocity * math.sin(theta_t*(math.pi/180))
            theta_t_dot = (self.velocity*math.sin((theta-theta_t)*(math.pi/180))/self.hitching_lenght)*(180/math.pi)
            if(self.valid_point(x+x_dot,y+y_dot,theta+theta_dot,xt+xt_dot,yt+yt_dot,theta_t+theta_t_dot)):
                neighbours.append([round(x+x_dot,2),round(y+y_dot,2),(round(theta+theta_dot,2))%360,round(xt+xt_dot,2),round(yt+yt_dot,2),(round(theta_t+theta_t_dot,2)+360)%360,1,i])
            if(self.valid_point(x-x_dot,y-y_dot,theta-theta_dot,xt-xt_dot,yt-yt_dot,theta_t-theta_t_dot)):
                neighbours.append([round(x-x_dot,2),round(y-y_dot,2),(round(theta-theta_dot,2))%360,round(xt-xt_dot,2),round(yt-yt_dot,2),(round(theta_t-theta_t_dot,2)+360)%360,-1,i])
        return neighbours
    
    def valid_point(self,x,y,theta,xt,yt,theta_t):
        boundary = self.boundry_get(x,y,theta)
        boundary_t = self.boundry_get(xt,yt,theta_t,"trailer")
        if x < 1 or y < self.vehicle_height or x > 200 - self.vehicle_width or y > 200 - self.vehicle_height/2.0:
            return False
        collision_check1 = self.check_polygon_intersection(boundary,self.obstacle)
        collision_check2 = self.check_polygon_intersection(boundary,self.vehicle1)
        collision_check3 = self.check_polygon_intersection(boundary_t,self.vehicle1)
        collision_check4 = self.check_polygon_intersection(boundary_t,self.obstacle)
        if (collision_check1 or collision_check2 or collision_check3 or collision_check4):
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
                
    def cost_cal(self,x1,y1,x2,y2):
        distance = math.sqrt((pow(x1-x2,2)+pow(y1-y2,2)))
        return distance
    
    def hurestic_function(self,x,y,theta,xt,yt,theta_t,vel):
        _theta = 0
        theta = (theta+360)%360
        theta_t = (theta_t+360)%360
        if theta_t == 0:
            theta_t= 360
        reverse_penalty = 0
        turn_penalty = 0
        obstacle_penalty = 0
        distance = math.sqrt((pow(self.robot_goal[0]-x,2)+pow(self.robot_goal[1]-y,2)))
        distance += math.sqrt(((pow((self.robot_goal[0]-self.vehicle_width)-(x+self.vehicle_width*math.cos(theta*(math.pi/180))),2)+pow((self.robot_goal[1]+self.vehicle_height)-(y+self.vehicle_height*math.sin(theta*(math.pi/180))),2))))
        if self.straight_available(x,y,xt,yt) and not(x > self.robot_goal[0]-5 and y > self.robot_goal[1]-5 and x <self.robot_goal[0]+5 and y <self.robot_goal[1]+5):
            _theta = abs((360 + (math.atan2(y-self.robot_goal[1],x-self.robot_goal[0]))*(180/math.pi))%360 - theta+180)
        else:
            _theta = 180
        if self.check_polygon_intersection([[x-15,y],[x+200*math.cos(theta*(math.pi/180))-15,y+200*math.sin(theta*(math.pi/180))],[15+x+200*math.cos(theta*(math.pi/180)),y+200*math.sin(theta*(math.pi/180))],[x+15,y]],self.obstacle):
            obstacle_penalty +=10
        if self.check_polygon_intersection([[x-15,y],[x+200*math.cos(theta*(math.pi/180))-15,y+200*math.sin(theta*(math.pi/180))],[15+x+200*math.cos(theta*(math.pi/180)),y+200*math.sin(theta*(math.pi/180))],[x+15,y]],self.obstacle):
            obstacle_penalty +=10
        if vel < 0:
            reverse_penalty = 1
        if abs(theta-theta_t)>15 and not(x > self.robot_goal[0]-5 and y > self.robot_goal[1]-5 and x <self.robot_goal[0]+5 and y <self.robot_goal[1]+5):
            turn_penalty = 5
        hurestic = distance + _theta + reverse_penalty + turn_penalty + obstacle_penalty 
        return hurestic

    def straight_available(self,x,y,xt,yt):
        boundary_line = [[x,y],[self.robot_goal[0],self.robot_goal[1]],[self.robot_goal[0]+1,self.robot_goal[1]],[x+1,y]]
        boundary_line_t = [[xt,yt],[self.robot_goal[0]+50,self.robot_goal[1]],[self.robot_goal[0]+1+50,self.robot_goal[1]],[xt+1,yt]]
        if (self.check_polygon_intersection(boundary_line,self.obstacle)) or (self.check_polygon_intersection(boundary_line,self.vehicle1)) or (self.check_polygon_intersection(boundary_line_t,self.obstacle)) or (self.check_polygon_intersection(boundary_line_t,self.vehicle1)):
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
        for x,y,theta,xt,yt,th_t in visited:
            if curr_node[0] == x and curr_node[1] == y and curr_node[2] == theta and curr_node[3]==xt and curr_node[4]==yt and curr_node[5]== th_t:
                return True
        return False

    def A_star(self):
        queue = []
        visited = []
        start = [self.robot_start[0],self.robot_start[1],self.robot_start[2],self.trailer_start[0],self.trailer_start[1],self.trailer_start[2]]
        f = 0
        g = 0
        path = [start]
        queue.append((start,f,g,path))
        while len(queue) > 0:
            index = self.priority(queue)
            (shortest,_,g_value,path) = queue[index]
            queue.pop(index)
            if not (self.visited_checker([round(shortest[0]),round(shortest[1]),round(shortest[2]),round(shortest[3]),round(shortest[4]),round(shortest[5])],visited)):
                visited.append([round(shortest[0]),round(shortest[1]),round(shortest[2]),round(shortest[3]),round(shortest[4]),round(shortest[5])])
                if round(shortest[0]) <= self.robot_goal[0]+5 and round(shortest[0]) >= self.robot_goal[0]-5 and round(shortest[1]) <= self.robot_goal[1]+5 and round(shortest[1]) >= self.robot_goal[1]-5 and shortest[2] <= self.robot_goal[2]+15 and shortest[2] >= self.robot_goal[2]-15:
                    return path
                neighbours = self.neighbours_getter(shortest[0],shortest[1],shortest[2],shortest[3],shortest[4],shortest[5])
                for neighbour in neighbours:
                    print(neighbour)
                    vel = neighbour[6]
                    turn = neighbour[7]
                    t_g = g_value + (0.1*self.cost_cal(shortest[0],shortest[1],neighbour[0],neighbour[1]))
                    t_f = t_g +(0.9*self.hurestic_function(neighbour[0],neighbour[1],neighbour[2],neighbour[3],neighbour[4],neighbour[5],vel))
                    queue.append((neighbour,t_f,t_g,path + [neighbour]))
        return path

if __name__ == "__main__":
    diff_drive_world = World()
    trajectory = diff_drive_world.A_star()
    print("Reached")
    print(trajectory[-1])

    for point in trajectory:
        plt.cla()
        diff_drive_world.construct_world(point[0],point[1],point[2],point[3],point[4],point[5])
        plt.pause(0.00001)


plt.figure("Axle Path")
plt.xlim([0,200])
plt.ylim([-20,200])
for point in trajectory:
    plt.scatter(point[0],point[1],color="black",s=1)
plt.show()
