#!/usr/bin/env python3

# Turtlebot code to follow a pre-defined global trajectory #
# v1 - Prop control of angular velocity to return to trajectory #

import time
import rospy
from math import sqrt,cos,sin,atan2
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu	
from nav_msgs.msg import Odometry
import sys, select, termios, tty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random
import numpy as np

global yaw, cur_loc

#### SET THIS MODE TO FALSE FOR CONSTANT SPEEP/DIRECTION ####
#### SET THIS MODE TO TRUE FOR RANDOM WAYPOINT EVASION   ####
random_evader = False
#############################################################

time_prev = time.time()
imu_data = Imu()
yaw = 0.0
twist = Twist()
odom = Odometry()

### Declaring GLOBAL Variables for Heading and Speed Control ###
yaw_error_prev = 0.0
yaw_error_sum = 0.0
spd_error_prev = 0.0
spd_error_sum = 0.0

dt = 0.1

class Node():
    def __init__(self, x,y, cost, index):
        self.x = x
        self.y = y
        self.cost = cost
        self.index = index

class Turtle():
    def __init__(self, x, y, step_size):
        self.position  = [x,y]
        self.move_list = [[step_size,0], #move right
                          [-step_size,0], #move left 
                          [0,step_size], #move up
                          [0,-step_size],#move down
                          [-step_size,-step_size], #move southwest
                          [step_size,-step_size],#move southeast
                          [step_size,step_size],#move northeast
                          [-step_size,step_size]#move northwest
                          ]
        
        self.visited_history = {}
        self.not_visited = {} 
        self.obstacle_location = {}
        
   
            
class ConfigSpace():
    
   # sets up a configuration space based on the following inputs:
   # x_bounds = [x_min,x_max]
   # y_bounds = [y_min,y_max]
   # spacing = grid spacing or step size in between values
    
    def __init__(self, x_bounds, y_bounds, spacing):
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.spacing = spacing
        
    def set_obstacles(self, obstacle_list):
        self.obstacles = obstacle_list
            
    def set_graph_coords(self):
        """graph coordinates and define the search space"""
        self.x_coords = np.arange(self.x_bounds[0], self.x_bounds[1]+self.spacing,
                                  self.spacing)
        
        self.y_coords = np.arange(self.y_bounds[0], self.y_bounds[1]+self.spacing,
                                  self.spacing)
        
        self.generate_search_space()
        
    def get_x_y_coords(self):
        return self.x_coords, self.y_coords
    
    def generate_search_space(self):
        """generate our search space"""
        self.search_space = np.zeros((len(self.x_coords),len(self.y_coords))) 
    
     
    def place_obstacles(self, obst_list):
        """places obstacles in grid by inserting a 1""" 
        for obstacle in obst_list:
            obs_x = obstacle[0]
            obs_y = obstacle[1]
            self.search_space[obs_x, obs_y]= 1
    
    def calc_index(self,position):
        """calculate index """
        index = (position[1] - self.y_bounds[0]) / \
            self.spacing * (self.x_bounds[1] - self.x_bounds[0] + self.spacing)/ \
                self.spacing + (position[0] - self.x_bounds[0]) / self.spacing
                
        return index       
    
#    def calc_index(self, position_x, position_y):
#        """calculate index """
#        index = (position_y - self.y_bounds[0]) / \
#            self.spacing * (self.x_bounds[1] - self.x_bounds[0] + self.spacing)/ \
#                self.spacing + (position_x - self.x_bounds[0]) / self.spacing
#                
#        return index            
    
def check_within_obstacle(obstacle_list, current_position, obstacle_radius):
    """check if I am within collision of obstacle return True if it is
    false if I'm not"""
    for obstacle in obstacle_list:
        distance = compute_distance(current_position, obstacle)
        
        if distance<=obstacle_radius:
            return True
        else:    
            return False

def check_if_obstacle_is_present(obstacle_list, node_in_question):
    """check to see if an obstacle is in the way"""
    if node_in_question in obstacle_list:
        return True

def check_obstacle_exists(obstacle_list):
    """sanity check to see if obstacle exists"""
    for obst in obstacle_list:
        if configSpace.search_space[obst[0],obst[1]] == 1:
            print("yes", configSpace.search_space[obst[0],obst[1]])

   
def compute_distance(current_pos, another_pos):
    """compute distance"""
    dist = sqrt((another_pos[0] - current_pos[0])**2+(another_pos[1]- current_pos[1])**2)
    
    return dist
    #return dist(current_pos, another_pos)

def check_out_bounds( current_position, x_bounds, y_bounds, bot_radius):
        """check out of bounds of configuration space"""
        
        if current_position[0] < x_bounds[0]+bot_radius or current_position[0] > x_bounds[1]-bot_radius:
            return True
        
        if current_position[1] < y_bounds[0]+bot_radius or current_position[1] > y_bounds[1]-bot_radius:
            return True
        
        return False
    
def dist(p1, p2):
	return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

# This function takes the waypoints in nodes and steers to them
# one at a time to within way_toler 
def traj_pointer(tar_node, cur_loc):
	des_direction = atan2(tar_node[1]-cur_loc[1],tar_node[0]-cur_loc[0])
	tar_distance = dist(cur_loc, tar_node)
	return tar_distance, des_direction



def heading_control(des_direction,cur_head):
	Kp = 2.5 #0.02
	Ki = 0.00
	Kd = 0.06 #0.1
	ang_vel_limit = .15

	global yaw_error_prev
	global yaw_error_sum

	yaw_error = (des_direction - cur_head)

	# Small wrapping help for yaw angles that are actually close to each other
	if yaw_error > 6.2832:
		yaw_error = yaw_error - 6.2832
	if yaw_error < -6.2832:
		yaw_error = yaw_error + 6.2832

	yaw_error_dot = (yaw_error - yaw_error_prev)/dt
	yaw_error_sum = yaw_error_sum + yaw_error*dt
	yaw_error_prev = yaw_error

	ang_vel_cmd = Kp*yaw_error + Ki*yaw_error_sum + Kd*yaw_error_dot
	if ang_vel_cmd > ang_vel_limit:
		ang_vel_cmd = ang_vel_limit
	if ang_vel_cmd < -ang_vel_limit:
		ang_vel_cmd = -ang_vel_limit
	return ang_vel_cmd




def odom_callback(data):
	global odom, imu_data, yaw
	odom = data
	imu_data = data.pose.pose
	quat_list = [imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w]
	(roll, pitch, yaw) = euler_from_quaternion(quat_list)


def Dijkstra(x_span, y_span,spacing, start_position, goal_point, obstacle_list, obstacle_radius,start_time, bot_radius):
    
    
    
    #%% ##### BUILD WORLD
    configSpace = ConfigSpace(x_span, y_span, spacing)
    configSpace.set_graph_coords()
    
    x_bounds, y_bounds = configSpace.get_x_y_coords()
    configSpace.set_obstacles(obstacle_list)
   



    turtle = Turtle(start_position[0],start_position[1],spacing)

    current_node = Node(turtle.position[0], turtle.position[1], 0, -1)
    current_index = configSpace.calc_index(turtle.position)
    turtle.not_visited[current_index] = current_node
    new_node_list = []
    node_cost_transaction_list= []
    while len(turtle.not_visited) != 0:
    
        current_node_index = min(turtle.not_visited, key=lambda x:turtle.not_visited[x].cost)
#        current_node_index = min(turtle.not_visited)
#        current_node_index = min(turtle.not_visited,  key=lambda x:turtle.not_visited[x].cost)
        current_node = turtle.not_visited[current_node_index]
#        print(current_node.x,current_node.y,current_node.cost,current_node.index)
        turtle.position = [current_node.x, current_node.y]
        turtle.visited_history[current_node_index] = current_node
        del turtle.not_visited[current_node_index]
        
#        if [current_node.x, current_node.y]  == goal_point:
#            #Have method to return path
#            print("I've arrived!", current_node.x, current_node.y)
#            break
        
#        print("turtle position is", turtle.position)
    
        for move in turtle.move_list:
            new_position = [turtle.position[0] + move[0], 
                            turtle.position[1] +move[1]]
            new_index = configSpace.calc_index(new_position)
            
            
            greedy_cost = compute_distance(new_position, [current_node.x, current_node.y]) + current_node.cost

            new_node = Node(new_position[0], new_position[1], greedy_cost, current_node_index)
           
            new_node_list.append([new_node.x,new_node.y,new_node.cost,new_node.index])
            if new_index in turtle.visited_history:
                continue
                
            if check_out_bounds(new_position, x_span, y_span, bot_radius) == True:
                
                continue
            
            if check_if_obstacle_is_present(obstacle_list, new_position) == True:
#                print('obstacle',new_index)
                continue
            
            if check_within_obstacle(obstacle_list, new_position, obstacle_radius) == True:
                continue
            if new_index not in turtle.not_visited:
                turtle.not_visited[new_index] = new_node
                continue
            if new_node.cost < turtle.not_visited[new_index].cost:
                node_cost_transaction_list.append([])
                turtle.not_visited[new_index].cost = new_node.cost
                turtle.not_visited[new_index].index = new_node.index
                continue    
    path_x = []
    path_y = []
    path_xy=[]
    
    goal_node = Node(goal_point[0],goal_point[1],0,0)
    path_index = configSpace.calc_index([goal_node.x,goal_node.y])
    path_x.append(turtle.visited_history[path_index].x)
    path_y.append(turtle.visited_history[path_index].y)
    
#    print (path_index)
#    print(turtle.visited_history[394].index)
    while turtle.visited_history[path_index].index != -1:
        path_index = turtle.visited_history[path_index].index        
        path_x.append(turtle.visited_history[path_index].x)
        path_y.append(turtle.visited_history[path_index].y)
        path_xy.append([turtle.visited_history[path_index].x,turtle.visited_history[path_index].y])
    first_point = goal_point
    final_cost = 0
    for point in path_xy:
        part_of_the_cost = compute_distance(first_point, point)
        final_cost = final_cost + part_of_the_cost
        first_point = point
#    print('The cost is: ', final_cost )
    
    #print("--- %s seconds ---" % (time.time() - start_time))
    #print('The x path is:',path_x)
    #print('The y path is:',path_y)
     #plotting grid#
    
    
       

    return path_x, path_y, path_xy, final_cost

def evasion_waypoints(x_span, y_span,spacing, start_position, goal_point, obstacle_list, obstacle_radius,start_time, bot_radius):
	#wayp = [0.0,3.0]# np.array([1.0,3.0]) # Starting at 3,3 (TB1 start position from Launch file)
	#if random_evader == True:
	#	for i in range(0,4): # Making 4 waypoints
	#		#xy_rand = random.randint(-1.0, 2.0), random.randint(-1.0, 2.0) # biased to keep going in approximately same direction
	#		xy_rand = random.randint(4.0, 5.0), random.randint(4.0, 5.0)
	#		wayp = np.vstack((wayp,xy_rand)) # This keeps bot from attempting to circle back and forth (too much)
	#else:
	#	wayp = np.vstack((wayp,[8.0, 8.0])) # Fixed target/trajectory. 


    #obstacle_list = []
    #obstacle_list = [[1,1], [4,4], [3,4], [5,0], [5,1], [0,7], [1,7], [2,7], [3,7], [4,7]]
    #obstacle_list = [[2,2], [2,3], [2,4], [2,5], [0,5], [1,5], [2,5], [3,5], [4,5], [5,5], [8,2], [9,2], [10,2], [11,2], [12,2], [13,3], [8,4], [8,5], [8,6], [8,7], [8,8], [8,9], [8,7], [2,7], [3,7], [4,7], [5,7], [6,7], [7,6], [9,6], [10,6], [11,6], [12,6], [15,8], [2,9], [2,10], [2,11], [2,12], [2,13], [5,9], [5,10], [5,11], [5,12], [5,13], [5,14], [5,15], [6,12], [7,12], [8,12], [9,12], [10,12], [11,12], [12,8], [12,9], [12,10], [12,11], [12,12]

	path_x, path_y, path_xy, final_cost = Dijkstra(x_span, y_span,spacing, start_position, goal_point, obstacle_list, obstacle_radius,start_time, bot_radius)
	wayp = path_xy[0]
	list_length = len(path_xy) - 1
	for i in range(0,list_length):
		print("index is: ",i)
		wayp = np.vstack((wayp,path_xy[i+1]))
	return wayp # Return list of waypoints for trajectory/path following
if __name__=="__main__":

	start_time = time.time()
	x_span = [0,15]
	y_span = [0,15]
	spacing = 1
	start_position = [2,1]
	goal_point = [7,2]
	bot_radius = .5
	obstacle_list = [[5,0],[5,1],[5,2],[5,3],[5,4],[0,5],[1,4],[2,3],[3,2],[3,3]]
	obstacle_radius = 0.5

	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('turtlebot3_evader')

	pub = rospy.Publisher('/turtlebot1/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber("/turtlebot1/odom",Odometry,odom_callback)

	turtlebot3_model = "burger"

	
	twist.angular.z = 0
	twist.linear.x = 0.0	
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x = 0
	twist.angular.y = 0
	
	pub.publish(twist) # Making sure vehicle is starting from rest.

	rospy.sleep(1.0) # let system initialize before taking off

	#trajectory = np.array(0) # Fix this, need to stick in random waypoints

	trajectory = evasion_waypoints(x_span, y_span,spacing, start_position, goal_point, obstacle_list, obstacle_radius,start_time, bot_radius) # Call function to generate waypoints
	print ("Trajectory of evader is ", trajectory)

	cur_pose = odom.pose.pose.position
	cur_loc = cur_pose.x, cur_pose.y
	cur_wpt = 0 # Initialize which waypoint we are going to
	last_wpt = trajectory.shape[0] # Number of points in trajectory

	vel_cmd = 0.1 # Constant forward speed - half speed
	#global yaw, cur_loc

	while(cur_wpt < last_wpt): # Stop when the last wpt has been reached
		last_wpt = trajectory.shape[0] # Updates as the algorithms are recalled
		cur_pose = odom.pose.pose.position
		cur_loc = cur_pose.x, cur_pose.y

		if (dist(cur_loc,trajectory[cur_wpt]) < 0.1): # move to next wpt when close
			cur_wpt = cur_wpt + 1
			if (cur_wpt == last_wpt):
				break

		dist_to_wpt, des_dir = traj_pointer(trajectory[cur_wpt], cur_loc) # returns distance and angle to waypoint
		ang_vel_cmd = heading_control(des_dir,yaw)

		twist = Twist()
		twist.linear.x = vel_cmd
		twist.angular.z = ang_vel_cmd
		
		pub.publish(twist) 
		
		#print twist.linear.x
		#print "Position",cur_pose.x, cur_pose.y, trajectory[cur_wpt]
		#print "Angles",perp_angle, wpt_angle, des_dir, yaw
		#print "Wpt Info", dist_to_traj, dist_to_wpt, fake_dist, dist_ratio
		#print "Cmds", ang_vel_cmd, vel_cmd

		rospy.sleep(dt)

	# Last waypoint reached, stop the turtlebot before terminating
	twist.linear.x = 0
	twist.angular.z = 0
	pub.publish(twist)
   

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
