#!/usr/bin/env python3
# -*- coding: utf-8 -*- 

"""
Proportional Navigation test script 
wrt means with respect to
Lidar sensor:
gives Range and Angle of detection from where turtlebot is at 
https://gamedev.stackexchange.com/questions/17313/how-does-one-prevent-homing-missiles-from-orbiting-their-targets
"""

import rospy
import numpy as np
from unmanned_systems import Turtlebot
import time 
#from unmanned_systems import util_functions

class PN():
    def __init__(self, dt, N=None):
        self.dt = dt 
        self.old_target_heading = None 
        self.old_target_position = None
        self.LOS = [0,0]
        self.LOS_dot_old = 0
        
        if N != None:
            self.N = N
        else:
            self.N = 0.1

    """dont need the this other stuff... using lidar"""    
    def update_vals(self, target_heading, pursuer_heading, LOS_dot):
        """update all my old values"""                
        self.LOS[1] = self.LOS[0]
        self.LOS[0] = target_heading  
        self.LOS_dot_old = LOS_dot
        
    def check_same_target_heading(self):
        if self.LOS[0] == self.LOS[1]:
            return True

    def get_turn_rate(self, target_heading, pursuer_heading, target_position):
        """very basic PN"""
        #for lidar it already gives you your relative position from detected targets
        # if (self.old_target_heading!=None) and (self.old_target_position!=None):
        LOS_dot = (self.LOS[0] - self.LOS[1])/self.dt 
        if self.check_same_target_heading():
            LOS_dot = self.LOS_dot_old
        
        if abs(target_heading)<=1.0:
            LOS_dot=0.0    
        
        self.update_vals(target_heading, pursuer_heading, LOS_dot)            
        print(LOS_dot*self.N)
        return LOS_dot*self.N        
    
def compute_mean(some_list):  
    """calculates the mean of list"""
    return sum(some_list)/len(some_list)

if __name__ == '__main__':
    
    rospy.init_node('prop_nav_tb')
    rate_val = 100
    rate = rospy.Rate(rate_val)
    
    #N_val = 0.625
    #N_val = 100 #1000%
    N_val = 10 #100%
    #N_val = 1 #10%
    pn = PN(dt=rate_val, N=N_val)
    
    #instantiate turtlebot class 
    tb_pursuer = Turtlebot.Turtlebot(0.0,0,0, rate_val, name='turtlebot2', 
                                     lidar_track=True)
    #forward_speed = 0.145
    forward_speed = 0.2 #for problem 1 and 5
    #forward_speed = 0.2 #for problem 4
    start_time = time.time()
    #nowish = 0
    #while nowish < 2.5:
    #    tb_pursuer.go_forward_turn(forward_speed,.4)
    #    nowish = time.time() - start_time
    while not rospy.is_shutdown():
        #nowish = time.time() - start_time
        #print("the time is", nowish)
        #A = pn.get_turn_rate()
        if tb_pursuer.detected_heading_angle_list and tb_pursuer.detected_range_list:                
                
            target_angle_mean = compute_mean(tb_pursuer.detected_heading_angle_list)
            target_range_mean = compute_mean(tb_pursuer.detected_range_list)
            
            los_dot = pn.get_turn_rate(target_heading=target_angle_mean, pursuer_heading=tb_pursuer.odom_yaw_deg, 
                                       target_position=target_range_mean)
    
            if los_dot != None:
                tb_pursuer.go_forward_turn(forward_speed, los_dot)
    
    
