#! /usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import random
import math
from gazebo_msgs.msg import *
import numpy as np
import csv
import rospkg
import matplotlib.pyplot as plt
from matplotlib import cm
import time
from environment import Env

if __name__ == "__main__": 
    rospy.init_node("path_controller_node", anonymous=False)
    
    env = Env()
    state_scan = env.reset()
    r = rospy.Rate(5) # 10hz

    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        pass

    while not rospy.is_shutdown():
        # FACA SEU CODIGO AQUI

        if (not env.goal) and (not env.moving):
            env.moving = True
            goal = MoveBaseGoal()

            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()

            goal.target_pose.pose.position = Point(env.goal_x,env.goal_y,0)
            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = 0.0
            goal.target_pose.pose.orientation.w = 1.0
            
            ac.send_goal(goal)

        elif env.goal:
            env.moving = False

        state_scan = env.step(None)
                
        r.sleep()
