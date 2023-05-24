#!/usr/bin/env python
"""
RRT-Based Local Path Planner ROS node
That Generates Local Waypoints.

More details, flowcharts and block diagrams about this node and the whole package in generai is available in the gitHub repository.

This ROS node generates one thread per vessel in the simulation environment. 
And in one thread these happen:
    Thread subscribes to /VesselX/VesselsInShipDomain ROS topic and /VesselX/Is_On_Global_Path to check if any vessels enter to the ship domain of own vessel.
    If a vessel enters to ship domain AND  /VesselX/Is_On_Global_Path == 1  then it means the thread needs to start generating a local path. 
        At that point, Switch_Mechanism ROS node will also be notified since it's subscribed to /VesselX/Is_On_Global_Path and /VesselX/VesselsInShipDomain. And it will set the /VesselX/Is_On_Global_Path to 2
    
    When the local path is generated, it will be published to /VesselX/Local_Trajectory and Switch_Mechanism ROS node will set the /VesselX/Is_On_Global_Path topic to 0 when it takes the  /VesselX/Local_Trajectory.

    After local path is generated and is being tracked by Trajectory_Tracker ROS node, this node will keep track of the local path and check if it gets interrupted.
    This will be done this way: 
        While While local path is not completed: #(while /VesselX/Is_On_Global_Path != 1) (last waypoint of local path is not reached) (checked by switch mechanism)
            Load current vessels inside ship domain and check if any vessel interferes with local path
            If an interference to local path occurs:
                Re-Generate local path 
"""

from os import kill
import rospy
import time
import math
from rospy.core import rospydebug
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray,Int16
import threading
import json
import random
import pyproj as proj
#import matplotlib.pyplot as plt
#import pygame
from geopy import distance
import os
from tf.transformations import euler_from_quaternion

from multivessel_msgs.msg import VesselPose,Perception

class RRT():
    def __init__(self,start,goal,vessel_poses_,VesselID_,yaw_):

        self.vesselID = VesselID_
        RRT_parameters = rospy.get_param("/local_path_planning_parameters")
        map_area_coefficient = RRT_parameters["map_area_coefficient"]
        ship_domain_radius = rospy.get_param("/vessel"+str(self.vesselID)+"_Ship_Domain_Radius")
        map_area = map_area_coefficient * ship_domain_radius

        self.own_vessel_yaw = yaw_

        self.dmax = RRT_parameters["d_max"] # This is the maximum distance from one node to another node. This is the maximum edge lenght
        self.vessel_forbidden_zone_coefficient_length = RRT_parameters["vessel_forbidden_zone_coefficient_length"]
        self.vessel_forbidden_zone_coefficient_width = RRT_parameters["vessel_forbidden_zone_coefficient_width"]
        self.vessel_forbidden_zone_coefficient_draft = RRT_parameters["vessel_forbidden_zone_coefficient_draft"]
        self.vessel_forbidden_zone_radius_equation = RRT_parameters["vessel_forbidden_zone_radius_equation"]

        #The description of the variables of constructor is below:
        
        #start variable is the starting point of the vessel in Lat-Lon coordinates.
        #And it is the pose of the vessel when this object is defined.
        #Value of this variable is provided from the Vessel_Controller object.

        #goal variable is the global goal point provided from global path planner in Lat-Lon coordinates..
        #Value of this variable is provided from the Vessel_Controller object.
        #And from the value of this variable, local_goal variable will be generated 
        #in order to set a goal point inside the bounding box of the RRT.

        #vessel_poses_ variable stores the locations of the vessels taken from the topic /Vessel*VESSEL_ID*/Perception
        #Value of this variable is provided from the Vessel_Controller object.

        #map_area_ is needed in order to set the bounding box of the RRT. 
        #Value of this variable is provided from the config.json file.

        #vessel_radius_ and vessel_safezone_ variables define the radius of circular forbidden_zones of the vessels. 
        #Value of these variables are provided from config.json file.
        #vessel_radius variable is defined in order to generate different forbidden zones depending on the lenght of the vessel
        
        #d_max_ is the maximum distance that a node can be placed distant from another node. Therefore, this is the maximum edge lenght.
        #Value of this variable is provided from the config.json file.

        self.map_X_min = -1*map_area
        self.map_X_max = map_area
        self.map_Y_min = -1*map_area
        self.map_Y_max = map_area

        self.map_area = map_area

        #We need to know our starting point and our goal point
        self.startLatLon = start
        self.goalLatLon = goal

        self.goal =(1,1)
        #local goal point is RRT goal
        #goal point in xy
        #This is a temporary value to test the functions. This value will be calculated appropriately considering the next global goal point. 
        #This calculation is being done in self.Generate_Local_Goal_Point()

        self.origin_lat = self.startLatLon[0]
        self.origin_lon = self.startLatLon[1]
        self.cust = proj.Proj("+proj=aeqd +lat_0={0} +lon_0={1} +datum=WGS84 +units=m".format(self.origin_lat, self.origin_lon))

        (startLat,startLon) = start
        x,y = self.get_local_coord( startLat,startLon )
        self.start = (x,y)
        print("self.start",self.start)
        #This will be 0,0 since the starting point is origin as well.

        self.vessel_poses = vessel_poses_

        self.vessel_poses_xy = []
        #A list of vessel poses in XY coordinates type.
        #This list will store the positions in form  of a list of Vessel_Pose() object
        print("Inside RRT")
        self.virtual_obstacles_xy = []

        self.Convert_Positions_to_XY()

        #We need a flag to store the data of if our graph reached the goal point or not
        self.tree_reached_to_goal_flag = False

        #We need three lists to store our graph
        #self.x, self.y, self.parent
        #x and y are the XY values of each node. Basically the location of each node
        #self.parent list stores each node's parent node information.
        self.x = []
        self.y = []
        self.parent = []

        #We need to initialize our graph
        #What is the first node of our graph? The starting point of the vessel of course.
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0) # We assume the parent node of the node 0 is node 0 itself. 

        self.obs_x = []#used in isFree
        self.obs_y = []

        self.iteration = 0
        self.t1 = 0
        self.t1 = time.time()

        #When our algorithm finishes it's work (one node of the graph reaches to goal point)
        #We will have a path
        #Our vessel will follow this path in order to reach it's goal position
        self.goalstate = None
        self.path = []
        self.pathCoords = []
        self.pathCoordsTopic = []

        self.smoothPath = []
        self.smoothPathLatLon = []

        self.goalFlag = False

        self.nodeRad = 0
        self.nodeThickness = 0
        self.edgeThickness =2

        self.Calculate_Local_Goal()

        #After local goal is calculated, publish the local map size to ROS param /VesselX_Local_Map_Size
        self.local_map_dimensions = []
        self.local_map_dimensions.append(self.map_X_min)
        self.local_map_dimensions.append(self.map_X_max)
        self.local_map_dimensions.append(self.map_Y_min)
        self.local_map_dimensions.append(self.map_Y_max)

        rospy.set_param('vessel'+str(self.vesselID)+"_Local_Map_Dimensions", self.local_map_dimensions)


        if self.virtual_obstacles_xy != []:
            self.vessel_poses_xy=self.vessel_poses_xy + self.virtual_obstacles_xy

        self.Remove_Obstacles_That_Collide_With_Starting_Point()

        self.Generate_Local_Path()

    def Generate_Local_Path(self):
        self.iterations()

        print("Number of Path Nodes: ", len(self.path) )
        print("Total number of nodes: ", len(self.x))
        self.getPathCoords()
        print(self.pathCoordsTopic)
        print( len(self.pathCoords) )
        smoothPath = self.PathSmoothen()
        
        self.smoothPath.reverse()#This is done to make the path start from zero and go to the goal point.
        print(self.smoothPath)        

        self.Convert_smoothenPath_to_LatLon()
        return self.smoothPathLatLon 

    def Convert_smoothenPath_to_LatLon(self):

        smoothPath = self.smoothPath

        for i in range(int(len(smoothPath))):
            pathPointX = smoothPath[i][0]
            pathPointY = smoothPath[i][1]
            lonPoint,latPoint = self.XY_to_LatLon(pathPointX,pathPointY)
            self.smoothPathLatLon.append(latPoint)
            self.smoothPathLatLon.append(lonPoint)
    
    def Calculate_Local_Goal(self):
        #This function generates a local goal point from the global goal point that have been taken from the class' arguments.
        self.Find_the_Projection_of_Global_Waypoint_to_Local_Map()
        self.Generate_COLREGS_Virtual_Obstacles()

        #After the initial position of the goal point is generated, check if it collides with any obstacle in the environment.
        self.is_goal_point_free = self.is_xy_point_Free_w_args(self.goal[0],self.goal[1])
        
        if self.is_goal_point_free == True:
            return 1
            
        elif self.is_goal_point_free == False:
            while self.is_goal_point_free == False:
                #While we can't find any local goal point that is free from obstacles:
                #Enlarge the map edges by 40 percent
                self.map_X_min = self.map_X_min*1.2
                self.map_X_max = self.map_X_max*1.2
                self.map_Y_min = self.map_Y_min*1.2
                self.map_Y_max = self.map_Y_max*1.2
                #After enlarging the map, re-generate the COLREGS virtual obstacles
                self.Generate_COLREGS_Virtual_Obstacles()
                self.Find_the_Projection_of_Global_Waypoint_to_Local_Map()
                self.vessel_poses_xy=self.vessel_poses_xy + self.virtual_obstacles_xy
                self.is_goal_point_free = self.is_xy_point_Free_w_args(self.goal[0],self.goal[1])
                if self.is_goal_point_free == True:
                    return 1
                    
                elif self.is_goal_point_free == False:
                    #If the local goal is still couldn't be found, drag the local goal point to either right or left depending on it's position.
                    #Try to shift the local goal point clockwise. 
                    self.Push_Local_Goal_Clockwise()
            print("Found a local goal.")

    def Push_Local_Goal_Clockwise(self):
        #should be called from the calculate local goal function. 
        #Takes the local goal point's initial position and stores it into a variable.
        # In each iteration of the loop:
        #   Pushes the goal point clockwise (initially right)
        #   Checks if the new local goal is free. If so, returns 1
        #   Else if local goal is not free, checks if we got too close to the edge of the map. 
        #       If we are too close to right edge, switch the pushing to bottom
        #       If we are too close to bottom edge, switch the pushing to left
        #       If we are too close to left edge, switch the pushing to up
        #       If we are too close to top edge, switch the pushing to right
        #   If we are not too close to any edge, no problem.
        #   If we ended up colliding our starting point, that means we did a full circle but yet couldn't find any good local goal spots. 
        #       That means we should return 0 to restart the bigger loop that we are in. 
        
        self.initial_position_of_goal = self.goal
        self.distance_to_initial_goal_point = 999999#placeholder to start the loop.
        iteration_count = 0

        self.pushing_value_x = self.dmax
        self.pushing_value_y = 0
        while (self.distance_to_initial_goal_point > self.dmax)and (iteration_count > 15):
            #Added one more condition for iteration count because I don't want this loop to stop right after it started.
            
            #Check if we got close to any walls. If we did, change the pushing values.
            distance_to_right_wall  = math.sqrt( (float(self.map_X_max) -float(self.goal[0]))**2 )
            distance_to_left_wall   = math.sqrt( (float(self.map_X_min) -float(self.goal[0]))**2 )
            distance_to_top_wall    = math.sqrt( (float(self.map_Y_max) -float(self.goal[1]))**2 )
            distance_to_bot_wall    = math.sqrt( (float(self.map_Y_min) -float(self.goal[1]))**2 )

            if distance_to_right_wall < self.dmax:
                #if we got too close to the right wall, back off a bit and change the pushing values.
                self.goal[0] = float(self.goal[0]) - float(self.dmax)*1.2
                self.pushing_value_y = float(-1)*(self.dmax)
                self.pushing_value_x = 0
            elif distance_to_left_wall < self.dmax:
                #if we got too close to the left wall, back off a bit and change the pushing values.
                self.goal[0] = float(self.goal[0]) + float(self.dmax)*1.2
                self.pushing_value_y = self.dmax
                self.pushing_value_x = 0
            elif distance_to_top_wall < self.dmax:
                #if we got too close to the top wall, back off a bit and change the pushing values.
                self.goal[1] = float(self.goal[1]) - float(self.dmax)*1.2
                self.pushing_value_y = 0
                self.pushing_value_x = self.dmax
            elif distance_to_bot_wall < self.dmax:
                #if we got too close to the top wall, back off a bit and change the pushing values.
                self.goal[1] = float(self.goal[1]) + float(self.dmax)*1.2
                self.pushing_value_y = 0
                self.pushing_value_x = float(-1)*self.dmax

            self.goal[0] = self.goal[0] + self.pushing_value_x
            self.goal[1] = self.goal[1] + self.pushing_value_y

            self.is_goal_point_free = self.is_xy_point_Free_w_args(self.goal[0],self.goal[1])
            if self.is_goal_point_free == True:
                    return True

            self.distance_to_initial_goal_point = math.sqrt( ( self.initial_position_of_goal[0] - self.goal[0] )**2 + (self.initial_position_of_goal[1] - self.goal[1])**2 )
            iteration_count = iteration_count + 1

            pass
         
        pass
    
    def Find_the_Projection_of_Global_Waypoint_to_Local_Map(self):
        #this function is called from Calculate_Local_Goal function. And it finds the local goal 
        goal_x,goal_y = self.get_local_coord( self.goalLatLon[0],self.goalLatLon[1] )

        if ( abs(goal_x) < self.map_X_max ) and ( abs(goal_y) < self.map_Y_max ):
            #if the goal point is inside the RRT map, no need to put it to edge.
            self.goal = (goal_x,goal_y)
        elif ( abs(goal_x) > self.map_X_max ) or ( abs(goal_y) > self.map_Y_max ):
            #if the goal point is outside the RRT map, we need to put it to edge.
            #4 conditions might occur. The goal point might either be at the top, right, bottom or left side of the RRT map. 
            #Find where it is. There are 8 regions I defined. These regions are:
            #   TopLeft,Top,TopRight,Left,Right,Botleft,Bot,Botright of the RRT map
            if (goal_x < self.map_X_min) and (goal_y > self.map_Y_max):
                #TopLeft
                self.goal = (self.map_X_min*0.9 , self.map_Y_max*0.9)

            elif (abs(goal_x) < self.map_X_max) and (goal_y > self.map_Y_max):
                #Top
                self.goal = (goal_x, self.map_Y_max*0.9)

            elif (goal_x > self.map_X_max) and (goal_y > self.map_Y_max):
                #TopRight
                self.goal = (self.map_X_max*0.9 , self.map_Y_max*0.9)

            elif (goal_x < self.map_X_min) and (abs(goal_y) < self.map_Y_max):
                #Left
                self.goal = (self.map_X_min*0.9 , goal_y)

            elif (goal_x > self.map_X_max) and (abs(goal_y) < self.map_Y_max):
                #Right
                self.goal = (self.map_X_max*0.9 , goal_y)

            elif (goal_x < self.map_X_min) and (goal_y < self.map_Y_min):
                #BotLeft
                self.goal = (self.map_X_min*0.9 , self.map_Y_min*0.9)

            elif (abs(goal_x) < self.map_X_max) and (goal_y < self.map_Y_min):
                #Bottom
                self.goal = (goal_x, self.map_Y_min*0.9)

            elif (goal_x > self.map_X_max) and (goal_y < self.map_Y_min):
                #BotRight
                self.goal = (self.map_X_max*0.9 , self.map_Y_min*0.9)
    
    def Generate_COLREGS_Virtual_Obstacles(self):
        #This function generates virtual obstacles for each ship inside the ship domain.
        #This function should remove all the virtual obstacles from before upon starting. Vessel forbidden zones should be kept.
        self.virtual_obstacles_xy = []

        iteration_count = 0
        vessel_poses = self.vessel_poses_xy.copy()
        while(len(vessel_poses)>0 ):
            vessel_pose_=vessel_poses.pop(0)
            relative_angle = math.atan((vessel_pose_.position.y)/(vessel_pose_.position.x))
            colregs_angle = self.own_vessel_yaw-relative_angle 
            #colregs_angle -= math.pi/3

            roll,pitch, ts_angle_yaw =  euler_from_quaternion([vessel_pose_.orientation.x, vessel_pose_.orientation.y, vessel_pose_.orientation.z, vessel_pose_.orientation.w])
            #ts_angle_yaw = -1*ts_angle_yaw

            if colregs_angle < 0:
                colregs_angle += 2*math.pi
            elif colregs_angle > 2*math.pi:
                colregs_angle -= 2*math.pi

            colregs_angle_degrees = math.degrees(colregs_angle)
            print("colregs_angle_degrees",colregs_angle_degrees)

            if (colregs_angle_degrees > 5) and (colregs_angle_degrees <= 112.5):
                print("OS is rule 15-give_way")
                forbidden_zone_radius = self.calc_forbidden_zone_radius(vessel_pose_.vessel_details.Length, vessel_pose_.vessel_details.Width)
                self.virtual_obs_distance_x = forbidden_zone_radius
                self.virtual_obs_distance_y = forbidden_zone_radius
                self.virtual_obs_x_minus_one = vessel_pose_.position.x
                self.virtual_obs_y_minus_one = vessel_pose_.position.y

                while (abs(self.virtual_obs_x_minus_one)<self.map_X_max ) and (abs(self.virtual_obs_y_minus_one)<self.map_Y_max ):
                    print("ts_angle_yaw",ts_angle_yaw)  
                    virtual_obs_x = self.virtual_obs_x_minus_one + math.sin(ts_angle_yaw)*self.virtual_obs_distance_x
                    virtual_obs_y = self.virtual_obs_y_minus_one + math.cos(ts_angle_yaw)*self.virtual_obs_distance_y

                    virtual_obs_xy = VesselPose()
                    virtual_obs_xy.position.x = virtual_obs_x
                    virtual_obs_xy.position.y = virtual_obs_y

                    virtual_obs_xy.vessel_details.Length = vessel_pose_.vessel_details.Length
                    virtual_obs_xy.vessel_details.Width = vessel_pose_.vessel_details.Width
                    self.virtual_obstacles_xy.append(virtual_obs_xy)

                    self.virtual_obs_x_minus_one = virtual_obs_x  
                    self.virtual_obs_y_minus_one = virtual_obs_y

            elif (colregs_angle_degrees > 112.5) and (colregs_angle_degrees <= 247.5):
                print("OS is rule 13-give_way")
            elif (colregs_angle_degrees > 247.5) and (colregs_angle_degrees <= 355):
                print("OS is rule 15-stand_on")
            elif ( (colregs_angle_degrees > 355) and (colregs_angle_degrees <= 360) ) or ((colregs_angle_degrees > 0) and (colregs_angle_degrees <= 5)):
                print("OS is rule 13-stand_on or rule 14")
                forbidden_zone_radius = self.calc_forbidden_zone_radius(vessel_pose_.vessel_details.Length, vessel_pose_.vessel_details.Width)
                self.virtual_obs_distance_x = forbidden_zone_radius
                self.virtual_obs_distance_y = forbidden_zone_radius
                self.virtual_obs_x_minus_one = vessel_pose_.position.x
                self.virtual_obs_y_minus_one = vessel_pose_.position.y
                
                while (abs(self.virtual_obs_x_minus_one)<self.map_X_max ) and (abs(self.virtual_obs_y_minus_one)<self.map_Y_max ):
                    virtual_obs_x = self.virtual_obs_x_minus_one - math.sin(ts_angle_yaw)*self.virtual_obs_distance_x
                    virtual_obs_y = self.virtual_obs_y_minus_one + math.cos(ts_angle_yaw)*self.virtual_obs_distance_y

                    virtual_obs_xy = VesselPose()
                    virtual_obs_xy.position.x = virtual_obs_x
                    virtual_obs_xy.position.y = virtual_obs_y

                    virtual_obs_xy.vessel_details.Length = vessel_pose_.vessel_details.Length
                    virtual_obs_xy.vessel_details.Width = vessel_pose_.vessel_details.Width
                    self.virtual_obstacles_xy.append(virtual_obs_xy)

                    self.virtual_obs_x_minus_one = virtual_obs_x  
                    self.virtual_obs_y_minus_one = virtual_obs_y

            iteration_count +=1
        pass
    
    def Convert_Positions_to_XY(self):
        #This function converts every position variable to XY 
        #Takes the Lat-Lon values from the self.vessel_poses list
        #and puts the XY conversions in the self.vessel_poses_xy list
        self.vessel_poses_xy = []
        #Make sure to reset this list in order to not add the same vessel positions over and over again.
        for i in range(len(self.vessel_poses)):
            pos_x,pos_y = self.get_local_coord(self.vessel_poses[i].position.x,self.vessel_poses[i].position.y)
            #Convert the Lat-Lon values to XY type
            vessel_pose_xy = VesselPose()
            
            vessel_pose_xy.vessel_id = self.vessel_poses[i].vessel_id
            
            vessel_pose_xy.vessel_details.Length = self.vessel_poses[i].vessel_details.Length
            vessel_pose_xy.vessel_details.Width = self.vessel_poses[i].vessel_details.Width

            vessel_pose_xy.position.x = pos_x
            vessel_pose_xy.position.y = pos_y

            #Make sure to put the orientation values as well.
            vessel_pose_xy.orientation.x = self.vessel_poses[i].orientation.x
            vessel_pose_xy.orientation.y = self.vessel_poses[i].orientation.y
            vessel_pose_xy.orientation.z = self.vessel_poses[i].orientation.z
            vessel_pose_xy.orientation.w = self.vessel_poses[i].orientation.w
            self.vessel_poses_xy.append(vessel_pose_xy)

        pass

    def Remove_Obstacles_That_Collide_With_Starting_Point(self):
        #This function checks all the vessel positions and removes if any vessel forbidden zone collides with the starting point.
        #This function should check not only vessel_poses, but virtual_obstacles as well.
        (x,y) = (0, 0)
        iteration_count = 0
        vessel_poses = self.vessel_poses_xy.copy()
        while(len(vessel_poses)>0 ):
            vessel_pose_=vessel_poses.pop(0)
            px = (float(vessel_pose_.position.x) - float(x))**2
            py = (float(vessel_pose_.position.y) - float(y))**2
            distance = (px+py)**(0.5)
            #print(distance)
            forbidden_zone_radius = self.calc_forbidden_zone_radius(vessel_pose_.vessel_details.Length, vessel_pose_.vessel_details.Width)
            if distance < forbidden_zone_radius*1.5:
                self.vessel_poses_xy.pop(iteration_count)
                print("can't place an obstacle on top of the starting point!")
                pass
            else:
                iteration_count +=1

    def XY_to_LatLon(self, x_point, y_point):
        lon , lat = self.cust(x_point, y_point, inverse = True)
        return lon, lat

    def get_local_coord(self, lat, lon):
        #Converts the position information in Lat-Lon to XY.
        WORLD_POLAR_M = 6356752.3142
        WORLD_EQUATORIAL_M = 6378137.0

        eccentricity = math.acos(WORLD_POLAR_M/WORLD_EQUATORIAL_M)        
        n_prime = 1/(math.sqrt(1 - math.pow(math.sin(math.radians(float(lat))),2.0)*math.pow(math.sin(eccentricity), 2.0)))        
        m = WORLD_EQUATORIAL_M * math.pow(math.cos(eccentricity), 2.0) * math.pow(n_prime, 3.0)        
        n = WORLD_EQUATORIAL_M * n_prime

        diffLon = float(lon) - float(self.origin_lon)
        diffLat = float(lat) - float(self.origin_lat)

        surfdistLon = math.pi /180.0 * math.cos(math.radians(float(lat))) * n
        surfdistLat = math.pi/180.00 * m

        x = diffLon * surfdistLon
        y = diffLat * surfdistLat

        return x,y
    
    def generate_random_obs_circle(self,obs_count):
        #generates random center point for random obstacle creation. Not used for now.
        
        howMany=obs_count
        i =0

        while i < howMany:
            
            centerX = float(random.uniform(self.map_X_min, self.map_X_max))
            centerY = float(random.uniform(self.map_Y_min, self.map_Y_max))

            #if random obstacle collide with another obs or startPoint or goalPoint, 
            #don't add to i and dont append

            #if random obstacle doesn't collide, add i and append()

            #for j in range(1,len(self.obstacles_x)):
            if (self.isFreeforObs(centerX,centerY)):
                #if it's obstacle free, add it
                print(centerX,centerY)
                self.obstacles_x.append(centerX)
                self.obstacles_y.append(centerY)
                i = i+1

    def add_node(self, n, x, y):
        #These arguments are:
        #   n is the ID number of the node
        #   x and y are the xy coordinates of the nodeInt64MultiArray,MultiArrayLayout,
        self.x.insert(n,x)
        self.y.insert(n,y)

    def remove_node(self,n):
        #in order to remove a node, ID number of the node is enough
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self,parent,child):
        #Assume that we have 2 nodes. What makes an edge in between them?
        #if one of two is parent and the other one is child, that makes us the edge
        #in order to do that, we append the new node to the parent list we made in def __init__
        self.parent.insert(child,parent)

    def remove_edge(self, n):
        #How can we remove an edge?
        #if we cut the parent relationship in between 2 nodes, there is no edge anymore
        self.parent.pop(n)

    def number_of_nodes(self):
        #in order to know how many nodes we have, len of the x list is enough
        #you can also use len of the y list. The most simple function in this code
        return len(self.x)

    def distance(self, n1, n2):
        (x1,y1) = (self.x[n1], self.y[n1])
        (x2,y2) = (self.x[n2], self.y[n2])
        #what is the distance between two nodes in a map?
        #the euclidean distance! 
        # Since we know x and y of each node, we can use euclidean distance to calculate
        px = (float(x1) - float(x2))**2
        py = (float(y1) - float(y2))**2

        return (px+py)**(0.5)

    def generate_random_XY(self):
        #We need a random XY couple inside our map for RRT expansion.
        x = float(random.uniform(self.map_X_min, self.map_X_max))
        y = float(random.uniform(self.map_Y_min, self.map_Y_max))
    
        return x,y

    def calc_forbidden_zone_radius(self,length_,width_):
        #This function takes a ship's lenghth and width measurements.
        #And calculates the forbidden_zone radius for the ship
        length = length_
        width = width_
        
        self.vessel_forbidden_zone_coefficient_length

        if self.vessel_forbidden_zone_radius_equation == "W*Wc+L*Lc":
            forbidden_zone_radius = (length*self.vessel_forbidden_zone_coefficient_length) + (width*self.vessel_forbidden_zone_coefficient_width)

        #This radius can be calculated by different equations.
        #This function returns the forbidden_zone_radius. 

        return forbidden_zone_radius
    
    def is_xy_point_Free_w_args(self,x_,y_):
        (x,y) = (x_, y_)
        vessel_poses = self.vessel_poses_xy.copy()
        while(len(vessel_poses)>0 ):
            vessel_pose_=vessel_poses.pop(0)

            px = (float(vessel_pose_.position.x) - float(x))**2
            py = (float(vessel_pose_.position.y) - float(y))**2
            distance = (px+py)**(0.5)

            forbidden_zone_radius = self.calc_forbidden_zone_radius(vessel_pose_.vessel_details.Length, vessel_pose_.vessel_details.Width)

            if distance < forbidden_zone_radius:
                
                print("This point is not free from obstacles. Try again.")
                
                return False
        
        
        virtual_obstacle_poses = self.virtual_obstacles_xy.copy()        
        while(len(virtual_obstacle_poses)>0 ):
            virtual_obs_pose_=virtual_obstacle_poses.pop(0)

            px = (float(virtual_obs_pose_.position.x) - float(x))**2
            py = (float(virtual_obs_pose_.position.y) - float(y))**2
            distance = (px+py)**(0.5)

            forbidden_zone_radius = self.calc_forbidden_zone_radius(virtual_obs_pose_.vessel_details.Length, vessel_pose_.vessel_details.Width)

            if distance < forbidden_zone_radius:
                
                print("This point is not free from obstacles. Try again.")
                
                return False
        
        return True
     
    def isFree(self):
        #When we locate a node to the map, we want to make sure that
        #the location of the node is in free space
        n = self.number_of_nodes() - 1
        (x,y) = (self.x[n], self.y[n])

        vessel_poses = self.vessel_poses_xy.copy()
        while(len(vessel_poses)>0 ):
            vessel_pose_=vessel_poses.pop(0)

            px = (float(vessel_pose_.position.x) - float(x))**2
            py = (float(vessel_pose_.position.y) - float(y))**2

            distance = (px+py)**(0.5)

            forbidden_zone_radius = self.calc_forbidden_zone_radius(vessel_pose_.vessel_details.Length, vessel_pose_.vessel_details.Width)

            if distance < forbidden_zone_radius:
                self.remove_node(n)
                print("can't place a node on top of a forbidden zone!")
                
                return False
        
        return True
   
    def crossObstacle(self, x1, x2, y1, y2):
        #We want to place the node in free space. That is necessary but not enough
        #We also want that none of our edges are on an obstacle
        vessel_poses = self.vessel_poses_xy.copy()
        while(len(vessel_poses)>0 ):
            vessel_pose_=vessel_poses.pop(0)
            forbidden_zone_radius = self.calc_forbidden_zone_radius(vessel_pose_.vessel_details.Length, vessel_pose_.vessel_details.Width)
            for i in range(0,201):
                u = i/200
                x = x1*u + x2*(1-u)
                y = y1*u + y2*(1-u)

                px = (float(vessel_pose_.position.x) - float(x))**2
                py = (float(vessel_pose_.position.y) - float(y))**2
                distance = (px+py)**(0.5)


                if distance < forbidden_zone_radius:
                    return True
        return False

    def connect(self,n1,n2):
        #take both xy values of each node  to a temporary variable
        (x1,y1) = (self.x[n1],self.y[n1])
        (x2,y2) = (self.x[n2],self.y[n2])

        if self.crossObstacle(x1,x2,y1,y2):
            #If we want to connect two nodes, we need to make sure that there is no obstacle in between them
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1,n2)
            return True

    def nearest(self, n):
        #How we add a new node to the RRT tree?
        #First, we pick a random point on a map.
        #then, we choose the nearest node to that random point.
        #How can we know which node in the tree is the closest one to our random point?
        #By this function!
        dmin = self.distance(0, n) # n is number of nodes in tree
        nnear = 0
        for i in range(0,n):
            if self.distance(i,n) < dmin:
                dmin = self.distance(i,n)
                nnear = i
        return nnear # returns the indice of the nearest node in the graph

    def step(self,nnear,nrand):
        d = self.distance(nnear, nrand)
        dmax = self.dmax
        if d > dmax:
            u = dmax / d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px, py) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(py, px)
            (x, y) = (int(xnear + dmax * math.cos(theta)),
                      int(ynear + dmax * math.sin(theta)))
            self.remove_node(nrand)
            if abs(x - self.goal[0]) <= dmax and abs(y - self.goal[1]) <= dmax:
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.add_node(nrand, x, y)

    def bias(self, ngoal):
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        #print(self.obstacles_x)
        if self.isFree():
            nnear = self.nearest(n)
            self.step(nnear, n)
            self.connect(nnear, n)
        return self.x, self.y, self.parent

    def expand(self):
        n = self.number_of_nodes()
        x, y = self.generate_random_XY()
        self.add_node(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)
        return self.x, self.y, self.parent

    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    def getPathCoords(self):
        self.pathCoords = []
        self.pathCoordsTopic = []
        for node in self.path:
            x, y = [self.x[node], self.y[node]]
            self.pathCoords.append([x, y])
            self.pathCoordsTopic.append(x)
            self.pathCoordsTopic.append(y)

        return self.pathCoords

    def drawPath(self, path):
        for node in path:
            node_pg = (node[0] + self.map_area , node[1] + self.map_area)

            #plotting_list_global[self.vesselID].ax.scatter(node[0],node[1],color="red",s=self.nodeRad*3)
            #pygame.draw.circle(self.map, self.Red, node_pg, 3, 0)
        #plt.figure(plotting_list_global[self.vesselID].figure_window.number)
        #plt.show()
        #plt.pause(0.01)

    def drawMap(self):
        #Visualize start and goal points
        self.pg_start = (self.start[0] + self.map_area , self.start[1] + self.map_area)
        self.pg_goal = (self.goal[0] + self.map_area , self.goal[1] + self.map_area)
        
        #plotting_list_global[self.vesselID].ax.scatter(self.start[0],self.start[1],color="green",s=self.nodeRad+5)
        #plotting_list_global[self.vesselID].ax.scatter(self.goal[0],self.goal[1],color="red",s=self.nodeRad+10)
        
        #pygame.draw.circle(self.map,self.Green,self.pg_start,self.nodeRad+5,0)
        #pygame.draw.circle(self.map,self.Red,self.pg_goal,self.nodeRad+10,0)

        #Visualize obstacles
        vessel_poses = self.vessel_poses_xy.copy()
        while(len(vessel_poses)>0 ):
            vessel_pose_ = VesselPose()
            vessel_pose_=vessel_poses.pop(0)
            print(vessel_pose_)
            obstacle = (vessel_pose_.position.x + self.map_area,vessel_pose_.position.y + self.map_area)
            
            #Width is necessary to print the core black circle in the obstacle in pygame.
            #The black circle will be printed as well for the reference.
            
            forbidden_zone_radius = self.calc_forbidden_zone_radius(vessel_pose_.vessel_details.Length, vessel_pose_.vessel_details.Width)
            
            #plotting_list_global[self.vesselID].ax.scatter(vessel_pose_.position.x,vessel_pose_.position.y,color="grey",s=forbidden_zone_radius)
            #plotting_list_global[self.vesselID].ax.scatter(vessel_pose_.position.x,vessel_pose_.position.y,color="black",s=vessel_pose_.vessel_details.Length)
            
            #pygame.draw.circle(self.map,self.grey,obstacle,forbidden_zone_radius)
            #pygame.draw.circle(self.map,self.Black,obstacle,vessel_pose_.vessel_details.Length)

    def iterations(self):
            while (not self.path_to_goal()):
                time.sleep(0.005)
                elapsed=time.time()-self.t1
                self.t1=time.time()
                #raise exception if timeout
                if elapsed > 300:
                    print('timeout re-initiating the calculations')
                    raise Exception("Couldn't find any local path for such a long time.")

                if self.iteration % 10 == 0:
                    
                    X, Y, Parent = self.bias(self.goal)
                    X_pg,Y_pg = (X[-1] + self.map_area , Y[-1]+ self.map_area)

                    X_Parent_pg,Y_Parent_pg = (X[Parent[-1]] + self.map_area , Y[Parent[-1]]+ self.map_area)
                     
                    #plotting_list_global[self.vesselID].ax.scatter(X[-1],Y[-1],color="grey",s=self.nodeRad*2)

                    plot_x = [X[-1],X[Parent[-1]]]
                    plot_y = [Y[-1],Y[Parent[-1]]]
                    #plotting_list_global[self.vesselID].ax.plot(plot_x,plot_y,color="blue",linewidth=self.edgeThickness)

                    #pygame.draw.circle(self.map, self.grey, (X_pg,Y_pg), self.nodeRad*2, 0)
                    #pygame.draw.line(self.map, self.Blue, (X_pg,Y_pg), (X_Parent_pg,Y_Parent_pg),self.edgeThickness)

                else:
                    X, Y, Parent = self.expand()
                    X_pg,Y_pg = (X[-1] + self.map_area , Y[-1]+ self.map_area)

                    X_Parent_pg,Y_Parent_pg = (X[Parent[-1]] + self.map_area , Y[Parent[-1]]+ self.map_area)

                    #plotting_list_global[self.vesselID].ax.scatter(X[-1],Y[-1],color="grey",s=self.nodeRad*2)
                    plot_x = [X[-1],X[Parent[-1]]]
                    plot_y = [Y[-1],Y[Parent[-1]]]
                    #plotting_list_global[self.vesselID].ax.plot(plot_x,plot_y,color="blue",linewidth=self.edgeThickness)
                    
                    #pygame.draw.circle(self.map, self.grey, (X_pg,Y_pg), self.nodeRad*2, 0)
                    #pygame.draw.line(self.map, self.Blue, (X_pg,Y_pg), (X_Parent_pg,Y_Parent_pg),self.edgeThickness)


                self.iteration += 1
                
                #plt.figure(plotting_list_global[self.vesselID].figure_window.number)
                #plt.show()
                #plt.pause(0.01)
                
                #pygame.display.update()
                #pygame.display.update()

    def PathSmoothen(self):
        #smoothens the path via deleting unnecessary waypoints
        #a=0,b=2 indices of path
        #are there any obstacles in between a-b?
            #if there are not, delete the waypoint b-1
            #if there are, a=b,b=b+1
        #is b = [0,0] which means we ran out of waypoints to smoothen
            #if yes, end process
            #if not, continue

        a = 0
        b = 2
        smoothPath = self.pathCoords.copy()
        pathLen = len(smoothPath) 
        print(pathLen)
        #print(smoothPath )
        notpop = False
       
        # def crossObstacle(self, x1, x2, y1, y2):

        while(True):
            #print(a,b)
            if ((b >= len(smoothPath)) or (a >= len(smoothPath))):
                #which indicates that it's done
                #print("index")
                
                self.smoothPath = smoothPath
                print(smoothPath)
                self.prPathSmoothen()
                return smoothPath
            
            if(self.crossObstacle(smoothPath[a][0], smoothPath[b][0], smoothPath[a][1], smoothPath[b][1])):
                #returns true if there is obstacle
                
                print(smoothPath[a][0], smoothPath[b][0], smoothPath[a][1], smoothPath[b][1])
                    
                a = b-1
                b = b+1

                #you might wonder "why a = b-1 but not =b"
                #because the time we find out that there is an obstacle in between a and b,
                #it indicates that we can draw a simplified line. That's when we are supposed to do it and not go further.
                #However, we won't be able to draw a simplified line in between these two. Because there is an obstacle in between.
                #But we know that there is no obstacle in between a and b-1 since we checked that in the iteration one before. 
                #So what do we do? We draw a line in between a and b-1. not a and b. That's why we put a = b-1 in line 542
                
            else:
                #delete b-1
                smoothPath.pop(b-1)

    def prPathSmoothen(self):
        pygameSmoothPath = self.smoothPath.copy()
        i = 1
        for node in pygameSmoothPath:
            node_pg = node[0] + + self.map_area  , node[1] + self.map_area
            #pygame.draw.circle(self.map, self.Purple, node_pg, 4, 0)
            
           
        for i in range(0,len(pygameSmoothPath)-1):
            x1,y1 = int(pygameSmoothPath[i][0] + self.map_area) , int(pygameSmoothPath[i][1]+ self.map_area )
            x2,y2 = int(pygameSmoothPath[i+1][0]+ self.map_area ), int(pygameSmoothPath[i+1][1]+ self.map_area)

            plot_x = [pygameSmoothPath[i][0],pygameSmoothPath[i+1][0]]
            plot_y = [pygameSmoothPath[i][1],pygameSmoothPath[i+1][1]]
            
            #plotting_list_global[self.vesselID].ax.plot(plot_x,plot_y,color="purple",linewidth=self.edgeThickness*2)
            #pygame.draw.line(self.map, self.Purple, (x1,y1), (x2,y2),self.edgeThickness*2)

        #plt.figure(plotting_list_global[self.vesselID].figure_window.number)
        #plt.show()
        #plt.pause(0.01)

class Local_Path_Planner(threading.Thread):
    def __init__(self, VesselID_):
        threading.Thread.__init__(self)

        #This class will be generated by main function and will perform the processes mentioned at the top description comment and flowchart.
        #This class will be called and one object will be called for each vessel.
        
        self.path = "~/vrx_ws/src/vrx/multiple_vessels"
        self.full_path = os.path.expanduser(self.path)
        
        with open(self.full_path+'/json_files/config.json','r') as f:    
            self.config_data = json.load(f)

        self.vesselID = VesselID_

        self.rrt_chosen = self.config_data["local_path_planning"][0]["RRT"][0]["chosen"]
        self.reaching_to_local_goal_treshold = self.config_data["local_path_planning"][0]["RRT"][0]["reaching_to_local_goal_treshold"]
        #Load our own vessel's details. Because it's necessary to calculate the ship domain's radius.
        vessel_details_param_txt = "/vessel" + str(self.vesselID)  + "_details"
        self.vessel_details = rospy.get_param(vessel_details_param_txt)
        
        self.global_trajectory_txt = '/vessel'+str(self.vesselID)+"_Global_Trajectory"
        self.global_trajectory = rospy.get_param(self.global_trajectory_txt)

        self.last_reached_global_wp_txt = '/vessel'+str(self.vesselID)+"_Last_Reached_Global_Wp"
        self.last_reached_global_wp = rospy.get_param(self.last_reached_global_wp_txt)

        self.current_lat = ""
        self.current_lon = ""
        
        self.roll = ""
        self.pitch = ""
        self.yaw = ""

        self.quaternion_x = ""
        self.quaternion_y = ""
        self.quaternion_z = ""
        self.quaternion_w = ""

        self.perception_list = []
        #This list contains the VesselIDs numbers of vessels in the sensing range of this vessel.
        self.vessel_poses = []
        self.vesselsInShipDomain = Perception() 

        #Subscribe to ShipDomain topic
        sim_Perception_topic_txt =  "vessel"+str(self.vesselID) + "/vessels_in_ship_domain"
        rospy.Subscriber(sim_Perception_topic_txt,Perception, self.CallbackShipDomain)

        #Subscribe to is_on_global_path topic
        sim_Perception_topic_txt =  "vessel"+str(self.vesselID) + "/is_on_global_path"
        rospy.Subscriber(sim_Perception_topic_txt,Int16, self.CallbackIsOnGlobalPath)

        #Sub to generate local path topic
        sim_generateLocalPath_topic_txt =  "vessel"+str(self.vesselID) + "/local_path_args"
        rospy.Subscriber(sim_generateLocalPath_topic_txt,Float64MultiArray, self.CallbackGenerateLocalPath)

        sub_topic_imu_txt = "vessel"+str(self.vesselID) +"/vessel"+str(self.vesselID)+"/sensors"+"/imu/imu/data"
        rospy.Subscriber(sub_topic_imu_txt,Imu, self.CallbackImu)

        #0:not calculating any local path
        #1:caltulating a local path 
        #2:calculated the local path

        #init the local_path publisher
        pub_local_path_txt = "vessel"+str(self.vesselID) + "/local_path"
        self.pub_local_path= rospy.Publisher(pub_local_path_txt,Float64MultiArray,queue_size=10)
        
        self.there_is_a_new_encounter = False
        self.Vessels_In_Ship_Domain_Interfere_With_Local_Path = False

        self.cnt = 0
        print(str(self.vesselID)," vessel local path planner started")

        self.vessel_is_on_global_path = 1

        self.localPath = []
        self.generate_local_path_flag =False

        self.generate_local_path_args = []

        self.local_path_args = []

        self.local_path_args_minusOne = []

    def run(self):
        while not rospy.is_shutdown():
            #try:
                rate.sleep()
                self.Check_Generate_Local_Path_Topic()
                
                if self.generate_local_path_flag == True:
                    print("Generate local path for vessel",str(self.vesselID))

                    self.Generate_Local_Path()
                    self.vessel_is_on_global_path =0
                    self.Publish_Local_Path()
                    print("published local path")

                    rospy.sleep(2)

                    while self.is_on_global_path == 0:
                        self.Publish_Local_Path()
                        # While the path tracker is tracking the local path and switch mechanism checks on whether or not local path is interrupted, keep publishing the local path
                        rate.sleep() 
                    
                    self.generate_local_path_flag = False

                elif self.there_is_a_new_encounter == False:
                    rate.sleep()

            #except Exception as e:
                #rospy.logerr(e)    
                rate.sleep()

    def Check_Generate_Local_Path_Topic(self):
        self.local_path_args = self.generate_local_path_args

        if (len(self.local_path_args) == 4) and (self.local_path_args_minusOne != self.local_path_args):
            self.generate_local_path_flag = True
        else:
            self.generate_local_path_flag = False
        self.local_path_args_minusOne = self.local_path_args
    
    def Check_if_Vessels_In_Ship_Domain_Interfere_With_Local_Path(self):
        print("Let's check any vessel interrupts the local path for vessel"+str(self.vesselID))
        pass

    def Generate_Local_Path(self):
        self.current_lat = self.generate_local_path_args[0]
        self.current_lon = self.generate_local_path_args[1]
        goalLat = self.generate_local_path_args[2]
        goalLon = self.generate_local_path_args[3]

        start =(self.current_lat ,self.current_lon)
        goal = (goalLat,goalLon)

        vessel_poses = []
        vessel_poses = self.vessels_in_ship_domain.vessel_poses

        roll,pitch,self.yaw = euler_from_quaternion([self.quaternion_x, self.quaternion_y, self.quaternion_z, self.quaternion_w])
        self.RRTobject = RRT(start,goal,vessel_poses,self.vesselID,self.yaw)
        self.localpath = self.RRTobject.smoothPathLatLon

        print("self.RRTobject.smoothPathLatLon",self.RRTobject.smoothPathLatLon)
        self.vessel_is_on_global_path = 0

    def Publish_Local_Path(self):
        msgToPublish = Float64MultiArray()
        msgToPublish.data = self.localpath
        self.pub_local_path.publish(msgToPublish)

    def CallbackIsOnGlobalPath(self,msg_):
        self.is_on_global_path = msg_.data

    def CallbackShipDomain(self,perception_msg_):
        self.vessels_in_ship_domain = perception_msg_

    def CallbackGenerateLocalPath(self,msg_):
        self.generate_local_path_args = msg_.data
        
    def CallbackImu(self,imu_msg):    
        self.quaternion_x  = imu_msg.orientation.x
        self.quaternion_y = imu_msg.orientation.y
        self.quaternion_z = imu_msg.orientation.z
        self.quaternion_w = imu_msg.orientation.w

class Plotting():
    #This class is defined to handle local path plotting for each vessel.
    #For each vessel in the simulation environment, this class' object is defined. 
    #In this class, ax variable is the one that we will plot and/or scatter on. 
    #In order to clean the plotting, we will simply use ax.cla() function. 
    # This will be done in order to plot new local paths on the same figure window.

    #Access to this objects from plotting_list_global list.
    def __init__(self,VesselID_):
        self.figure_window,self.ax  = plt.subplots()
        self.VesselID = VesselID_
        self.ax.set_title('Vessel '+str(self.VesselID) + " Local Path Graph")
        plt.ion()
        plt.show(block=False)
        plt.pause(0.001) 

if __name__ == '__main__':
    rospy.init_node('multi_vessel_local_path_planner', anonymous=True)
    hz = 10
    rate = rospy.Rate(hz)

    vessel_objects = []
    vessel_count = int(rospy.get_param("/vessel_count"))#get the number of how many Vessels do we spawn in this run
    print("Number of Vessels: ", str(vessel_count))

    for i in range(vessel_count):
        vessel_objects.append(Local_Path_Planner(i+1))

    for obj in vessel_objects:
        obj.start()