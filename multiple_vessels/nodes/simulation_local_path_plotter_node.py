#!/usr/bin/env python
"""
Simulation Local Path Plotter Node
Plots the local paths and virtual obstacles.

Also the logger node will visualize the current positions and such.
"""


from os import kill
import rospy
import time
import math
from rospy.core import rospydebug
from std_msgs.msg import Int16
from std_msgs.msg import Float64MultiArray
import json
import random
import os
import csv
from geopy import distance
from datetime import date
from multivessel_msgs.msg import VesselPose,Perception

import nest_asyncio
nest_asyncio.apply()

#import matplotlib
#matplotlib.use('Agg')
#matplotlib.use("GTK3Agg")

import matplotlib.pyplot as plt
from mpl_toolkits.basemap import Basemap
from matplotlib.animation import FuncAnimation

from tf.transformations import euler_from_quaternion

class Vessel_is_on_global_path_Subscriber():
    def __init__(self, vesselID_):

        self.is_on_global_path = 1
        
        self.vesselID = vesselID_

        #subscribe to sensors
        is_on_global_path_txt = "vessel"+str(self.vesselID) + "/is_on_global_path"
        rospy.Subscriber(is_on_global_path_txt,Int16, self.Callback_is_on_global_path)

        #Subscribe to local path topic.
        sim_local_path_topic_txt =  "vessel"+str(self.vesselID) + "/local_path"
        rospy.Subscriber(sim_local_path_topic_txt,Float64MultiArray, self.CallbackLocalPath)

        #Subscribe to ShipDomain topic
        sim_Perception_topic_txt =  "vessel"+str(self.vesselID) + "/vessels_in_ship_domain"
        rospy.Subscriber(sim_Perception_topic_txt,Perception, self.CallbackShipDomain)

        self.cnt = 0
        

    def Callback_is_on_global_path(self,msg_):
        self.is_on_global_path = msg_.data

    def CallbackLocalPath(self,local_path_msg_):
        self.localPath  = local_path_msg_.data

    def CallbackShipDomain(self,perception_msg_):
        self.vessels_in_ship_domain = perception_msg_



        
         


class Vessel_Details():
    def __init__(self):
        self.vessel_id =0
        self.VesselName = ""
        self.CallSign = ""
        self.Cargo = 0
        self.Draft = 0
        self.IMO = ""
        self.Length = 0
        self.Width = 0
        self.MMSI = ""
        self.TransceiverClass = ""
        self.VesselType = 0


    """
                print("vessel2_details",vessel2_details)
            vessel_details.vessel_details.vessel_id = self.vessel_indx
            vessel_details.vessel_details.VesselName.data = vessel2_details["VesselName"]
            vessel_details.vessel_details.CallSign.data = vessel2_details["CallSign"]
            vessel_details.vessel_details.Cargo = vessel2_details["Cargo"]
            vessel_details.vessel_details.Draft = vessel2_details["Draft"]
            vessel_details.vessel_details.IMO.data = vessel2_details["IMO"]
            vessel_details.vessel_details.Length = vessel2_details["Length"]
            vessel_details.vessel_details.MMSI.data = vessel2_details["MMSI"]
            vessel_details.vessel_details.TransceiverClass.data = vessel2_details["TransceiverClass"]
            vessel_details.vessel_details.VesselType = vessel2_details["VesselType"]
            vessel_details.vessel_details.Width = vessel2_details["Width"]"""     

class Local_plotter():
    def __init__(self,VesselID_):

        
        self.vesselID = VesselID_

        #Read the JSON config file
        self.path = "~/vrx_ws/src/vrx/multiple_vessels"
        self.full_path = os.path.expanduser(self.path)
        
        with open(self.full_path+'/json_files/config.json','r') as f:    
            self.config_data = json.load(f)

        with open(self.full_path+'/json_files/Json_Global_Waypoints.json','r') as f:    
            self.global_wp_data = json.load(f)

        #self.json_TopRightLat = self.global_wp_data["TopRightLat"]
        #self.json_TopRightLon = self.global_wp_data["TopRightLon"]
        #self.json_BotLeftLat = self.global_wp_data["BotLeftLat"]
        #self.json_BotLeftLon = self.global_wp_data["BotLeftLon"]
        #print(self.json_TopRightLat,"self.json_TopRightLat")
        
        self.scatter_distance_treshold = self.config_data["local_path_plotter_logger"][0]["scatter_distance_treshold"]
        self.logging_frequency = self.config_data["local_path_plotter_logger"][0]["logging_frequency"]
        self.plotting_interval = self.config_data["local_path_plotter_logger"][0]["plotting_interval"]
        self.plotting_available = self.config_data["local_path_plotter_logger"][0]["plotting_available"]
        self.logging_available = self.config_data["local_path_plotter_logger"][0]["logging_available"]

        
        
        self.hz = self.logging_frequency
        self.rate = rospy.Rate(hz)
    
        self.vessel_objects = []
        self.vessel_count = int(rospy.get_param("/vessel_count"))#get the number of how many Vessels do we spawn in this run
        print("Number of Vessels: ", str(self.vessel_count))
    
        self.Vessel_objects()
        #self.Initialize_Threads()
        self.Vessel_Is_On_Global_Path_List_Generator()
        time.sleep(0.2)

        
        self.local_plot_list = []
        self.local_plot_list.append([]) #This is done to eliminate indexing problems. This is for vessel0 but it's not a plot object but an empty list.

            
    
        self.gps_list = []
        self.imu_list = []
        self.vessel_details_list = []

        self.rows = [] # This list will store the CSV file rows. 
        #If the logger is freshly started, this list will include every row that are appended to the csv file.
        #If the logger node is started and resumes on an older file, it loads the data first and appends to this list while this ROS node works.

        self.trail_scatter_list = [] #This list store the previous positions of the vessels that are seperated enough in between to scatter.
        self.last_scatter_pos = []
        self.local_path_plot_list = []
        self.square_scatter_list = []
        self.cross_scatter_list = []
        #self.local_plot_list = []
        self.last_plotted_local_path_list = []
        self.local_plot_flag_list = []
        self.vessel_poses_xy = []
        #In order to not scatter too many unnecessary points on the map and overwhelm the matplotlib, we decided to use this list to store the previous location information.
        #This list is multi-dimensional. First dimension of this list indicates the Vessel ID number. For example, self.trail_scatter_list[1] indicates Vessel1's previous points.
        #The previous positions will be put consecutively to this list while this ROS node operates. 
        i = 0
        while i<= self.vessel_count:
            self.trail_scatter_list.append([])#I am doing this to put one dimension for every vessel in the simulation environment. 
            self.last_scatter_pos.append([])
            self.local_path_plot_list.append([])
            self.square_scatter_list.append([])
            self.cross_scatter_list.append([])
            #self.local_plot_list.append([])
            self.last_plotted_local_path_list.append([])
            self.vessel_poses_xy.append([])
            self.local_plot_flag_list.append([False,False])# Local path Flags for each vessel
            #If this flag is set to False, we plot a cross on the map in order to mark the point of switching to local path.
            #And we set this flag back to True in order to not plot crosses all over the trail of the vessel.
            #Same goes for switching to the global path.


            #I start from 0 because there might be a vessel0 in the future of development. 
            # And even if there would never be one, defining an empty list for vessel0 makes indexing less complicated.

            #In self.local_path_plot_list, there are indexes defined for each vessel by appending [].
            #After that, each local path will be indexed seperately. For example, self.local_path_plot_list[1][2] will have the Vessel1 local_path2 in it. 
            #For each local path that generated, we will append to self.local_path_plot_list[self.vessel_index]
            #Local paths will be stored by Lat and Lon. For example, self.local_path_plot_list[1][2][0] will have the Vessel1 local_path2 Latitude points. 
            #self.local_path_plot_list[1][2][1] will have the Vessel1 local_path2 Longitude points. 
            i +=1

        self.localPath = []

        self.current_local_path = []

        self.local_path_is_generated = False

        

        local_path_planning_parameters = rospy.get_param("/local_path_planning_parameters")
        self.vessel_forbidden_zone_coefficient_length = local_path_planning_parameters["vessel_forbidden_zone_coefficient_length"]
        self.vessel_forbidden_zone_coefficient_width = local_path_planning_parameters["vessel_forbidden_zone_coefficient_width"]
        self.vessel_forbidden_zone_coefficient_draft = local_path_planning_parameters["vessel_forbidden_zone_coefficient_draft"]
        self.vessel_forbidden_zone_radius_equation = local_path_planning_parameters["vessel_forbidden_zone_radius_equation"]
        
        

        #self.cross_placed_for_switching_to_local_path_flag = False
        #self.square_placed_for_switching_to_global_path_flag = False
        

        self.cross_scatter_list_lat = []
        self.cross_scatter_list_lon = []

        self.square_scatter_list_lat = []
        self.square_scatter_list_lon = []

        self.vessel_local_waypoints_lat = []
        self.vessel_local_waypoints_lon = []
        
        #self.scatter_distance_treshold = 30 # Distance treshold for scattering in meters. 
        
        sub_topic_gps_txt = "simulation/gps_list"
        rospy.Subscriber(sub_topic_gps_txt,Float64MultiArray, self.CallbackGpsList)

        sub_topic_imu_txt = "simulation/imu_list"
        rospy.Subscriber(sub_topic_imu_txt,Float64MultiArray, self.CallbackImuList)

        self.Get_Vessel_Detail_Params()
        rate.sleep()
        self.Init_Plotting()

        self.figure_window,self.ax  = plt.subplots()
        self.VesselID = VesselID_
        self.ax.set_title('Vessel '+str(self.VesselID) + " Local Path Graph")
        plt.ion()
        plt.show(block=False)
        plt.pause(0.001)


        plt.gcf().canvas.start_event_loop(0.001)
        #self.anim = FuncAnimation(self.figure_window, self.Plot_Iteration, interval=100, blit=True,repeat=True)#repeat = False # self.fig # plt.gcf()

        


        


        #rospy.spin()
        # 
        # 

        """
        if self.logging_available == True:
            self.CSV_File_Init()
            rate.sleep()

        if self.plotting_available == True:
            plt.ion()
            plt.show()
            self.Init_Plotting()
            print("after init plotting")
            self.anim = FuncAnimation(plt.gcf(), self.Plot_Iteration, interval=self.plotting_interval, blit=True)#repeat = False # self.fig # plt.gcf()
            #TODO: Assign interval from json
            plt.show(block=False)
            plt.gcf().canvas.start_event_loop(0.001)
            #plt.pause(0.01) 

        if self.logging_available == True:
            print("After anim is called")
            self.Log_To_CSV_File_Loop()

        if (self.logging_available == False) and (self.plotting_available == False):
            print("Both logging and plotting is set to not available. Shutting down.")
            exit()
        """

    def CallbackGpsList(self,gpsList_msg):
        self.gps_list = gpsList_msg.data
        #print(self.gps_list)

    def CallbackImuList(self,gpsList_msg):
        self.imu_list = gpsList_msg.data

    def Init_Plotting(self):
        self.plots = []
        #Instead of dividing into 3 lists, gather all the iterative plots into one list called iterative_self.vessel_tracked_path_plots
        #This iterative plots will be emptied in every iteration in order to get refilled from live data in every iteration


        """ 
        self.local_path_plots = []
        self.vessel_location_plots = []
        self.vessel_tracked_path_plots = []
        """
        
        self.vesselColors =[]
        #self.fig = plt.figure()

        #assign random colors to every vessel in the simulation environment. 
        i = 0
        while i<=self.vessel_count:
            
            r = random.random()
            b = random.random()
            g = random.random()
            color = (r, g, b)
            self.vesselColors.append(color)
            i = i+1

        print("self.vesselColors",self.vesselColors)

        #plt.title("Logger Node Plotting Initialized")
        #plt.show(block=False)#block=False
        
        #
            
            

        return self.plots
   
    
    def Plot_Iteration(self):
        #The FuncAnimation function uses this animate function to iterate. 
        #In this function, we plot the:
        ##Global Paths of the vessels with plot function to plot the global paths with lines.
        ##Vessels' position and orientations with quiver function.
        ##Ship domains of every vessel with scatter. 
        ##Local paths for vessels if there is any.  
        self.plots = []

        print("In plot iteration")
        #plt.cla()
        self.vessel_index = self.vesselID#I will make this parametric depending on the config file. Right now I start from 1 because there is no vessel0

        #for every vessel in the simulation environment, do the following:
        
        #Plot the global path for the vessel
        #Plot global paths
        self.vessel_global_waypoints_lat = []
        self.vessel_global_waypoints_lon = []
        

        self.AIS_data = self.global_wp_data["AIS_Data"]
        for j in range(len(self.AIS_data[self.vessel_index-1])):#-1 because the JSON file starts from 0 not 1
            #Goes through every AIS data from start to finish for a vessel.
            one_ais_data = self.AIS_data[self.vessel_index-1][j]
            self.vessel_global_waypoints_lat.append(one_ais_data["LAT"])
            self.vessel_global_waypoints_lon.append(one_ais_data["LON"])
            
            #below is not necessary
            #self.vessel_global_waypoints.append(one_ais_data["BaseDateTime"])
            #self.vessel_global_waypoints.append(one_ais_data["Velocity_to_Next_Waypoint"])
            #self.vessel_global_waypoints.append(one_ais_data["Heading_to_Next_Waypoint"])

        
        while self.gps_list == []:
            rate.sleep()
        #print("self.vessel_index*",self.vessel_index)
        #print("self.gps_list",self.gps_list)
        LAT = self.gps_list[self.vessel_index*2 - 2]
        LON = self.gps_list[self.vessel_index*2 - 1]
        
        self.origin_lat = LAT
        self.origin_lon = LON
        
        MP_Mode = self.vessel_objects[self.vessel_index - 1].is_on_global_path

        self.cross_placed_for_switching_to_local_path_flag = self.local_plot_flag_list[self.vessel_index][0]
        self.square_placed_for_switching_to_global_path_flag = self.local_plot_flag_list[self.vessel_index][1]

        #For each iteration, we need to check the local path and compare it with the local path from earlier. 

        if MP_Mode == 0:
            #If the path planning is local path mode. Switch both the flags below to false in order to put the appropriate marker.
            self.local_plot_flag_list[self.vessel_index][1] = False
            self.ax.cla()

            if self.cross_placed_for_switching_to_local_path_flag == False:
                #In here, we clear the figure. Because we want to delete the local path from earlier. 
                #self.cross_scatter_list[self.vessel_index].append([LAT,LON,Heading])
                self.local_plot_flag_list[self.vessel_index][0] = True

            

            self.local_wp_index = 2
            self.local_path = self.vessel_objects[self.vessel_index - 1].localPath
            
            print("local path", self.local_path)
            new_local_path_x = []
            new_local_path_y = []
            new_local_path = []

            self.origin_lat = self.local_path[0]
            self.origin_lon = self.local_path[1]

            while self.local_wp_index <= len(self.local_path)/2:
                #In self.local_path_plot_list, there are indexes defined for each vessel by appending [].
                #After that, each local path will be indexed seperately. For example, self.local_path_plot_list[1][2] will have the Vessel1 local_path2 in it. 
                #For each local path that generated, we will append to self.local_path_plot_list[self.vessel_index]
                #Local paths will be stored by Lat and Lon. For example, self.local_path_plot_list[1][2][0] will have the Vessel1 local_path2 Latitude points. 
                #self.local_path_plot_list[1][2][1] will have the Vessel1 local_path2 Longitude points. 

                waypointLat = self.local_path[(2*(self.local_wp_index))-4]
                waypointLon = self.local_path[(2*(self.local_wp_index))-3]

                nextWaypointLat = self.local_path[(2*(self.local_wp_index))-2]
                nextWaypointLon = self.local_path[(2*(self.local_wp_index))-1]

                wp_x,wp_y = self.get_local_coord( waypointLat,waypointLon )
                wp_x_plus1,wp_y_plus1 = self.get_local_coord( nextWaypointLat,nextWaypointLon )

                new_local_path_x.append(wp_x)
                new_local_path_x.append(wp_x_plus1)
                new_local_path_y.append(wp_y)
                new_local_path_y.append(wp_y_plus1)
                #Convert to XY

                #local_path_plotting,= self.m.plot(self.vessel_local_waypoints_lon,self.vessel_local_waypoints_lat,latlon = True,color=self.vesselColors[self.vessel_index],linestyle = '-',label = "Vessel "+str(self.vessel_index)+" Local Path")

                self.local_wp_index +=2

            new_local_path.append(new_local_path_x)    
            new_local_path.append(new_local_path_y)
            self.local_path_plot_list[self.vessel_index]=new_local_path

            print("self.local_path_plot_list",self.local_path_plot_list)
            print(" self.local_path_plot_list[self.vessel_index]", self.local_path_plot_list[self.vessel_index])

            self.last_plotted_local_path_list[self.vessel_index] = self.local_path
            
            self.ax.plot(self.local_path_plot_list[self.vessel_index][0],self.local_path_plot_list[self.vessel_index][1],color=self.vesselColors[self.vessel_index],linestyle = '-',label = "Vessel "+str(self.vessel_index)+" Local Path")
            print("plotting local path for vessel",self.vessel_index )

            #plot own vessel, other vessels in the ship domain and virtual obstacles
            #Plot the Vessel Positions with quivers.
            LAT = self.gps_list[(self.vessel_index)*2 - 2]
            LON = self.gps_list[((self.vessel_index)*2) - 1]

            self.last_scatter_pos[self.vessel_index] = [LAT,LON,0]
            X_pos,Y_pos = self.get_local_coord(LAT,LON)

            #while self.imu_list== []:
            #    rate.sleep()

            self.orientation_x = self.imu_list[self.vessel_index*4 -4]
            self.orientation_y = self.imu_list[self.vessel_index*4 -3]
            self.orientation_z = self.imu_list[self.vessel_index*4 -2]
            self.orientation_w = self.imu_list[self.vessel_index*4 -1]
            roll,pitch,Heading = euler_from_quaternion([self.orientation_x, self.orientation_y, self.orientation_z, self.orientation_w])

            X_direct = math.cos(Heading)*10
            Y_direct = math.sin(Heading)*10 
            vessel_location = self.ax.quiver(X_pos,Y_pos,X_direct,Y_direct,color=self.vesselColors[self.vessel_index],label = "Vessel "+str(self.vessel_index))

            vessel_scatter_list = self.trail_scatter_list[self.vessel_index]

            for i in range(len(vessel_scatter_list)):
                #Scatter from self.trail_scatter_list
                lat = float(self.trail_scatter_list[self.vessel_index][i][0])
                lon = float(self.trail_scatter_list[self.vessel_index][i][1])
                x,y = self.get_local_coord( lat,lon)
                self.ax.scatter(x,y,color=self.vesselColors[self.vessel_index],alpha=0.4)
            
            #Plot trail scatter to indicate the positions that a vessel have been on before.
            lat_last_scatter = self.last_scatter_pos[self.vessel_index][0]
            lon_last_scatter = self.last_scatter_pos[self.vessel_index][1]
            last_scatter_position = (lat_last_scatter,lon_last_scatter)
            current_pos = (LAT,LON)

            distance_in_between = distance.distance(current_pos, last_scatter_position).meters

            if distance_in_between > self.scatter_distance_treshold:
                self.trail_scatter_list[self.vessel_index].append([LAT,LON,Heading])
                self.last_scatter_pos[self.vessel_index] = [LAT,LON,Heading]

            #Plot other vessels inside the ship domain and the virtual obstacles that they generate.
            #TODO:
            self.Convert_Vessel_Positions_to_XY(self.vessel_index)
            self.Generate_COLREGS_Virtual_Obstacles(self.vessel_index)

            


        
        if MP_Mode == 1:
            #If the vessel has switched to global path, that means we completed the local path. Make assignments to the flags accordingly. 
            self.cross_placed_for_switching_to_local_path_flag = False
            self.local_plot_flag_list[self.vessel_index][0] = False
            self.trail_scatter_list[self.vessel_index] = []
            
            if self.square_placed_for_switching_to_global_path_flag == False:
                
                self.local_plot_flag_list[self.vessel_index][1] = True
            
        
        if MP_Mode == 2:
            #Calculate the positions of the virtual obstacles. Plot Own vessel, other vessels in the ship domain and the virtual obstacles of these vessels. 
            self.trail_scatter_list[self.vessel_index] = []
            self.Convert_Vessel_Positions_to_XY(self.vessel_index)
            self.Generate_COLREGS_Virtual_Obstacles(self.vessel_index)
            pass

        
        print("plot iteration complete for vessel",self.vesselID)
        #plt.show(block=False)#block=False
        #plt.pause(0.01) 
        plt.gcf().canvas.draw_idle()
        
        #plt.gcf().canvas.start_event_loop(0.001)
        #plt.gcf().canvas.start_event_loop(0.001)

        #If it's unclear that why I use plt.gcf().canvas.draw_idle() instead of plt.pause(0.01) or any other commented line here, please refer to:
        #https://stackoverflow.com/questions/45729092/make-interactive-matplotlib-window-not-pop-to-front-on-each-update-windows-7/45734500#45734500
        
        return self.plots

        


    
    def Convert_Vessel_Positions_to_XY(self,vesselID):
        #This function converts every position variable to XY 
        #Takes the Lat-Lon values from the self.vessel_poses list
        #and puts the XY conversions in the self.vessel_poses_xy list
        """
        self.vessel_poses = vessel_poses_
        self.vessel_poses is in type of a list of Vessel_Pose

        self.vessel_poses_xy = []
        same as the vessel_poses_xy will be."""
        self.vessel_index = vesselID
        self.vessel_poses_xy = []
        #Make sure to reset this list in order to not add the same vessel positions over and over again.s
        self.vessel_poses = self.vessel_objects[self.vessel_index - 1].vessels_in_ship_domain.vessel_poses

        
        #print(self.vessel_poses)
        
        for i in range(len(self.vessel_poses)):
            #print(i,"in loop")
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

            #Plot vessel with quiver.
            roll,pitch,Heading = euler_from_quaternion([vessel_pose_xy.orientation.x, vessel_pose_xy.orientation.y, vessel_pose_xy.orientation.z, vessel_pose_xy.orientation.w])

            X_direct = math.cos(Heading)*10
            Y_direct = math.sin(Heading)*10 
            vessel_location = self.ax.quiver(pos_x,pos_y,X_direct,Y_direct,color=self.vesselColors[vessel_pose_xy.vessel_id],label = "Vessel "+str(vessel_pose_xy.vessel_id))

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


    def Generate_COLREGS_Virtual_Obstacles(self,vesselID):
        #This function reads the map size from 'vessel'+str(self.vesselIDl)+"_Local_Map_Dimensions ROS parameter 
        #And with using the self.vessel_poses_xy list, this function generates the COLREGS virtual obstacles and appends to the self.vessel_poses_xy list.
        #Therefore, local path will be checked if it got interrupted by both from vessel poses and virtual obstacles.
        self.vesselID = vesselID
        vessel_local_map_param_txt = "/vessel" + str(self.vesselID)  + "_Local_Map_Dimensions"
        self.local_map_details = []
        self.local_map_details = rospy.get_param(vessel_local_map_param_txt)

        self.map_X_min = self.local_map_details[0]
        self.map_X_max = self.local_map_details[1]
        self.map_Y_min = self.local_map_details[2]
        self.map_Y_max = self.local_map_details[3]

        self.virtual_obstacles_xy = []

        

        iteration_count = 0
        vessel_poses = self.vessel_poses_xy.copy()
        while(len(vessel_poses)>0 ):
            vessel_pose_=vessel_poses.pop(0)
            """
            relative_angle = ( math.pi/2 )- math.atan((vessel_pose_.position.y)/(vessel_pose_.position.x))
            colregs_angle = relative_angle - self.own_vessel_yaw"""

            roll,pitch,self.own_vessel_yaw = euler_from_quaternion([self.orientation_x, self.orientation_y, self.orientation_z, self.orientation_w])


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
                #TODO: burada Place virtual obstacles to in front of the vessel. hesaplamalarını yap 13 stand-on daki gibi.
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
                    #scatter virtual_obs_xy.position.x and virtual_obs_xy.position.y.
                    
                    self.ax.scatter(virtual_obs_xy.position.x,virtual_obs_xy.position.y,s=forbidden_zone_radius,color=self.vesselColors[vessel_pose_.vessel_id],alpha=0.4)

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
                    #scatter virtual_obs_xy.position.x and virtual_obs_xy.position.y.
                    self.ax.scatter(virtual_obs_xy.position.x,virtual_obs_xy.position.y,s=forbidden_zone_radius,color=self.vesselColors[vessel_pose_.vessel_id],alpha=0.4)


                    self.virtual_obs_x_minus_one = virtual_obs_x  
                    self.virtual_obs_y_minus_one = virtual_obs_y

            iteration_count +=1



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
    
    def Vessel_objects(self):
        self.vessel_objects = []
        #i =1
        print(self.vessel_count)
        
        for i in range(self.vessel_count):
            self.vessel_objects.append(Vessel_is_on_global_path_Subscriber(i+1))

    def Vessel_Is_On_Global_Path_List_Generator(self):
        self.vessel_is_on_global_path_list = []

        #print("Vessel objects:",int(len(self.vessel_objects)))
        for i in range(int(len(self.vessel_objects))):
            self.vessel_is_on_global_path_list.append( self.vessel_objects[i].is_on_global_path )

        print("self./VesselX/Is_On_Global_Path_list",self.vessel_is_on_global_path_list)


    def Get_Vessel_Detail_Params(self):
        #This function gets the ROS parameters about the vessel details in the simulation.
        #Log in the vessel details to the message.
        self.vessel_indx = 1 # TODO:This variable will be assigned from the config.JSON file later.
        #Because I want to include the vessel0 in the simulation after I'm done with everything else.
        self.vessel_details_list = []
        
        
        vessel_details = Vessel_Details()
        self.vessel_details_list.append(vessel_details)#Right now I append an empty placeholder data just to not mess up the indexing.
        #Because vessels start with number 1 right now and I don't have a vessel0 at the moment.

        while  self.vessel_indx <= self.vessel_count:
        
            vessel_details = Vessel_Details()
            #print("vessel_details",vessel_details)
            details_param_txt = "/vessel"+str(self.vessel_indx)+"_details"
            vessel2_details = rospy.get_param(details_param_txt)
            #print("vessel2_details",vessel2_details)
            vessel_details.vessel_id = self.vessel_indx
            vessel_details.VesselName = vessel2_details["VesselName"]
            vessel_details.CallSign = vessel2_details["CallSign"]
            vessel_details.Cargo = vessel2_details["Cargo"]
            vessel_details.Draft = vessel2_details["Draft"]
            vessel_details.IMO = vessel2_details["IMO"]
            vessel_details.Length = vessel2_details["Length"]
            vessel_details.MMSI = vessel2_details["MMSI"]
            vessel_details.TransceiverClass = vessel2_details["TransceiverClass"]
            vessel_details.VesselType = vessel2_details["VesselType"]
            vessel_details.Width = vessel2_details["Width"]

            self.vessel_details_list.append(vessel_details)
            self.vessel_indx +=1


    def CSV_File_Init(self):
        #This function initializes the CSV file to be written.
        self.filename = 'logged_sim_data.csv'
        
        #We want to give the option of resuming to the current logging data or freshly starting to a new logging.
        
        while True:
            print('Please enter 0 for resuming or 1 for a fresh start.')
            resume_or_fresh_start_input = input()
            self.resume_or_fresh_start = int(resume_or_fresh_start_input)

            if self.resume_or_fresh_start == 1:
                with open(self.full_path+"/simulation_log_files/"+self.filename, mode='w') as csv_file:
                    self.CSVfieldnames = ['MMSI','BaseDateTime','LAT','LON','SOG','COG','Heading','VesselName','IMO','CallSign','VesselType','MP_Mode','Length','Width','Draft','Cargo','TransceiverClass','VesselID']#TODO: This field names are referenced from the real world AIS data obtained from marinecadastre.gov
                    self.writer = csv.writer(csv_file)
                    self.writer.writerow(self.CSVfieldnames)
                    break

            elif self.resume_or_fresh_start == 0:
                #don't need to do anything for this option.
                break
            else:
                print("Invalid input")





        

    def CSV_File_Iteration(self):
        #This function handles every iteration of logging. 
        # In every iteration, this function logs every vessel's pose and path planning mode for one time.

        self.vessel_indx = 1 # TODO:This variable will be assigned from the config.JSON file later.
        #Because I want to include the vessel0 in the simulation after I'm done with everything else.
        #print("self.vessel_count",self.vessel_count)

        while  self.vessel_indx <= self.vessel_count:
            #print("self.vessel_indx",self.vessel_indx)
            
            #fieldnames = ['MMSI','BaseDateTime','LAT','LON','SOG','COG','Heading','VesselName','IMO','CallSign','VesselType','MP_Mode','Length','Width','Draft','Cargo','TransceiverClass']#TODO: This field names are referenced from the real world AIS data obtained from marinecadastre.gov
            
            #print(self.vessel_details_list,"self.vessel_details_list")
            MMSI = self.vessel_details_list[self.vessel_indx].MMSI
            
            dateNow = date.today()#Base Date Time will be based on the time of the computer that the simulation is being run.
            t = time.localtime()
            current_time = time.strftime("%H:%M:%S", t)
            dateStr = str(dateNow)
            timeStr = str(current_time)
            dateTime = dateStr + "T" + timeStr
            BaseDateTime = dateTime

            #print(self.gps_list,"self.gps_list")
            print(self.vessel_indx,"self.vessel_indx in  csv logging")
            LAT = self.gps_list[(self.vessel_indx)*2 - 2]
            LON = self.gps_list[((self.vessel_indx)*2) - 1]

            SOG = 0#we don't use these things
            COG = 0

            orientation_x = self.imu_list[self.vessel_indx*4 -4]
            orientation_y = self.imu_list[self.vessel_indx*4 -3]
            orientation_z = self.imu_list[self.vessel_indx*4 -2]
            orientation_w = self.imu_list[self.vessel_indx*4 -1]
            roll,pitch,Heading = euler_from_quaternion([orientation_x, orientation_y, orientation_z, orientation_w])

            VesselName = self.vessel_details_list[self.vessel_indx].VesselName

            IMO = self.vessel_details_list[self.vessel_indx].IMO

            CallSign = self.vessel_details_list[self.vessel_indx].CallSign
            
            VesselType = self.vessel_details_list[self.vessel_indx].VesselType

            MP_Mode = self.vessel_is_on_global_path_list[self.vessel_indx -1]

            Length = self.vessel_details_list[self.vessel_indx].Length

            Width = self.vessel_details_list[self.vessel_indx].Width

            Draft = self.vessel_details_list[self.vessel_indx].Draft

            Cargo = self.vessel_details_list[self.vessel_indx].Cargo

            TransceiverClass = self.vessel_details_list[self.vessel_indx].TransceiverClass

            VesselID = self.vessel_details_list[self.vessel_indx].vessel_id
            
            RowData = [MMSI,BaseDateTime,LAT,LON,SOG,COG,Heading,VesselName,IMO,CallSign,VesselType,MP_Mode,Length,Width,Draft,Cargo,TransceiverClass,VesselID]#TODO: This field names are referenced from the real world AIS data obtained from marinecadastre.gov
            
            
            with open(self.full_path+"/simulation_log_files/"+self.filename, mode='a') as csv_file:
                self.writer = csv.writer(csv_file)#fieldnames = self.CSVfieldnames
                self.writer.writerow(RowData)
                csv_file.close()
           
            self.vessel_indx +=1



    def Log_To_CSV_File_Loop(self):
        self.rate = rospy.Rate(2)


        while not rospy.is_shutdown():
            #try:

                #Publish every vessel's GPS sensor data in a list.


                #repeat=False
                
                #plt.pause(0.01) 
                plt.gcf().canvas.draw_idle()
                plt.gcf().canvas.start_event_loop(0.001)


                #self.Plot_Iteration()
                #self.anim = FuncAnimation(plt.gcf(), self.Plot_Iteration, frames=10 , interval=100, blit=True)
                
                #plt.show()
                #self.Plot_Iteration()


                
                self.rate.sleep()
                
            #except Exception as e:
                #print(e)
                #rospy.logerr(e)
                #exit()
        
    #This function will be used to publish distance_measurement_list and coordination_list

        
        


            


if __name__ == '__main__':
    rospy.init_node('Simulation_Logger_Node', anonymous=True)
    hz = 10
    rate = rospy.Rate(hz)
    vessel_objects = []
    vessel_count = int(rospy.get_param("/vessel_count"))#get the number of how many Vessels do we spawn in this run
    print("Number of Vessels: ", str(vessel_count))
     
    
    for i in range(vessel_count):
        vessel_objects.append(Local_plotter(i+1))

    while not rospy.is_shutdown():
        for i in vessel_objects:
            i.Plot_Iteration()


    
