#!/usr/bin/env python
"""
Simulation Logger node
Stores the vessel positions in the simulation in a csv file as the AIS data format.

Also the logger node will visualize the current positions and such.
"""

#That's how you keep the work going in the background
#https://stackoverflow.com/questions/28269157/plotting-in-a-non-blocking-way-with-matplotlib

#How to use this anim functions with a class
#https://gist.github.com/grburgess/09f9bedc85887ecbbf69efd387fa1cf6

#How to plot stuff on top of basemap
#https://www.youtube.com/watch?v=XiZbrii49pI

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

import nest_asyncio
nest_asyncio.apply()

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

        print(str(self.vesselID)," vessel tracker started")

    def Callback_is_on_global_path(self,msg_):
        self.is_on_global_path = msg_.data

    def CallbackLocalPath(self,local_path_msg_):
        self.localPath  = local_path_msg_.data

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

class Logger():
    def __init__(self):
        #Read the JSON config file
        self.path = "~/vrx_ws/src/vrx/MultiVessel_Simulation/multiple_vessels"
        self.full_path = os.path.expanduser(self.path)
        
        with open(self.full_path+'/json_files/config.json','r') as f:    
            self.config_data = json.load(f)

        with open(self.full_path+'/json_files/Json_Global_Waypoints.json','r') as f:    
            self.global_wp_data = json.load(f)

        self.json_TopRightLat = self.global_wp_data["TopRightLat"]
        self.json_TopRightLon = self.global_wp_data["TopRightLon"]
        self.json_BotLeftLat = self.global_wp_data["BotLeftLat"]
        self.json_BotLeftLon = self.global_wp_data["BotLeftLon"]
        print(self.json_TopRightLat,"self.json_TopRightLat")
        
        self.scatter_distance_treshold = self.config_data["simulation_logger"][0]["scatter_distance_treshold"]
        self.logging_frequency = self.config_data["simulation_logger"][0]["logging_frequency"]
        self.plotting_interval = self.config_data["simulation_logger"][0]["plotting_interval"]
        self.plotting_available = self.config_data["simulation_logger"][0]["plotting_available"]
        self.logging_available = self.config_data["simulation_logger"][0]["logging_available"]

        self.hz = self.logging_frequency
        self.rate = rospy.Rate(hz)
    
        self.vessel_objects = []
        self.vessel_count = int(rospy.get_param("/vessel_count"))#get the number of how many Vessels do we spawn in this run
        print("Number of Vessels: ", str(self.vessel_count))
    
        self.Vessel_objects()
        #self.Initialize_Threads()
        self.Vessel_Is_On_Global_Path_List_Generator()
        time.sleep(0.2)
            
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
        self.local_plot_list = []
        self.last_plotted_local_path_list = []
        self.local_plot_flag_list = []
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
            self.local_plot_list.append([])
            self.last_plotted_local_path_list.append([])
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
        
        self.cross_scatter_list_lat = []
        self.cross_scatter_list_lon = []

        self.square_scatter_list_lat = []
        self.square_scatter_list_lon = []

        self.vessel_local_waypoints_lat = []
        self.vessel_local_waypoints_lon = []
        
        sub_topic_gps_txt = "simulation/gps_list"
        rospy.Subscriber(sub_topic_gps_txt,Float64MultiArray, self.CallbackGpsList)

        sub_topic_imu_txt = "simulation/imu_list"
        rospy.Subscriber(sub_topic_imu_txt,Float64MultiArray, self.CallbackImuList)

        self.Get_Vessel_Detail_Params()
        rate.sleep()
        
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

    def CallbackGpsList(self,gpsList_msg):
        self.gps_list = gpsList_msg.data

    def CallbackImuList(self,imuList_msg):
        self.imu_list = imuList_msg.data

    def Init_Plotting(self):
        self.plots = []
        #Instead of dividing into 3 lists, gather all the iterative plots into one list called iterative_self.vessel_tracked_path_plots
        #This iterative plots will be emptied in every iteration in order to get refilled from live data in every iteration
        
        self.vesselColors =[]

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

        #This function runs the commands to initialize the world map to be plotted on. 
        self.m = Basemap(projection='mill',
           llcrnrlat = self.json_BotLeftLat,#These values are pulled off from the global paths JSON
           urcrnrlat = self.json_TopRightLat,
           llcrnrlon = self.json_BotLeftLon,
           urcrnrlon = self.json_TopRightLon,
           resolution = 'l')
        self.m.drawcoastlines()
        self.m.drawcountries(linewidth = 2)
        self.m.drawstates(color = 'b')
        self.m.drawmapboundary(fill_color='white') # I want to make the water white so we can see the plotted lines and quivers better. 
        self.m.fillcontinents(color='coral',lake_color='aqua')

        plt.title("Logger Node Plotting Initialized")
        plt.show(block=False)
        plt.gcf().canvas.start_event_loop(0.001)

        #That's how you keep the work going in the background
        #https://stackoverflow.com/questions/28269157/plotting-in-a-non-blocking-way-with-matplotlib

        #How to use this anim functions with a class
        #https://gist.github.com/grburgess/09f9bedc85887ecbbf69efd387fa1cf6

        #How to plot stuff on top of basemap
        #https://www.youtube.com/watch?v=XiZbrii49pI

        self.Plot_Global_Paths_And_Vessel_Locations()

        #If we are not freshly starting, we need to read the previous vessel positions from the current logged_sim_data.csv file. And plot them.
        if self.resume_or_fresh_start == 0:
            self.Read_Previous_Locations()
            self.Scatter_Previous_Locations()
            plt.show(block=False)
            plt.gcf().canvas.start_event_loop(0.001)

        elif self.resume_or_fresh_start == 1:

            self.vessel_index = 1#I will make this parametric depending on the config file. Right now I start from 1 because there is no vessel0
            while self.vessel_index <= self.vessel_count:#If vesselcount is 4, this loop goes 1,2,3,4 now.
                #for every vessel in the simulation environment, do the following:
                #Set the last scatter position to the current position. 
                LAT = self.gps_list[(self.vessel_index)*2 - 2]
                LON = self.gps_list[((self.vessel_index)*2) - 1]
                self.last_scatter_pos[self.vessel_index] = [LAT,LON,0]
                
                self.vessel_index +=1

        return self.plots

    def Read_Previous_Locations(self):
        #Read from csv file and plot previous locations of vessels 
        print("Loading previous vessel locations from logged_sim_data.csv.")
        
        self.rows = [] 
        with open(self.full_path+"/simulation_log_files/"+self.filename, mode='r') as csv_file:
            # creating a csv reader object
            csvreader = csv.reader(csv_file)
            
            # extracting field names through first row
            fields = next(csvreader)

                    # extracting each data row one by one
            for row in csvreader:
                self.rows.append(row)
        
            # get total number of rows
            print("Total no. of rows: %d"%(csvreader.line_num))
            self.total_number_of_rows = csvreader.line_num
            #After the previous rows are loaded, plot based on this self.rows list

    def Scatter_Previous_Locations(self):
        #This function is called by Init_Plotting function if the logging is not freshly started but resuming on the logging from before.
        #This function reads through the self.rows list to generate the self.trail_scatter_list.
        # The self.trail_scatter_list stores the points that are seperated enough to be considered worth scattering. 

        self.vessel_index = 1#I will make this parametric depending on the config file. Right now I start from 1 because there is no vessel0
        while self.vessel_index <= self.vessel_count:#If vesselcount is 4, this loop goes 1,2,3,4 now.
            #for every vessel in the simulation environment, do the following:
            #Append the initial vessel positions to the self.trail_scatter_list 

            row_index = self.vessel_index-1
            LAT = self.rows[row_index][2]
            LON = self.rows[row_index][3]
            Heading = self.rows[row_index][6]

            self.last_scatter_pos[self.vessel_index] = [LAT,LON,Heading]
            self.trail_scatter_list[self.vessel_index].append([LAT,LON,Heading])
            self.vessel_index +=1

        self.vessel_index = 1#I will make this parametric depending on the config file. Right now I start from 1 because there is no vessel0
        while self.vessel_index <= self.vessel_count:#If vesselcount is 4, this loop goes 1,2,3,4 now.
            #for every vessel in the simulation environment, do the following:

            self.row_index =self.vessel_index -1#Starting from the first row for every vessel, go through the rows. 
            #If we are looking for the first line for vessel2
            #I put a -1 because self.rows list starts from zero.

            while self.row_index < self.total_number_of_rows-1:
                print("self.rows[self.row_index]",self.rows[self.row_index])
                lat_from_row = self.rows[self.row_index][2]
                lon_from_row = self.rows[self.row_index][3]
                heading_from_row = self.rows[self.row_index][6]
                row_position = (lat_from_row,lon_from_row)

                print("self.trail_scatter_list",self.trail_scatter_list)
                print(self.last_scatter_pos,"last_scatter_pos")

                lat_last_scatter = self.last_scatter_pos[self.vessel_index][0]
                lon_last_scatter = self.last_scatter_pos[self.vessel_index][1]
                last_scatter_position = (lat_last_scatter,lon_last_scatter)

                distance_in_between = distance.distance(row_position, last_scatter_position).meters
                print(distance_in_between,"distance_in_between")
               
                if distance_in_between > self.scatter_distance_treshold:
                    self.trail_scatter_list[self.vessel_index].append([lat_from_row,lon_from_row,heading_from_row])
                    self.last_scatter_pos[self.vessel_index] = [lat_from_row,lon_from_row,heading_from_row]

                self.row_index = (self.row_index) + self.vessel_count

            self.vessel_index +=1

        self.vessel_index = 1#I will make this parametric depending on the config file. Right now I start from 1 because there is no vessel0
        while self.vessel_index <= self.vessel_count:#If vesselcount is 4, this loop goes 1,2,3,4 now.
            #for every vessel in the simulation environment, do the following:
            vessel_scatter_list = self.trail_scatter_list[self.vessel_index]
            print("vessel_scatter_list",vessel_scatter_list)
            
            for i in range(len(vessel_scatter_list)):
                #Scatter from self.trail_scatter_list
                lat = float(self.trail_scatter_list[self.vessel_index][i][0])
                lon = float(self.trail_scatter_list[self.vessel_index][i][1])
                self.m.scatter(lon,lat,latlon = True,color=self.vesselColors[self.vessel_index])

            self.vessel_index +=1

    def Plot_Global_Paths_And_Vessel_Locations(self):
        #This function reads the Global Waypoints JSON file and plots the global paths of the vessels.
        self.vessel_index = 1#I will make this parametric depending on the config file. Right now I start from 1 because there is no vessel0
        while self.vessel_index <= self.vessel_count:#If vesselcount is 4, this loop goes 1,2,3,4 now.
            #for every vessel in the simulation environment, do the following:

            #Plot global paths
            self.vessel_global_waypoints_lat = []
            self.vessel_global_waypoints_lon = []

            self.AIS_data = self.global_wp_data["AIS_Data"]
            for j in range(len(self.AIS_data[self.vessel_index-1])):#-1 because the JSON file starts from 0 not 1
                #Goes through every AIS data from start to finish for a vessel.
                one_ais_data = self.AIS_data[self.vessel_index-1][j]
                self.vessel_global_waypoints_lat.append(one_ais_data["LAT"])
                self.vessel_global_waypoints_lon.append(one_ais_data["LON"])

            print(self.vessel_global_waypoints_lat,"self.vessel_global_waypoints_lat")
            global_path_plotting,= self.m.plot(self.vessel_global_waypoints_lon,self.vessel_global_waypoints_lat,latlon = True,color=self.vesselColors[self.vessel_index],label = "Vessel "+str(self.vessel_index)+" Global Path")

            #Plot the Vessel Positions with quivers.
            LAT = self.gps_list[(self.vessel_index)*2 - 2]
            LON = self.gps_list[((self.vessel_index)*2) - 1]
            X_pos,Y_pos = self.m(LON, LAT)

            orientation_x = self.imu_list[self.vessel_index*4 -4]
            orientation_y = self.imu_list[self.vessel_index*4 -3]
            orientation_z = self.imu_list[self.vessel_index*4 -2]
            orientation_w = self.imu_list[self.vessel_index*4 -1]
            roll,pitch,Heading = euler_from_quaternion([orientation_x, orientation_y, orientation_z, orientation_w])

            X_direct = math.cos(Heading)*10
            Y_direct = math.sin(Heading)*10 
            vessel_location = self.m.quiver(X_pos,Y_pos,X_direct,Y_direct,color=self.vesselColors[self.vessel_index],label = "Vessel "+str(self.vessel_index))
            print("in anim func")

            self.vessel_index +=1

        plt.gcf().canvas.start_event_loop(0.001)
        
    def Plot_Iteration(self,i):
        #The FuncAnimation function uses this animate function to iterate. 
        #In this function, we plot the:
        ##Global Paths of the vessels with plot function to plot the global paths with lines.
        ##Vessels' position and orientations with quiver function.
        ##Ship domains of every vessel with scatter. 
        ##Local paths for vessels if there is any.  
        self.plots = []

        print("In plot iteration")
        plt.cla()
        self.vessel_index = 1#I will make this parametric depending on the config file. Right now I start from 1 because there is no vessel0
        while self.vessel_index <= self.vessel_count:#If vesselcount is 4, this loop goes 1,2,3,4 now.
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

            global_path_plotting,= self.m.plot(self.vessel_global_waypoints_lon,self.vessel_global_waypoints_lat,latlon = True,color=self.vesselColors[self.vessel_index],linestyle = '--',label = "Vessel "+str(self.vessel_index)+" Global Path")

            #Plot the Vessel Positions with quivers.
            LAT = self.gps_list[(self.vessel_index)*2 - 2]
            LON = self.gps_list[((self.vessel_index)*2) - 1]
            X_pos,Y_pos = self.m(LON, LAT)

            orientation_x = self.imu_list[self.vessel_index*4 -4]
            orientation_y = self.imu_list[self.vessel_index*4 -3]
            orientation_z = self.imu_list[self.vessel_index*4 -2]
            orientation_w = self.imu_list[self.vessel_index*4 -1]
            roll,pitch,Heading = euler_from_quaternion([orientation_x, orientation_y, orientation_z, orientation_w])

            X_direct = math.cos(Heading)*10
            Y_direct = math.sin(Heading)*10 
            vessel_location = self.m.quiver(X_pos,Y_pos,X_direct,Y_direct,color=self.vesselColors[self.vessel_index],label = "Vessel "+str(self.vessel_index))

            vessel_scatter_list = self.trail_scatter_list[self.vessel_index]

            for i in range(len(vessel_scatter_list)):
                #Scatter from self.trail_scatter_list
                lat = float(self.trail_scatter_list[self.vessel_index][i][0])
                lon = float(self.trail_scatter_list[self.vessel_index][i][1])
                self.m.scatter(lon,lat,latlon = True,color=self.vesselColors[self.vessel_index],alpha=0.4)
            
            #Plot trail scatter to indicate the positions that a vessel have been on before.
            lat_last_scatter = self.last_scatter_pos[self.vessel_index][0]
            lon_last_scatter = self.last_scatter_pos[self.vessel_index][1]
            last_scatter_position = (lat_last_scatter,lon_last_scatter)
            current_pos = (LAT,LON)

            distance_in_between = distance.distance(current_pos, last_scatter_position).meters

            if distance_in_between > self.scatter_distance_treshold:
                self.trail_scatter_list[self.vessel_index].append([LAT,LON,Heading])
                self.last_scatter_pos[self.vessel_index] = [LAT,LON,Heading]

            #After we plot the global paths and current vessel positions, plot the local paths if there is any.
            MP_Mode = self.vessel_objects[self.vessel_index - 1].is_on_global_path

            self.cross_placed_for_switching_to_local_path_flag = self.local_plot_flag_list[self.vessel_index][0]
            self.square_placed_for_switching_to_global_path_flag = self.local_plot_flag_list[self.vessel_index][1]
            
            if MP_Mode == 1:
                self.cross_placed_for_switching_to_local_path_flag = False
                self.local_plot_flag_list[self.vessel_index][0] = False
                
                if self.square_placed_for_switching_to_global_path_flag == False:
                    self.square_scatter_list[self.vessel_index].append([LAT,LON,Heading])
                    #Plot a square in order to indicate that vessel has just switched to the global path.
                    self.local_plot_flag_list[self.vessel_index][1] = True

            if MP_Mode == 2:
                #If a local path is being planned at the moment, put a cross to the map to indicate the motion planning mode will be set to local after this point.
                if self.cross_placed_for_switching_to_local_path_flag == False:
                    self.cross_scatter_list[self.vessel_index].append([LAT,LON,Heading])
                    self.local_plot_flag_list[self.vessel_index][0] = True
            
            if MP_Mode == 0:
                #If the path planning is local path mode. Switch both the flags below to false in order to put the appropriate marker.
                self.local_plot_flag_list[self.vessel_index][1] = False

                if self.cross_placed_for_switching_to_local_path_flag == False:
                    self.cross_scatter_list[self.vessel_index].append([LAT,LON,Heading])
                    self.local_plot_flag_list[self.vessel_index][0] = True

                self.local_wp_index = 2
                self.local_path = self.vessel_objects[self.vessel_index - 1].localPath
                
                if self.local_path != self.last_plotted_local_path_list[self.vessel_index]:
                    print("new local path", self.local_path)
                    new_local_path_lat = []
                    new_local_path_lon = []
                    new_local_path = []
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

                        new_local_path_lat.append(waypointLat)
                        new_local_path_lat.append(nextWaypointLat)
                        new_local_path_lon.append(waypointLon)
                        new_local_path_lon.append(nextWaypointLon)

                        self.local_wp_index +=2

                    new_local_path.append(new_local_path_lat)    
                    new_local_path.append(new_local_path_lon)
                    self.local_path_plot_list[self.vessel_index].append(new_local_path)  

                    print("self.local_path_plot_list",self.local_path_plot_list)
                    print(" self.local_path_plot_list[self.vessel_index]", self.local_path_plot_list[self.vessel_index])

                    self.last_plotted_local_path_list[self.vessel_index] = self.local_path

            square_scatter_list = self.square_scatter_list[self.vessel_index]

            if square_scatter_list != []:
                for i in range(len(square_scatter_list)):
                    #Scatter from self.square_scatter_list
                    lat = float(self.square_scatter_list[self.vessel_index][i][0])
                    lon = float(self.square_scatter_list[self.vessel_index][i][1])
                    self.m.scatter(lon,lat,latlon = True,color=self.vesselColors[self.vessel_index],alpha=0.7,marker='s')

            cross_scatter_list = self.cross_scatter_list[self.vessel_index]

            if cross_scatter_list !=[]:
                for i in range(len(cross_scatter_list)):
                    #Scatter from self.cross_scatter_list
                    lat = float(self.cross_scatter_list[self.vessel_index][i][0])
                    lon = float(self.cross_scatter_list[self.vessel_index][i][1])
                    self.m.scatter(lon,lat,latlon = True,color=self.vesselColors[self.vessel_index],alpha=0.7,marker='X')

            if self.local_path_plot_list[self.vessel_index] != []:
                print("plotting local path for vessel",self.vessel_index )
                for local_path in  self.local_path_plot_list[self.vessel_index]:
                    local_path_plotting,= self.m.plot(local_path[1],local_path[0],latlon = True,color=self.vesselColors[self.vessel_index],linestyle = '-',label = "Vessel "+str(self.vessel_index)+" Local Path")
            
            self.vessel_index +=1

        
        print("plot iteration complete")
        plt.gcf().canvas.draw_idle()
        #If it's unclear that why I use plt.gcf().canvas.draw_idle() instead of plt.pause(0.01) or any other commented line here, please refer to:
        #https://stackoverflow.com/questions/45729092/make-interactive-matplotlib-window-not-pop-to-front-on-each-update-windows-7/45734500#45734500
        
        return self.plots

    def Vessel_objects(self):
        self.vessel_objects = []
        print(self.vessel_count)
        
        for i in range(self.vessel_count):
            self.vessel_objects.append(Vessel_is_on_global_path_Subscriber(i+1))

    def Vessel_Is_On_Global_Path_List_Generator(self):
        self.vessel_is_on_global_path_list = []

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

        while  self.vessel_indx <= self.vessel_count:
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
                self.Vessel_Is_On_Global_Path_List_Generator()

                self.CSV_File_Iteration()

                plt.gcf().canvas.draw_idle()
                plt.gcf().canvas.start_event_loop(0.001)
                
                self.rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node('Simulation_Logger_Node', anonymous=True)
    hz = 10
    rate = rospy.Rate(hz)
    my_logger_object = Logger()
    rospy.spin()