#!/usr/bin/env python
"""
This node is the Global_Path_planner node
for every spawned Vessel in the sim environment.

In this ROS node, Vessel_global_planner class is defined for every Vessel that's available in the 
Gazebo simulation environment.

Global Motion Planner Node that generates global waypoints 
"""

from os import kill
import rospy
from rospy.core import rospydebug
import json
import os

class Vessel_Global_Path_Planner():
    def __init__(self,vessel_count_):

        self.path = "~/vrx_ws/src/vrx/multiple_vessels"
        self.full_path = os.path.expanduser(self.path)

        with open(self.full_path+'/json_files/Global_Waypoints_userDefinedVessels.json','r') as f:    
                self.global_wp_data = json.load(f)

        self.vessel_count = vessel_count_
        #This list contains the VesselIDs numbers of vessels in the sensing range of this vessel.

        self.Set_Global_Trajectories()

    def Set_Global_Trajectories(self):
        #This function sets the ROS parameters for every vessel:
        #     /VesselX/Global_Trajectory
        #     /VesselX/Last_Reached_Global_Wp
        #     /VesselX/Is_On_Global_Path
        #Based on AIS data from Json_Global_Waypoints.json

        self.AIS_data = self.global_wp_data["AIS_Data"]

        vessel_id =  self.vessel_count+1
        for i in range(len(self.AIS_data)):
            #In each iteration, set the vessel information by reading from AIS data JSON
            self.vessel_global_waypoints = []
            #This list will store the global waypoints for one vessel in the iteration.
            #The format will be like this:
            #Lat0, Lon0, BaseDateTime0, Velocity_to_Next_Wp0, Heading_to_Next_Wp0, Lat1, Lon1, BaseDateTime1, Velocity_to_Next_Wp1, Heading_to_Next_Wp1,,,,
            for j in range(len(self.AIS_data[i])):
                #Goes through every AIS data from start to finish for a vessel.
                one_ais_data = self.AIS_data[i][j]
                self.vessel_global_waypoints.append(one_ais_data["LAT"])
                self.vessel_global_waypoints.append(one_ais_data["LON"])
                self.vessel_global_waypoints.append(one_ais_data["BaseDateTime"])
                self.vessel_global_waypoints.append(one_ais_data["Velocity_to_Next_Waypoint"])
                self.vessel_global_waypoints.append(one_ais_data["Heading_to_Next_Waypoint"])
            
            #After the whole global waypoints are collected in one list, set it to a ROS parameter
            rospy.set_param('vessel'+str(vessel_id)+"_Global_Trajectory", self.vessel_global_waypoints)
            #Set the /VesselX/Last_Reached_Global_Wp and /VesselX/Is_On_Global_Path parameters as well
            rospy.set_param('vessel'+str(vessel_id)+"_Last_Reached_Global_Wp", 0)

            vessel_id +=1
    
if __name__ == '__main__':
    rospy.init_node('multi_vessel_global_planner', anonymous=True)
    hz = 10
    rate = rospy.Rate(hz)

    vessel_count = int(rospy.get_param("/vessel_count"))#get the number of how many Vessels did we spawn in this run
    print("Number of Vessels: ", str(vessel_count))
    user_defined_vessel_count = int(rospy.get_param("/user_defined_vessel_count")) 
    vesselID_of_udv =  vessel_count+1

    Vessel_Global_Path_Planner(vessel_count)