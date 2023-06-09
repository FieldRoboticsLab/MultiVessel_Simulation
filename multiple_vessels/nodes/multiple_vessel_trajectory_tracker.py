#!/usr/bin/env python
"""
Trajectory Tracker ROS node

More details, flowcharts and block diagrams about this node and the whole package in generai is available in the gitHub repository.

Subscribing to both local and global planning ROS topics and ROS parameters, this node publishes the current waypoint for the controller of the robot to follow.
"""

from os import kill
import rospy
import time
import math
from rospy.core import rospydebug
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray,Int16
import threading
import json
from geopy import distance
import sys
import os
from enum import Enum

class Path_Mode(Enum):
    LOCAL = 0
    GLOBAL = 1
    CALCULATING_LOCAL_PATH = 2
    PLACEHOLDER = 99

class Trajectory_Tracker(threading.Thread):
    def __init__(self, VesselID_):
        threading.Thread.__init__(self)

        #This class will be generated by main function and will perform the processes mentioned at the top description comment and flowchart.
        
        self.path = "~/vrx_ws/src/vrx/multiple_vessels"
        self.full_path = os.path.expanduser(self.path)
        
        with open(self.full_path+'/json_files/config.json','r') as f:    
            self.config_data = json.load(f)

        self.vesselID = VesselID_
        self.is_on_global_path =1 # In the beginning, we assume every vessel starts at global path

        #Vessel trajectory tracker settings from config.json

        self.global_wp_reaching_treshold_meters = self.config_data["path_tracker"][0]["global_wp_reaching_treshold_meters"]
        self.local_wp_reaching_treshold_meters = self.config_data["path_tracker"][0]["local_wp_reaching_treshold_meters"]

        #Load our own vessel's details. Because it's necessary to calculate the ship domain's radius.
        vessel_details_param_txt = "/vessel" + str(self.vesselID)  + "_details"
        self.vessel_details = rospy.get_param(vessel_details_param_txt)
        
        self.global_trajectory_txt = '/vessel'+str(self.vesselID)+"_Global_Trajectory"
        self.global_trajectory = []
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

        #subscribe to GPS and IMU sensors
        sub_topic_gps_txt = "vessel"+str(self.vesselID) +"/vessel"+str(self.vesselID)+"/sensors"+"/gps/gps/fix"
        rospy.Subscriber(sub_topic_gps_txt,NavSatFix, self.CallbackGps)
        sub_topic_imu_txt = "vessel"+str(self.vesselID) +"/vessel"+str(self.vesselID)+"/sensors"+"/imu/imu/data"
        rospy.Subscriber(sub_topic_imu_txt,Imu, self.CallbackImu)

        #Subscribe to is_on_global_path topic
        is_on_global_path_txt = "vessel"+str(self.vesselID) + "/is_on_global_path"
        rospy.Subscriber(is_on_global_path_txt,Int16, self.Callback_is_on_global_path)

        #Subscribe to local path topic.
        sim_local_path_topic_txt =  "vessel"+str(self.vesselID) + "/local_path"
        rospy.Subscriber(sim_local_path_topic_txt,Float64MultiArray, self.CallbackLocalPath)

        #Init the current_global_waypoint topic
        pub_current_waypoint_txt = "vessel"+str(self.vesselID) + "/current_waypoint"
        self.pub_current_waypoint= rospy.Publisher(pub_current_waypoint_txt,Float64MultiArray,queue_size=10)
        
        print(str(self.vesselID)," vessel trajectory tracker started")

        self.localPath = []
        
        self.localPath_minus_one = []

        self.current_local_path = []

        self.vessel_is_on_global_path_t_minus = 1

        self.current_waypointLat = 0
        self.current_waypointLon=0

        self.last_reached_local_wp_index = 0

        self.vessel_is_on_global_path = Path_Mode.PLACEHOLDER.value#placeholder to not let the code run before taking the topic data

        self.reached_to_final_global_wp = False

        
    def run(self):
        while not rospy.is_shutdown():
            #try:
                rate.sleep()

                try:
                    self.vessel_is_on_global_path = self.is_on_global_path.data
                except:
                    pass
                if self.reached_to_final_global_wp == False:

                    if self.vessel_is_on_global_path == Path_Mode.LOCAL.value:
                        #vessel is on local path
                        self.Find_Current_Local_Wp()
                        self.vessel_is_on_global_path_t_minus = self.vessel_is_on_global_path
                        pass
                    
                    elif self.vessel_is_on_global_path == Path_Mode.GLOBAL.value:
                        #vessel is on global path
                        self.Find_Current_Global_Wp()
                        self.vessel_is_on_global_path_t_minus = self.vessel_is_on_global_path
                        pass

                    elif self.vessel_is_on_global_path == Path_Mode.CALCULATING_LOCAL_PATH.value:
                        #local path is being generated now. Track whatever the vessel was tracking until the local path is generated. 

                        if self.vessel_is_on_global_path_t_minus == Path_Mode.LOCAL.value:
                            self.vessel_is_on_global_path = Path_Mode.LOCAL.value
                            pass
                        elif self.vessel_is_on_global_path_t_minus == Path_Mode.GLOBAL.value:
                            self.vessel_is_on_global_path = Path_Mode.GLOBAL.value
                            pass 
                        pass

                    self.Publish_Current_Waypoint()
                else:
                    print("Global trajectory is finished for vessel"+str(self.vesselID))
                    #After the global path is finished, publish [999,999,999] to give that information to path tracker
                    self.vessel_is_on_global_path=Path_Mode.PLACEHOLDER.value

                    while True:
                        self.Publish_Current_Waypoint()
                        rate.sleep()

    def Find_Current_Global_Wp(self):
        #This function finds which global waypoint is the current waypoint.
        #And checks if we got close enough to the current global waypoint to proceed to the next global waypoint.

        #Look at the global path
        #Look at the last reached global wp
        #Find which waypoint is the current waypoint from these two information.
        #If the distance in between the vessel and the current global waypoint is closer than global wp reaching treshold, that means the vessel reached to the current global wp.
        #   Therefore the current waypoint should be updated. Increase the last reached global wp by five (since there are five list elements for each waypoint.)
        #Assign the current global waypoint to a variable.
        #END OF THE ITERATION.

        self.localPath = []#In every iteration we reset the localPath. Because if we have switched to global path, there is no need to store the local path anymore.

        self.Last_Reached_Global_WpRosParam = rospy.get_param("/vessel"+str(self.vesselID)+"_Last_Reached_Global_Wp")

        if self.Last_Reached_Global_WpRosParam <= (len(self.global_trajectory)/5) - 1: 

            WaypointLat = self.global_trajectory[5*(self.Last_Reached_Global_WpRosParam)]
            WaypointLon = self.global_trajectory[(5*(self.Last_Reached_Global_WpRosParam))+1]
            self.waypoint = (WaypointLat,WaypointLon)
            self.current_position = (self.current_lat,self.current_lon)

            distance_in_between = distance.distance(self.waypoint, self.current_position).meters
            
            if distance_in_between < self.global_wp_reaching_treshold_meters:
                self.Last_Reached_Global_WpRosParam +=1#jump to the next waypoint.
                rospy.set_param('vessel'+str(self.vesselID)+"_Last_Reached_Global_Wp", self.Last_Reached_Global_WpRosParam)
                print('vessel'+str(self.vesselID)+"_Last_Reached_Global_Wp: ",self.Last_Reached_Global_WpRosParam)
            else:
                self.current_waypointLat = WaypointLat
                self.current_waypointLon = WaypointLon
        else:
            print("vessel"+str(self.vesselID)+"reached to the final waypoint.")
            self.reached_to_final_global_wp = True

    def Find_Current_Local_Wp(self):
        #if local_path_minus_one != local_path:
        #   fresh start to a new local path.
        #   local_path_minus_one = local_path
        #   last_reached_local_wp = 0

        #else:
        #   keep on tracking this local path
        #rest is the same with local path.
        #at the end of it, assign the current local path to self.current_waypoint

        #before all, check if global path is completed. If so, don't even bother doing anything.
        self.Last_Reached_Global_WpRosParam = rospy.get_param("/vessel"+str(self.vesselID)+"_Last_Reached_Global_Wp")
        
        if self.Last_Reached_Global_WpRosParam >= len(self.global_trajectory)/5: 
            print("vessel"+str(self.vesselID)+"reached to the final waypoint.")
            self.reached_to_final_global_wp = True
        
        elif self.localPath != []:
            if self.localPath != self.localPath_minus_one:
                print("fresh start to a new local path. Vessel"+str(self.vesselID))
                self.localPath_minus_one = self.localPath 
                self.last_reached_local_wp_index = 0
            else:
                if self.last_reached_local_wp_index <=(len(self.localPath)/2)-1: 
                    #local wp is not changed.
                    waypointLat = self.localPath[2*(self.last_reached_local_wp_index)]
                    waypointLon = self.localPath[(2*(self.last_reached_local_wp_index))+1]

                    self.waypoint = (waypointLat,waypointLon)
                    self.current_position = (self.current_lat,self.current_lon)

                    distance_in_between = distance.distance(self.waypoint, self.current_position).meters

                    if distance_in_between < self.local_wp_reaching_treshold_meters:
                        self.last_reached_local_wp_index +=1
                    else:
                        self.current_waypointLat = waypointLat
                        self.current_waypointLon = waypointLon
                else:
                    print("vessel"+str(self.vesselID)+"reached to the local goal waypoint.")

    def Publish_Current_Waypoint(self):
        #In this function, we publish the current waypoint for the controller to make the vessel reach.
        #With the lat-lon information, we also include the state of the /vesselX/is_on_global_path
        #Because in the controller, we might run the robot in different speeds for tracking the local and global paths.
        msg_to_be_published = Float64MultiArray()
        msg_to_be_published.data = [self.current_waypointLat,self.current_waypointLon,self.vessel_is_on_global_path]
        self.pub_current_waypoint.publish(msg_to_be_published)
        
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

    def CallbackLocalPath(self,local_path_msg_):
        self.localPath  = local_path_msg_.data

    def Callback_is_on_global_path(self,msg_):
        self.is_on_global_path = msg_

    def CallbackGps(self,GPS_msg):

        self.current_lat = GPS_msg.latitude
        self.current_lon = GPS_msg.longitude
    
    def CallbackImu(self,imu_msg):    
        self.quaternion_x  = imu_msg.orientation.x
        self.quaternion_y = imu_msg.orientation.y
        self.quaternion_z = imu_msg.orientation.z
        self.quaternion_w = imu_msg.orientation.w
        
if __name__ == '__main__':
    rospy.init_node('multi_vessel_trajectory_tracker', anonymous=True)
    hz = 10
    rate = rospy.Rate(hz)

    vessel_objects = []
    vessel_count = int(rospy.get_param("/vessel_count"))#get the number of how many Vessels do we spawn in this run
    print("Number of Vessels: ", str(vessel_count))

    i=1
    for i in range(vessel_count):
        vessel_objects.append(Trajectory_Tracker(i+1))

    for obj in vessel_objects:
        obj.start()