#!/usr/bin/env python
"""
Vessel_Controller ROS node
That sends the actuator signals to Waypoint.

Subscribes to /vesselX/current_waypoint topic. 
Using the waypoint's position and vessel's current position, this vessel controller node computates and outputs the actuator signals. 
Pure Pursuit algorithm is used for this controller.

More details, flowcharts and block diagrams about this node and the whole package in generai is available in the gitHub repository.
"""

from os import kill
import rospy
import time
import math
from rospy.core import rospydebug
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32,Float64MultiArray
import threading
import json
from geopy import distance
from tf.transformations import euler_from_quaternion
import os
import sys

class Vessel_Controller_PurePursuit(threading.Thread):
    def __init__(self, VesselID_):
        threading.Thread.__init__(self)
        self.path = "~/vrx_ws/src/vrx/multiple_vessels"
        self.full_path = os.path.expanduser(self.path)

        #This class will be generated by main function and will perform the processes mentioned at the top description comment and flowchart.
        #One object of this class will be called for each vessel.
        
        with open(self.full_path+'/json_files/config.json','r') as f:    
            self.config_data = json.load(f)

        self.vesselID = VesselID_

        #settings from config.json
        self.controller_pure_pursuit_chosen = self.config_data["controller"][0]["Pure_Pursuit"][0]["chosen"]
        self.look_ahead_distance_local = self.config_data["controller"][0]["Pure_Pursuit"][0]["look_ahead_distance_local_path_meters"]
        self.look_ahead_distance_global = self.config_data["controller"][0]["Pure_Pursuit"][0]["look_ahead_distance_global_path_meters"]
        self.linear_velocity_local = self.config_data["controller"][0]["Pure_Pursuit"][0]["linear_velocity_propeller_command_local_path"]
        self.linear_velocity_global = self.config_data["controller"][0]["Pure_Pursuit"][0]["linear_velocity_propeller_command_global_path"]

        #For the fail-safe, we get the local wp reaching treshold. 
        #So this ROS node can understand if the path tracker ROS node is crashed simply because the path tracker ROS node
        #does not publish a new goal pose before we got too close to it. Closer than half of the local treshold is a convenient value for this.
        self.local_wp_reaching_treshold_meters = self.config_data["path_tracker"][0]["local_wp_reaching_treshold_meters"]


        vessel_details_param_txt = "/vessel" + str(self.vesselID)  + "_details"
        self.vessel_details = rospy.get_param(vessel_details_param_txt)
        
        self.current_lat = ""
        self.current_lon = ""
        
        self.roll = ""
        self.pitch = ""
        self.yaw = ""

        self.quaternion_x = ""
        self.quaternion_y = ""
        self.quaternion_z = ""
        self.quaternion_w = ""

        #subscribe to GPS and IMU sensors
        sub_topic_gps_txt = "vessel"+str(self.vesselID) +"/vessel"+str(self.vesselID)+"/sensors"+"/gps/gps/fix"
        rospy.Subscriber(sub_topic_gps_txt,NavSatFix, self.CallbackGps)
        sub_topic_imu_txt = "vessel"+str(self.vesselID) +"/vessel"+str(self.vesselID)+"/sensors"+"/imu/imu/data"
        rospy.Subscriber(sub_topic_imu_txt,Imu, self.CallbackImu)

        #Subscribe to the current_waypoint topic
        sub_current_waypoint_txt = "vessel"+str(self.vesselID) + "/current_waypoint"
        rospy.Subscriber(sub_current_waypoint_txt,Float64MultiArray,self.Callback_current_waypoint)


        #init the actuator publishers
        pub_topic_leftThrustcmd_txt = "vessel"+str(self.vesselID) + "/thrusters/left_thrust_cmd"
        self.pub_leftThrustcmd= rospy.Publisher(pub_topic_leftThrustcmd_txt,Float32,queue_size=10)
        
        pub_topic_rightThrustcmd_txt = "vessel"+str(self.vesselID) + "/thrusters/right_thrust_cmd"
        self.pub_rightThrustcmd= rospy.Publisher(pub_topic_rightThrustcmd_txt,Float32,queue_size=10)

        pub_topic_leftThrustAngle_txt = "vessel"+str(self.vesselID) + "/thrusters/left_thrust_angle"
        self.pub_leftThrustAngle= rospy.Publisher(pub_topic_leftThrustAngle_txt,Float32,queue_size=10)

        pub_topic_rightThrustAngle_txt = "vessel"+str(self.vesselID) + "/thrusters/right_thrust_angle"
        self.pub_rightThrustAngle= rospy.Publisher(pub_topic_rightThrustAngle_txt,Float32,queue_size=10)

        
        print(str(self.vesselID)," vessel controller started")

        self.current_waypoint= []
        self.current_waypoint_minus_one= []

        self.current_waypointLat = 0
        self.current_waypointLon=0

        self.is_on_global_path =1 # In the beginning, we assume every vessel starts at global path

        self.intermediate_goal_x =0
        self.intermediate_goal_y = 0

        self.right_cmd_vel = Float32
        self.left_cmd_vel = Float32

        self.right_cmd_angle = Float32
        self.left_cmd_angle = Float32

    def run(self):
        while not rospy.is_shutdown():
            try:
                rate.sleep()
                
                if self.current_lat != "":
                
                    if self.current_waypoint != [] :
                        #something got published to the current waypoint topic.
                        
                        if self.current_waypoint != self.current_waypoint_minus_one:
                            #waypoint has been changed 
                            print("For Vessel",str(self.vesselID),", fresh start to reach the current waypoint:",self.current_waypoint)
                            self.current_waypoint_minus_one=self.current_waypoint
                        else:
                            #waypoint is not changed. Keep navigating to the waypoint.
                            self.current_waypointLat = self.current_waypoint[0]
                            self.current_waypointLon = self.current_waypoint[1]
                            self.is_on_global_path = self.current_waypoint[2]

                            self.waypoint = (self.current_waypointLat,self.current_waypointLon)
                            self.current_position = (self.current_lat,self.current_lon)

                            self.distance_inBetween_current_wp_to_vessel= distance.distance(self.waypoint, self.current_position).meters

                            if self.is_on_global_path == 99:
                                    print("Global waypoints are finished for vessel"+str(self.vesselID))
                                    sys.exit()
                            
                            if self.distance_inBetween_current_wp_to_vessel<(self.local_wp_reaching_treshold_meters/2):
                                #if distance in between is smaller than half of the local treshold, something's going wrong
                                print("Distance to the current waypoint is smaller than half of the reaching treshold.")
                                print("Did trajectory tracker ROS node crashed for vessel",str(self.vesselID),"?")
                                pass

                            else:
                                #distance is not smaller than half of the local treshold. We are safe to proceed.
                                self.origin_lat = self.current_lat
                                self.origin_lon = self.current_lon
                                self.current_x,self.current_y = self.get_local_coord(self.current_lat,self.current_lon)
                                #It will return 0,0 since we put the origin to wherever the vessel is at
                                roll,pitch,self.yaw = euler_from_quaternion([self.quaternion_x, self.quaternion_y, self.quaternion_z, self.quaternion_w])
                                self.waypoint_x,self.waypoint_y = self.get_local_coord(self.current_waypointLat,self.current_waypointLon)

                                if self.is_on_global_path == 0:
                                    #on local path. Set the look ahead distance and linear velocity.
                                    self.look_ahead_distance = self.look_ahead_distance_local
                                    self.linear_velocity = self.linear_velocity_local

                                elif self.is_on_global_path == 1:
                                    #on global path. Set the look ahead distance and linear velocity.
                                    self.look_ahead_distance = self.look_ahead_distance_global
                                    self.linear_velocity = self.linear_velocity_global

                                elif self.is_on_global_path == 2:
                                    #Generating the local waypoint right now. 
                                    # Keep tracking the waypoint from before until local path is generated.
                                    self.look_ahead_distance = self.look_ahead_distance_local
                                    self.linear_velocity = self.linear_velocity_local
                                else:
                                    self.look_ahead_distance = self.look_ahead_distance_local
                                    self.linear_velocity = self.linear_velocity_local

                                self.Calculate_Intermediate_Goal()
                                #print(str(self.vesselID)," vessel int goal: ",self.intermediate_goal_x,self.intermediate_goal_y)
                                alpha = math.atan2(self.intermediate_goal_y-self.current_y, self.intermediate_goal_x-self.current_x) - self.yaw#pose_goal_.theta	
                                K = 2*math.sin(alpha)/self.look_ahead_distance #curvature
                                v = self.linear_velocity
                                w = self.linear_velocity * K

                                self.right_cmd_vel = v
                                self.left_cmd_vel = v
                                self.right_cmd_angle = -1*w
                                self.left_cmd_angle = -1*w 
                                #You might wonder why we multiply this angular velocity value with -1
                                #For the vessels that have this configuration, which has some form of a rear rudder, you need to steer the exact opposite to go towards where you want.
                                #I found that out while we were testing out our first ever USV. And I was like "why does it turn the opposite? Did we set that wrong?"
                                #And I look at the rudder and no, rudder turns wherever I steer from my controller.
                                #Then I realized the mechanics of such vessel. It pushes the water to the right so it can turn it's body to left. And vice versa.
                                #It's similar to forklift. When a forklift goes, the wheels steer the opposite way compared to a regular front wheel steering car.

                                self.pub_leftThrustcmd.publish(self.left_cmd_vel)
                                self.pub_rightThrustcmd.publish(self.right_cmd_vel)
                                self.pub_rightThrustAngle.publish(self.right_cmd_angle)
                                self.pub_leftThrustAngle.publish(self.left_cmd_angle)

       
                    else:
                        print("no waypoints are published yet. Did you start the switch mechanism, local path planner and path tracker ROS nodes?")
                        rate.sleep()
                else:
                    print("No GPS information. Is the VRX simulation environment started?")
                    print("Or is the vessel_count ROS parameter is set wrong? So is this ROS node is trying to subscribe to a GPS that doesn't exist?")
                    rate.sleep()

            except Exception as e:
                rospy.logerr(e)
                
    def Calculate_Intermediate_Goal(self):
        #This function calculates the intermediate goal for the pure pursuit algorithm.
        #Because pure pursuit algorithm works best for the goal points that are not too far away.
        #distance_to_goal = math.hypot(self.waypoint_x-self.current_x, self.waypoint_y-self.current_y)
        distance_to_wp = self.distance_inBetween_current_wp_to_vessel

        if distance_to_wp > self.look_ahead_distance:
            angle_difference = math.atan2(self.waypoint_y-self.current_y, self.waypoint_x-self.current_x)
            self.intermediate_goal_x = self.current_x + self.look_ahead_distance * math.cos(angle_difference)
            self.intermediate_goal_y = self.current_y + self.look_ahead_distance * math.sin(angle_difference)
        else:
            #distance to waypoint is not bigger than look ahead distance. So we take the current goal as the intermediate goal
            self.intermediate_goal_x = self.waypoint_x
            self.intermediate_goal_y = self.waypoint_y

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

    def Callback_current_waypoint(self,msg_):
        self.current_waypoint = msg_.data

    def CallbackGps(self,GPS_msg):
        self.current_lat = GPS_msg.latitude
        self.current_lon = GPS_msg.longitude
    
    def CallbackImu(self,imu_msg):    
        self.quaternion_x  = imu_msg.orientation.x
        self.quaternion_y = imu_msg.orientation.y
        self.quaternion_z = imu_msg.orientation.z
        self.quaternion_w = imu_msg.orientation.w
        
if __name__ == '__main__':
    rospy.init_node('multi_vessel_controller', anonymous=True)
    hz = 10
    rate = rospy.Rate(hz)

    vessel_objects = []
    vessel_count = int(rospy.get_param("/vessel_count"))#get the number of how many vessels are spawned in this run
    print("Number of Vessels: ", str(vessel_count))

    for i in range(vessel_count):
        vessel_objects.append(Vessel_Controller_PurePursuit(i+1))

    for obj in vessel_objects:
        obj.start()