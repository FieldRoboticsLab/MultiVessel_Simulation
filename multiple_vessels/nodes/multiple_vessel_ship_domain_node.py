#!/usr/bin/env python
"""
This node is the Vessel ship_domain node
for every spawned Vessel in the sim environment.

In this ROS node, Vessel_ship_domain class is defined for every Vessel that's available in the 
Gazebo simulation environment.

And it subscribes to /VesselX/Perception topic. And sensor topics for it's vessel /VesselX/GPS and /VesselX/AHRS 
If one or more vessels enter to the ship domain, put that information into /VesselX/Vessels_In_Ship_Domain topic.
Local motion planner node is subscribed to that topic and it will generate a local path if a change occurs.
"""

from os import kill
import rospy
import time
import math
from rospy.core import rospydebug
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
import threading
import json
from geopy import distance
import os
from multivessel_msgs.msg import VesselPose,Perception

class Vessel_ship_domain(threading.Thread):
    def __init__(self, VesselID_):
        threading.Thread.__init__(self)
        
        self.path = "~/vrx_ws/src/vrx/MultiVessel_Simulation/multiple_vessels"
        self.full_path = os.path.expanduser(self.path)
        
        with open(self.full_path+'/json_files/config.json','r') as f:    
            self.config_data = json.load(f)

        self.vesselID = VesselID_

        self.ship_domain_radius_coefficient_length = self.config_data["ship_domain"][0]["Circular_Ship_Domain"][0]["ship_domain_radius_coefficient_length"]
        self.ship_domain_radius_coefficient_width = self.config_data["ship_domain"][0]["Circular_Ship_Domain"][0]["ship_domain_radius_coefficient_width"]
        self.ship_domain_radius_coefficient_draft = self.config_data["ship_domain"][0]["Circular_Ship_Domain"][0]["ship_domain_radius_coefficient_draft"]
        self.ship_domain_radius_coefficient_velocity = self.config_data["ship_domain"][0]["Circular_Ship_Domain"][0]["ship_domain_radius_coefficient_velocity"]
        self.ship_domain_radius_equation = self.config_data["ship_domain"][0]["Circular_Ship_Domain"][0]["ship_domain_radius_equation"]
        
        #Load our own vessel's details. Because it's necessary to calculate the ship domain's radius.
        vessel_details_param_txt = "/vessel" + str(self.vesselID)  + "_details"
        self.vessel_details = rospy.get_param(vessel_details_param_txt)

        if self.ship_domain_radius_equation == "L*Lc": #Calculate the ship domain radius for this equation if it's chosen
            self.ship_domain_radius = self.vessel_details["Length"]*self.ship_domain_radius_coefficient_length
            rospy.set_param('vessel'+str(self.vesselID)+"_Ship_Domain_Radius", self.ship_domain_radius)
        
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

        self.perception_msg = Perception()
        self.perception_msg_now = Perception()
        self.vesselsInShipDomain = Perception() 

        #subscribe to GPS and IMU sensors
        sub_topic_gps_txt = "vessel"+str(self.vesselID) +"/vessel"+str(self.vesselID)+"/sensors"+"/gps/gps/fix"
        rospy.Subscriber(sub_topic_gps_txt,NavSatFix, self.CallbackGps)

        sub_topic_imu_txt = "vessel"+str(self.vesselID) +"/vessel"+str(self.vesselID)+"/sensors"+"/imu/imu/data"
        rospy.Subscriber(sub_topic_imu_txt,Imu, self.CallbackImu)

        #Subscribe to Perception topic
        sim_Perception_topic_txt = "vessel"+str(self.vesselID)+"/perception"
        rospy.Subscriber(sim_Perception_topic_txt,Perception, self.CallbackPerception)
        
        #init the vessels_in_ship_domain publisher
        pub_vessels_in_ship_domain_txt = "vessel"+str(self.vesselID) + "/vessels_in_ship_domain"
        self.pub_vessels_in_ship_domain= rospy.Publisher(pub_vessels_in_ship_domain_txt,Perception,queue_size=10)

        self.cnt = 0
        print(str(self.vesselID)," vessel ship domain started")

        self.isThereAnyVesselsInPerception = True
        
    def run(self):
        time.sleep(1)
        while not rospy.is_shutdown():
            try:
                
                self.Calculate_Vessels_In_Circular_Ship_Domain()
                self.Publish_Vessels_In_Ship_Domain()
            except Exception as e:
                rospy.logerr(e)
                
            rate.sleep()

    def Calculate_Vessels_In_Circular_Ship_Domain(self):
        #This function measures the distances in between own vessel to other vessels in our own vessel's perception.
        #Put the vessel poses taken from Perception topic to a new list. That list is called perception_msg_now
        #Put our own vessel pose to a new variable called ownVesselPose. ownVesselPose = [Lat1,Lon1]
        #Look through the perception_msg list and measure the distance in between perception_msg[i] and ownVesselPose
        #   If the distance between perception_msg[i] and ownVesselPose is bigger than self.ship_domain_radius, pop the perception_msg[i]

        self.perception_msg_now = self.perception_msg
        self.vessel_poses = self.perception_msg_now.vessel_poses
        ourVesselPosition = (self.current_lat ,self.current_lon)

        self.vessel_poses_in_ship_domain = []

        if self.vessel_poses != []:
            self.isThereAnyVesselsInPerception =True

            for i in range(len(self.vessel_poses)):
                other_vessel_lat = self.vessel_poses[i].position.x
                other_vessel_lon = self.vessel_poses[i].position.y
                otherVesselPosition = (other_vessel_lat,other_vessel_lon)
                distance_in_between = distance.distance(otherVesselPosition, ourVesselPosition).meters
                if distance_in_between < self.ship_domain_radius:
                    self.vessel_poses_in_ship_domain.append( self.vessel_poses[i] )

        elif self.vessel_poses == []:
            #no vessels in perception
            self.isThereAnyVesselsInPerception = False
          
    def Publish_Vessels_In_Ship_Domain(self):
        #This function publishes to the "vessel"+str(self.vesselID) + "/vessels_in_ship_domain" topic after calculations are completed.
        self.vesselsInShipDomain.vessel_poses = self.vessel_poses_in_ship_domain
        if (len(self.vessel_poses) == 0) or (self.isThereAnyVesselsInPerception == False):
            #print("No vessels in ship domain for vessel"+str(self.vesselID))
            self.pub_vessels_in_ship_domain.publish(self.vesselsInShipDomain)
            pass
        elif len(self.vessel_poses) != 0:
            self.pub_vessels_in_ship_domain.publish(self.vesselsInShipDomain)    
  
    def CallbackPerception(self,perception_msg_):
        self.perception_msg = perception_msg_

    def CallbackGps(self,GPS_msg):
        self.current_lat = GPS_msg.latitude
        self.current_lon = GPS_msg.longitude
    
    def CallbackImu(self,imu_msg):    
        self.quaternion_x  = imu_msg.orientation.x
        self.quaternion_y = imu_msg.orientation.y
        self.quaternion_z = imu_msg.orientation.z
        self.quaternion_w = imu_msg.orientation.w
     
    def get_local_coord(self, lat, lon):
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

    def euler_from_quaternion(self, x, y, z, w):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)

            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)

            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)

            return roll_x, pitch_y, yaw_z # in radians

if __name__ == '__main__':
    rospy.init_node('multi_vessel_ship_domain', anonymous=True)
    hz = 10
    rate = rospy.Rate(hz)

    vessel_objects = []
    vessel_count = int(rospy.get_param("/vessel_count"))#get the number of how many Vessels do we spawn in this run
    print("Number of Vessels: ", str(vessel_count))

    for i in range(vessel_count):
      vessel_objects.append(Vessel_ship_domain(i+1))

    for obj in vessel_objects:
        obj.start()
