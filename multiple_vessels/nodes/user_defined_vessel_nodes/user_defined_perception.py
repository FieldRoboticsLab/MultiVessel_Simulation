#!/usr/bin/env python

from os import kill
import rospy
import time
import math
from rospy.core import rospydebug
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
import threading
import json
import os
from geopy import distance
from multivessel_msgs.msg import VesselPose,Perception

class Vessel_Subscriber():
    def __init__(self, vesselID_):

        self.is_on_global_path = 1
        self.vesselID = vesselID_

        #subscribe to sensors
        sub_topic_gps_txt = "vessel"+str(self.vesselID) +"/vessel"+str(self.vesselID)+"/sensors"+"/gps/gps/fix"
        rospy.Subscriber(sub_topic_gps_txt,NavSatFix, self.CallbackGps)

        sub_topic_gps_txt = "vessel"+str(self.vesselID) +"/vessel"+str(self.vesselID)+"/sensors"+"/imu/imu/data"
        rospy.Subscriber(sub_topic_gps_txt,Imu, self.CallbackImu)

    def CallbackGps(self,GPS_msg):
        self.current_lat = GPS_msg.latitude
        self.current_lon = GPS_msg.longitude    

    def CallbackImu(self,imu_msg):
        self.quaternion_x  = imu_msg.orientation.x
        self.quaternion_y = imu_msg.orientation.y
        self.quaternion_z = imu_msg.orientation.z
        self.quaternion_w = imu_msg.orientation.w

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

class UDV_Perception(threading.Thread):
    def __init__(self, VesselID_):
        threading.Thread.__init__(self)
        #Read the JSON config file
        self.path = "~/vrx_ws/src/vrx/multiple_vessels"
        self.full_path = os.path.expanduser(self.path)
        
        with open(self.full_path+'/json_files/config_user_defined.json','r') as f:    
            self.config_data = json.load(f)
        
        self.hz = 10
        self.rate = rospy.Rate(hz)
        self.Vessel_ID = VesselID_

        self.sensing_range = self.config_data["Localization"][0]["Subscribe_to_Topics"][0]["sensing_range"]
        #Sensing range for one vessel to sense another

        self.vessel_count = int(rospy.get_param("/vessel_count"))#get the number of how many Vessels do we spawn in this run
        print("Number of Vessels: ", str(self.vessel_count))
        self.user_defined_vessel_count = int(rospy.get_param("/user_defined_vessel_count")) 
        vesselID_of_udv =  vessel_count+1

        vessel_details_param_txt = "/vessel" + str(self.Vessel_ID)  + "_details"
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
        sub_topic_gps_txt = "vessel"+str(self.Vessel_ID) +"/vessel"+str(self.Vessel_ID)+"/sensors"+"/gps/gps/fix"
        rospy.Subscriber(sub_topic_gps_txt,NavSatFix, self.CallbackGps)
        sub_topic_imu_txt = "vessel"+str(self.Vessel_ID) +"/vessel"+str(self.Vessel_ID)+"/sensors"+"/imu/imu/data"
        rospy.Subscriber(sub_topic_imu_txt,Imu, self.CallbackImu)

        pub_topic_perception_txt = "vessel"+str(self.Vessel_ID) + "/perception_udv"
        self.pub_perception= rospy.Publisher(pub_topic_perception_txt,Perception,queue_size=10)

        self.Define_Vessel_sub_objects()

    def Define_Vessel_sub_objects(self):
        self.vessel_sub_objects = []
        self.total_vessel_count = self.vessel_count + self.user_defined_vessel_count
        print(self.total_vessel_count)
        
        i = 1
        print( "self.Vessel_ID",self.Vessel_ID)
        print("self.total_vessel_count",self.total_vessel_count)

        while i <self.total_vessel_count:
            if i != self.Vessel_ID:
                self.vessel_sub_objects.append(Vessel_Subscriber(i))
            i +=1

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

    def CallbackGps(self,GPS_msg):
        self.current_lat = GPS_msg.latitude
        self.current_lon = GPS_msg.longitude
    
    def CallbackImu(self,imu_msg):    
        self.quaternion_x  = imu_msg.orientation.x
        self.quaternion_y = imu_msg.orientation.y
        self.quaternion_z = imu_msg.orientation.z
        self.quaternion_w = imu_msg.orientation.w
    
    def run(self):
        while not rospy.is_shutdown():
            try:
                self.Perception_Iteration()
                self.rate.sleep()
            except Exception as e:
                pass
    
    def Perception_Iteration(self):
        self.Pose_List = []
        self.Perception_Message = Perception()
        
        for other_vessel in self.vessel_sub_objects:
            
            self.ourVesselPosition = (self.current_lat ,self.current_lon)
            self.other_vessel_lat = other_vessel.current_lat
            self.other_vessel_lon = other_vessel.current_lon
            otherVesselPosition = (self.other_vessel_lat,self.other_vessel_lon)
            distance_in_between = distance.distance(otherVesselPosition, self.ourVesselPosition).meters
            if distance_in_between < self.sensing_range:
                pose = VesselPose()
                vessel2_ID = other_vessel.vesselID
                #Log in the vessel details to the message.
                details_param_txt = "/vessel"+str(vessel2_ID)+"_details"
                vessel2_details = rospy.get_param(details_param_txt)
                pose.vessel_details.vessel_id = vessel2_ID
                pose.vessel_details.VesselName.data = vessel2_details["VesselName"]
                pose.vessel_details.CallSign.data = vessel2_details["CallSign"]
                pose.vessel_details.Cargo = vessel2_details["Cargo"]
                pose.vessel_details.Draft = vessel2_details["Draft"]
                pose.vessel_details.IMO.data = vessel2_details["IMO"]
                pose.vessel_details.Length = vessel2_details["Length"]
                pose.vessel_details.MMSI.data = vessel2_details["MMSI"]
                pose.vessel_details.TransceiverClass.data = vessel2_details["TransceiverClass"]
                pose.vessel_details.VesselType = vessel2_details["VesselType"]
                pose.vessel_details.Width = vessel2_details["Width"]

                pose.vessel_id = int(vessel2_ID)
                pose.position.x = self.other_vessel_lat
                pose.position.y = self.other_vessel_lon
                pose.position.z = 0 # Since Vessels have Lat and Lon informations and they should not fly, altitude information is set to zero.

                pose.orientation.x = other_vessel.quaternion_x
                pose.orientation.y = other_vessel.quaternion_y
                pose.orientation.z = other_vessel.quaternion_z
                pose.orientation.w = other_vessel.quaternion_w

                self.Pose_List.append(pose)

        self.Perception_Message.vessel_poses = self.Pose_List

        self.pub_perception.publish(self.Perception_Message)
        print("self.Perception_Message",self.Perception_Message)

if __name__ == '__main__':
    rospy.init_node('Perception_Pose_Aggregator_Node', anonymous=True)
    hz = 10
    rate = rospy.Rate(hz)

    vessel_objects = []
    vessel_count = int(rospy.get_param("/vessel_count"))#get the number of how many Vessels do we spawn in this run
    print("Number of Vessels: ", str(vessel_count))

    user_defined_vessel_count = int(rospy.get_param("/user_defined_vessel_count")) 
    vesselID_of_udv =  vessel_count+1

    i=1
    while i <= user_defined_vessel_count:
        print("vesselID_of_udv",vesselID_of_udv)
        vessel_objects.append(UDV_Perception(vesselID_of_udv))
        i+=1
        vesselID_of_udv +=1

    for obj in vessel_objects:
        obj.start()