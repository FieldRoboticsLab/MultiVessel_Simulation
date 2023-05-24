#!/usr/bin/env python
"""
Perception node for vessels.
"""

from os import kill
import rospy
import math
from rospy.core import rospydebug
from std_msgs.msg import Float64MultiArray, Int64MultiArray
import threading
import json
from multivessel_msgs.msg import VesselPose,Perception
import os

class Perception_Thread(threading.Thread):

    def __init__(self, distance_measurementListForThread_, sensing_range_):
        threading.Thread.__init__(self)
        self.distance_measurementListForThread = distance_measurementListForThread_
        self.sensing_range = sensing_range_#Sensing range for vessels seeing each other

        self.vessel_count = int(rospy.get_param("/vessel_count"))
        self.user_defined_vessel_count = int(rospy.get_param("/user_defined_vessel_count")) 
        self.vessel_count =  self.vessel_count+self.user_defined_vessel_count
        print("Number of Vessels including udv: ", str(self.vessel_count))

        self.gps_list = ""
        self.imu_list = "" 

        self.current_x = ""
        self.current_y = ""

        self.target_x = ""
        self.target_y = ""

        self.roll = ""
        self.pitch = ""
        self.yaw = ""

        self.quaternion_x = ""
        self.quaternion_y = ""
        self.quaternion_z = ""
        self.quaternion_w = ""
        hz = 20
        self.rate = rospy.Rate(hz)

        self.iteration_complete = False
        #Bool variable to store the information of is an iteration in the Check_Distance is completed.
        #This variable will be False in the constructor because no iteration is even got started yet.

        self.Thread_Msg_CalledInMain = False
        #This bool variable serves for the same purpose as the one above.
        #However, we still added this flag to moderate our iterations as well.
        #Because if the freqency of thread is higher than the frequency of main function, the self.iteration_complete bool flag can work properly.
        #But if that's not the case, we can still prevent the loss of information by self.Thread_Msg_CalledInMain bool flag.

        #subscribe to sensors
        sub_topic_gps_txt = "simulation/gps_list"
        rospy.Subscriber(sub_topic_gps_txt,Float64MultiArray, self.CallbackGpsList)
        
    def run(self):

        while not rospy.is_shutdown():
            try:
                self.Check_Distance()
                self.rate.sleep()
            except Exception as e:
                pass
        
    def CallbackGpsList(self,gpsList_msg):
        self.gps_list = gpsList_msg.data

    def Check_Distance(self):
        #Checks the coordinates list and calculates distance in between vessels.
        #If distance is smaller than sensing range defined in JSON, print to terminal that vesselX and vesselY are closer than M meters.
        print("self.distance_measurementListForThread",self.distance_measurementListForThread)
        print("int(len(self.distance_measurementListForThread))",int(len(self.distance_measurementListForThread)))
        while not rospy.is_shutdown():
            try:

                self.iteration_complete = False
                #If the program is running in this for loop below, that means one iteration is not yet completed.
                #Therefore, we set the self.iteration_complete variable False here.
                
                self.origin_lat = self.gps_list[0]
                self.origin_lon = self.gps_list[0]
                self.i = 0
                self.Thread_perceptionTopicMsg = []#List for storing the data to send to Perception Topic
 
                for i in range(0,int(len(self.distance_measurementListForThread)),2):
                  
                    self.vessel1_ID = int(self.distance_measurementListForThread[i])
                    self.vessel2_ID = int(self.distance_measurementListForThread[i+1])
                    
                    lat1 = self.gps_list[self.vessel1_ID*2 -2]
                    lon1 = self.gps_list[self.vessel1_ID*2 -1]
                    lat2 = self.gps_list[self.vessel2_ID*2 -2]
                    lon2 = self.gps_list[self.vessel2_ID*2 -1]

                    x1,y1 = self.get_local_coord(lat1,lon1)
                    x2,y2 = self.get_local_coord(lat2,lon2)

                    self.distance = math.sqrt((y2 - y1)**2 + (x2 - x1)**2)

                    if self.distance < self.sensing_range:
                    
                        self.Append_to_Perception_List(self.vessel1_ID,self.vessel2_ID)

                self.iteration_complete = True
                #If program manages to get to here, that means the for loop above is completed. 
                #That means one iteration is completed and every distance_measurement that assigned to this thread has been made.
                #Therefore, self.Thread_perceptionTopicMsg list is completed for this thread.
                self.rate.sleep()

                while self.Thread_Msg_CalledInMain == False:
                    self.rate.sleep()
                    print("Sleeping")
                    #If the self.Thread_perceptionTopicMsg list is not yet accessed from main function, don't proceed to the next iteration and wait.

            except Exception as e:
                print(e)
                self.rate.sleep()

    def Append_to_Perception_List(self, vessel1_ID_, vessel2_ID_):
        #If perception list is choosen to work with, this function will run
        self.Thread_perceptionTopicMsg.append(vessel1_ID_)
        self.Thread_perceptionTopicMsg.append(vessel2_ID_)

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
      
class Pose_Publisher():
    def __init__(self, vesselID_,vessel_count_):
        self.vessel_ID = vesselID_
        self.vessel_count = vessel_count_
        self.gps_list = []
        self.imu_list = []
        
        sub_topic_gps_txt = "simulation/gps_list"
        rospy.Subscriber(sub_topic_gps_txt,Float64MultiArray, self.CallbackGpsList)

        sub_topic_imu_txt = "simulation/imu_list"
        rospy.Subscriber(sub_topic_imu_txt,Float64MultiArray, self.CallbackImuList)

    def CallbackGpsList(self,gpsList_msg):
        self.gps_list = gpsList_msg.data

    def CallbackImuList(self,gpsList_msg):
        self.imu_list = gpsList_msg.data
        
    def Convert_to_Perception_msg(self,perception_list_):
        self.perception_list = perception_list_
        self.Perception_Message = Perception()

        self.Pose_List = []

        for i in range(len(self.perception_list)):
            vesselID_ = self.perception_list[i]
            if self.vessel_ID == vesselID_:#if an element of the self.perception_list is the same as the own VesselID of an object
                if i % 2 == 0:#If the index number of the element is an even number
                    vessel2_ID = self.perception_list[i+1]
                elif i % 2 != 0: #IF the index numbre of the element is an odd number  
                    vessel2_ID = self.perception_list[i-1]

                pose = VesselPose()

                lat1 = self.gps_list[vessel2_ID*2 -2]
                lon1 = self.gps_list[vessel2_ID*2 -1]

                orientation_x = self.imu_list[vessel2_ID*4 -4]
                orientation_y = self.imu_list[vessel2_ID*4 -3]
                orientation_z = self.imu_list[vessel2_ID*4 -2]
                orientation_w = self.imu_list[vessel2_ID*4 -1]

                pose.vessel_id = int(vessel2_ID)
                pose.position.x = lat1
                pose.position.y = lon1
                pose.position.z = 0 # Since Vessels have Lat and Lon informations and they should not fly, altitude information is set to zero.

                pose.orientation.x = orientation_x
                pose.orientation.y = orientation_y
                pose.orientation.z = orientation_z
                pose.orientation.w = orientation_w

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

                self.Pose_List.append(pose)

        self.Perception_Message.vessel_poses = self.Pose_List

    def Publish_Perception_msg(self,perception_list_):
        self.perception_list = perception_list_
        self.Convert_to_Perception_msg(self.perception_list)
        
        pub_topic_perception_txt = "vessel"+str(self.vessel_ID) + "/perception"
        self.pub_perception= rospy.Publisher(pub_topic_perception_txt,Perception,queue_size=10)

        self.pub_perception.publish(self.Perception_Message)

if __name__ == '__main__':
    rospy.init_node('perception_slave_threads', anonymous=True)

    path = "~/vrx_ws/src/vrx/multiple_vessels"
    full_path = os.path.expanduser(path)

    with open(full_path+'/json_files/config.json','r') as f:    
            config_data = json.load(f)

    sensing_range = config_data["Localization"][0]["Subsribe_to_Topics"][0]["sensing_range"]

    with open('Perception_Distance_Measurement_List.json','r') as infile:
        distance_measurementList = json.load(infile)

    print("distance_measurementList is: ",distance_measurementList)

    vessel_count = int(rospy.get_param("/vessel_count"))
    
    thread_objects = []
    number_of_threads = len(distance_measurementList)
    print(distance_measurementList ,number_of_threads)

    for i in range(number_of_threads):
        thread_objects.append(Perception_Thread(distance_measurementList[i], sensing_range))

    for obj in thread_objects:
        obj.start()

    perception_topic_hz = 30
    rate2 = rospy.Rate(perception_topic_hz)

    pub_perceptionTopic_txt = "simulation/perception_list"
    pub_perceptionTopic= rospy.Publisher(pub_perceptionTopic_txt,Int64MultiArray,queue_size=10)

    publisher_objects = []
    for i in range(vessel_count):
        publisher_objects.append(Pose_Publisher(int(i+1),vessel_count))

    while not rospy.is_shutdown():
        try:
    
                perception_topic_list = []
                perception_topic_msg = Int64MultiArray()
                
                for obj in thread_objects:
                    while obj.iteration_complete == False:
                        rate2.sleep()
                
                for obj in thread_objects:
                    obj.Thread_Msg_CalledInMain = False

                for obj in thread_objects:
                    perception_topic_list.extend( obj.Thread_perceptionTopicMsg)
                    obj.Thread_Msg_CalledInMain = True

                perception_topic_msg.data = perception_topic_list
                pub_perceptionTopic.publish(perception_topic_msg)
                print("Perception list is:", perception_topic_msg)

                for obj in publisher_objects:
                    obj.Publish_Perception_msg(perception_topic_list)

                rate2.sleep()
    
        except Exception as e:
            print(e)
            rate2.sleep()
            pass

    rospy.spin()