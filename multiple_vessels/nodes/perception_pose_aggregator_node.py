#!/usr/bin/env python
"""
Perception node for vessels.
This ROS node is written in order to emulate vessels' sensing capability. 
In order to reduce the required processing power to run this simulation, built-in LIDARs in vessels are disabled.
Instead, GPS locations of vessels are published to each vessels controller if the distance in between two vessel is smaller than a pre-defined sensing range.
"""

"""
Perception_Pose_Aggregator_Node and Perception_Detect_Vessels_In_Sensing_Range_Node ROS nodes serve for this purpose. 

Perception_Pose_Aggregator_Node node has two classes. Threader class and Vessel_Subscriber class.

Vessel_Subscriber class has one argument. It is vesselID_. 
vesselID_ argument defines the ID number of vessel to subscribe it's GPS topics.
One Vessel_Subscriber object subscribes to one vessel's GPS topics.
A list of Vessel_Subscriber objects are defined in the Threader object. 
Therefore, it is possible to store coordinates of every vessel in the simulation environment in one ROS topic.

Threader class is defined to use to define one object in the main function. 
Threader object takes no arguments. Instead, it accesses to config.json file for necessary values.,

NOTE: For Perception_Pose_Aggregator_Node and Perception_Detect_Vessels_In_Sensing_Range_Node ROS nodes, the word "distance_measurement" means calculating the distance in between two vessels. 

Variables which get set their values from config.json file are:
    self.multiThreadBool : Whether if the perception should be run in multi-threads
    self.multiThread_NumberOfDistanceMeasurementsForEachThread : Number of distance_measurement For Each Thread. 


"""

from os import kill
import rospy
import time
import math
from rospy.core import rospydebug
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import json
import os

class Vessel_Subscriber():
    def __init__(self, vesselID_):
        
        self.current_lat = ""
        self.current_lon = ""
        
        self.vesselID = vesselID_
        self.got_origin = False

        self.current_x = ""
        self.current_y = ""
        
        self.origin_lat = ""
        self.origin_lon = ""
        self.got_origin = False
        
        self.desired_yaw = ""
        self.yaw_error = ""

        self.roll = ""
        self.pitch = ""
        self.yaw = ""

        self.quaternion_x = ""
        self.quaternion_y = ""
        self.quaternion_z = ""
        self.quaternion_w = ""

        #subscribe to sensors
        sub_topic_gps_txt = "vessel"+str(self.vesselID) +"/vessel"+str(self.vesselID)+"/sensors"+"/gps/gps/fix"
        rospy.Subscriber(sub_topic_gps_txt,NavSatFix, self.CallbackGps)

        sub_topic_gps_txt = "vessel"+str(self.vesselID) +"/vessel"+str(self.vesselID)+"/sensors"+"/imu/imu/data"
        rospy.Subscriber(sub_topic_gps_txt,Imu, self.CallbackImu)
        
        self.cnt = 0
        print(str(self.vesselID)," vessel tracker started")

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

class Threader():
    def __init__(self):
        #Read the JSON config file
        self.path = "~/vrx_ws/src/vrx/multiple_vessels"
        self.full_path = os.path.expanduser(self.path)
        
        with open(self.full_path+'/json_files/config.json','r') as f:    
            self.config_data = json.load(f)
        
        self.hz = 10
        self.rate = rospy.Rate(hz)
        
        self.multiThreadBool = self.config_data["Localization"][0]["MultiThreading"][0]["chosen"]
        self.multiThread_NumberOfDistanceMeasurementsForEachThread = self.config_data["Localization"][0]["MultiThreading"][0]["Number_Of_Distance_Measurements_For_Each_Thread"] 
        self.multiThread_NumberOfDistanceMeasurementsForEachThread_choosen = self.config_data["Localization"][0]["MultiThreading"][0]["Number_Of_Distance_Measurements_For_Each_Thread_chosen"]
        #instead of specifying number of threads method.
        #If this variable is false, that means user choose to specify the number of threads.
        self.multiThread_number_of_threads = self.config_data["Localization"][0]["MultiThreading"][0]["number_of_threads"] #Number of threads

        self.sensing_range = self.config_data["Localization"][0]["Subsribe_to_Topics"][0]["sensing_range"]
        #Sensing range for one vessel to sense another

        self.vessel_objects = []
        self.vessel_count = int(rospy.get_param("/vessel_count"))#get the number of how many Vessels do we spawn in this run
        print("Number of Vessels: ", str(self.vessel_count))
        self.user_defined_vessel_count = int(rospy.get_param("/user_defined_vessel_count")) 
        self.vessel_count =  self.vessel_count+self.user_defined_vessel_count
        print("Number of Vessels including udv: ", str(self.vessel_count))
        
        self.Generate_Distance_Measurements_List()
        self.Vessel_objects()
        self.Thread_Calculations()

    def Generate_Distance_Measurements_List(self):
        a = 1
        b = 2
        self.distance_measurementList = []
        #This list stores all the possible distance measurements for the perception node.
        #For example, if we have 10 vessels in the simulation, self.distance_measurementList should be like:
        #[1,2,1,3,1,4,1,5,1,6,1,7,1,8,1,9,1,10,2,3,2,4 etc.]
        while a < self.vessel_count:
            if b <= self.vessel_count:

                self.distance_measurementList.append(a)
                self.distance_measurementList.append(b)
                print(a,b)
                b +=1
            elif b > self.vessel_count:
                a +=1
                b = a+1
                print("\n")
            
        print(self.distance_measurementList)
        self.distance_measurement_count = len(self.distance_measurementList) / 2
        print("The amount of distance measurements in each iteration: ", str(self.distance_measurement_count)) 

    def Thread_Calculations(self):
        #Now, to set it multi-thread, we need to calculate how many distance measurements of checking distance in between vessels are there
        #It's quite simple. Let's say we want to check distances in between Vessel-1 and every other vessel. We just check V1-V2, V1-V3,V1-V4 etc.
        #And when we are done for Vessel-1 and proceed to Vessel-2, we don't need to check V2-V1 back again. Since it's the same with V1-V2
        #Therefore, we can reduce the amount of distance measurements to be done in each iteration. 
        #Let's say the vessel count is 10. Therefore, the amount of distance measurements for each iteration is 9+8+7+6+5+4+3+2+1

        if self.multiThreadBool == True:#If user wants to run the perception node multi-threaded

            if self.multiThread_NumberOfDistanceMeasurementsForEachThread_choosen == True:#If user wants to specify distance measurements count in each thread and initialize threads that way
            #Calculate the amount of threads
                self.number_of_threads = int(self.distance_measurement_count  / self.multiThread_NumberOfDistanceMeasurementsForEachThread) 
                self.p_c_modulo = self.distance_measurement_count % self.multiThread_NumberOfDistanceMeasurementsForEachThread # modulo of distance measurements count 
                if self.p_c_modulo != 0:
                    self.number_of_threads +=1
            
            #We do that because if remainder in division is not zero, that means we need to put one more thread in action.
            #Let's say there are 11 vessels and I want to operate 10 distance measurements in each thread. 
            # 11 vessels means 55 distance measurements . 5 Threads for 10 distance measurements  each equals to 50. 
            # We need one more thread that includes 5 distance measurements  to cover the rest of it. Therefore 6 threads in total.
                elif self.p_c_modulo == 0:
                    pass
                
                self.Initialize_Threads(self.number_of_threads,self.multiThread_NumberOfDistanceMeasurementsForEachThread,self.p_c_modulo)
            
            elif self.multiThread_NumberOfDistanceMeasurementsForEachThread_choosen == False:#If user wants to specify the number of threads
                self.dmcfen_modulo = self.distance_measurement_count % self.multiThread_number_of_threads # modulo of distance_measurement count to number of threads
                if self.dmcfen_modulo == 0 :
            #if distance_measurement count perfectly divides into number of threads, no problem.
                    self.distance_measurement_count_for_each_thread = int(self.distance_measurement_count / self.multiThread_number_of_threads)
                elif self.dmcfen_modulo != 0 :
            #if distance_measurement count doesn't perfectly divides into number of threads, there are a few more steps to calculate
            #how many distance_measurements should we assign to each thread.
                    self.distance_measurement_count_for_each_thread  = int(self.distance_measurement_count / self.multiThread_number_of_threads)
                    self.distance_measurement_count_for_each_thread  +=1
                    self.dmcfen_modulo = self.distance_measurement_count % self.multiThread_number_of_threads
            #In order to undestand why did we do the last part, let's see an example.
            #Let's say that we want to track 11 vessels with 10 threads. There are 55 distance_measurement to be done in each iteration.
            #When we divide 55/10, it says 5 distance_measurements for each thread but 5 more distance_measurements to be done. 
            #Since we want to keep the number of threads the same, we increase the distance_measurement count for eacht thread by one.
            #Therefore, it's 6 distance_measurements for each thread except the last thread. Last thread has 1 distance_measurement. 
            #6 times 9 threads is 54 distance_measurements. The 1 distance_measurement left is assigned to the last thread.
            
            self.Initialize_Threads(self.multiThread_number_of_threads,self.distance_measurement_count_for_each_thread,self.dmcfen_modulo)
        elif self.multiThreadBool == False:#If user doesn't want to run the perception node multi-threaded
            self.number_of_threads = 1
            self.Initialize_Threads(self.number_of_threads,self.distance_measurement_count,0)
            
    def Initialize_Threads(self, number_of_threads_,distance_measurement_for_each_thread_,modulo_):
        #In this function, we slice the self.self.distance_measurementList for the purpose of 
        #dividing our whole program into threads.
        #This function takes number of threads, number of distance_measurement count for each thread and number of excess distance_measurement (modulo)
  
        print("Number of threads: ", number_of_threads_)
        print("istance_measurement count for each thread: ", distance_measurement_for_each_thread_)
        print("Excess istance_measurements (modulo): ", modulo_)

        #while not rospy.is_shutdown():
            #try:
        #while True:
        self.distance_measurementListForThread = []
        self.thread_objects = []
        self.coordinates_list = []
        
        #https://www.learnbyexample.org/python-list-slicing/
        
        a = int(0)
        b = int(distance_measurement_for_each_thread_)*2 
        
        if modulo_ == 0:
            for i in range(number_of_threads_):
                self.distance_measurementListForThread.append(self.distance_measurementList[a:b])
                
                a += distance_measurement_for_each_thread_*2
                b += distance_measurement_for_each_thread_*2  
        
        elif modulo_ != 0:
            for i in range(number_of_threads_ - 1):
                self.distance_measurementListForThread.append(self.distance_measurementList[a:b])
                a += distance_measurement_for_each_thread_*2
                b += distance_measurement_for_each_thread_*2
            
            listLenght = len(self.distance_measurementList) 
            self.modulo_list = self.distance_measurementList[int(listLenght - modulo_*2):listLenght]
            #We need the self.modulo_list list above since we need to append to self.distance_measurementListForThread list and publish this lists ingredients.

            self.distance_measurementListForThread.append(self.modulo_list)

        print("self.distance_measurementListForThread: ",self.distance_measurementListForThread)

        #Publish to the distance_measurementList topics.
        #Ingredients of self.distance_measurementListForThread list will be published to topics such as:
        #/simulation/distance_measurementList1
        #/simulation/distance_measurementList2
        #/simulation/distance_measurementList3
        #In this part of the ROS node, we will only initialize the topics to publish information.
        #We will publish to these topics in the loop at the ROS_Topic_Publish_Loop function
        self.topic_txt=[]
        self.pub_leftThrustcmd=[]
        
        print(int(len(self.distance_measurementListForThread)))
        #After we complete to generate the istance_measurement list, we can write it to a JSON file
        #https://stackoverflow.com/questions/19460457/how-can-i-write-a-list-of-lists-into-a-txt-file

        with open('Perception_Distance_Measurement_List.json','w') as myfile:
            json.dump(self.distance_measurementListForThread,myfile)

        os.system("gnome-terminal -- bash -c \"source ~/vrx_ws/devel/setup.bash;python perception_detect_vessels_in_sensing_range_node.py; bash\" ")
       
        self.ROS_Topic_Publish_Loop()

    def Vessel_objects(self):
        self.vessel_objects = []
        #i =1
        print(self.vessel_count)
        
        for i in range(self.vessel_count):
            self.vessel_objects.append(Vessel_Subscriber(i+1))

    def Coord_List_Generator(self):
        self.coordinates_list = []

        for i in range(int(len(self.vessel_objects))):
            self.coordinates_list.append( self.vessel_objects[i].current_lat )
            self.coordinates_list.append( self.vessel_objects[i].current_lon )

        print("self.coordinates_list",self.coordinates_list)

    def Orientation_List_Generator(self):
        self.orientation_list = []

        for i in range(int(len(self.vessel_objects))):
            self.orientation_list.append( self.vessel_objects[i].quaternion_x )
            self.orientation_list.append( self.vessel_objects[i].quaternion_y )
            self.orientation_list.append( self.vessel_objects[i].quaternion_z )
            self.orientation_list.append( self.vessel_objects[i].quaternion_w )

        print("self.orientation_list",self.orientation_list)

    def ROS_Topic_Publish_Loop(self):
        self.gps_topic_txt = "simulation/gps_list"
        self.gps_topic_pub = rospy.Publisher(self.gps_topic_txt, Float64MultiArray, queue_size=10)

        self.imu_topic_txt = "simulation/imu_list"
        self.imu_topic_pub = rospy.Publisher(self.imu_topic_txt, Float64MultiArray, queue_size=10)

        gps_list = Float64MultiArray()
        imu_list = Float64MultiArray()

        while not rospy.is_shutdown():
            try:
        
                #Publish every vessel's GPS sensor data in a list.
                self.Coord_List_Generator()
                gps_list.data = self.coordinates_list
                self.gps_topic_pub.publish(gps_list)

                self.rate.sleep()
                #Publish every vessel's IMU sensor data in a list.
                self.Orientation_List_Generator()
                imu_list.data = self.orientation_list
                self.imu_topic_pub.publish(imu_list)

                self.rate.sleep()
                
            except Exception as e:
                print(e)

if __name__ == '__main__':
    rospy.init_node('Perception_Pose_Aggregator_Node', anonymous=True)
    hz = 10
    rate = rospy.Rate(hz)

    my_threader_object = Threader()

    rospy.spin()
