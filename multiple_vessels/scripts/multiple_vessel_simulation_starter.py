#!/usr/bin/env python
"""
This Python script reads the configs.json file and 
starts the simulation environment.

The procedure of starting the simulation environment is:
    * Set all the required ROS parameters for simulation environment.
        Such as:
            Vessel count,  
            Vessel information for every vessel (such as height, width, name etc.), 
            local path planner details (RRT in this case)
            Global path planner details (User specified path or path derived from AIS data)
            Vessel controller parameters (either the simulation uses PID, Pure pursuit or any control loop algorithm)
"""
#source ~/vrx_ws/devel/setup.bash

import os
from os import kill
import subprocess
import rospy
import time
import math
from rospy.core import rospydebug
from geometry_msgs.msg import Quaternion
import json
import roslaunch
import xml.etree.ElementTree as ET
from geopy import distance


class Simulation():
    def __init__(self):
        self.path = "~/vrx_ws/src/vrx/multiple_vessels"
        self.full_path = os.path.expanduser(self.path)
        
        with open(self.full_path+'/json_files/config.json','r') as f:    
            self.config_data = json.load(f)

        with open(self.full_path+'/json_files/config_user_defined.json','r') as f:    
            self.config_data_udv = json.load(f)

        with open(self.full_path+'/json_files/Json_Global_Waypoints.json','r') as f:    
            self.global_wp_data = json.load(f)
        
        os.system("export IGN_IP=127.0.0.1")
        
        self.vessel_count = self.config_data["simulation"][0]["number_of_vessels"]
        self.number_of_user_defined_vessels = self.config_data["simulation"][0]["number_of_user_defined_vessels"]

        print(self.vessel_count)

        rospy.set_param('vessel_count', self.vessel_count)#sets the number of vessels in the vessel_count ROS parameter
        rospy.set_param('user_defined_vessel_count', self.number_of_user_defined_vessels )#sets the number of user_defined_vessels in the vessel_count ROS parameter

        self.Local_Path_Planner_setParam()
        self.UDV_Local_Path_Planner_setParam()
        self.World_Details_setParam()
        self.Vessel_Details_setParam() # New function for setting up vessel details from real AIS data.

        if self.number_of_user_defined_vessels >0:
            with open(self.full_path+'/json_files/Global_Waypoints_userDefinedVessels.json','r') as f:    
                self.global_wp_data_userDefinedVessels = json.load(f)
            self.User_Defined_Vessel_Details_setParam()
        
        #self.Vessel_Controller_setParam()

        self.path = "~/vrx_ws/src/vrx/multiple_vessels"
        self.full_path = os.path.expanduser(self.path)
        self.UsvSpawner()

        #subprocess.run(["roslaunch "+ self.full_path+"/vrx_multivessel_parametric.launch"])

        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [self.full_path+"/launch/vrx_multivessel_parametric.launch"])
        launch.start()
        launch.spin()


    def World_Details_setParam(self):
        #This function sets the Gazebo simulation world's parameters by reading from the Json_Global_Waypoints.json file.
        #Edits the multiple_vessels.world file with xmltree
        #https://www.youtube.com/watch?v=bWfAD7wAfOI

        self.world_path = "~/vrx_ws/src/vrx/multiple_vessels/multiple_vessels_worlds"
        self.full_world_path =  os.path.expanduser(self.world_path)
        self.xmlFile = self.full_world_path + "/multiple_vessels.world"
        self.tree = ET.parse(self.xmlFile)
        self.root = self.tree.getroot()



        json_origin_lat = self.global_wp_data["OriginLat"]
        json_origin_lon = self.global_wp_data["OriginLon"]

        for element in self.root.findall("./world[@name ='multiVessel_world']/spherical_coordinates/latitude_deg"):
            element.text = str(json_origin_lat)
        
        for element in self.root.findall("./world[@name ='multiVessel_world']/spherical_coordinates/longitude_deg"):
            element.text = str(json_origin_lon)

        #from topright_latlon and botleft_latlon, calculate the size of the simulation environment.
        json_TopRightLat = self.global_wp_data["TopRightLat"]
        json_TopRightLon = self.global_wp_data["TopRightLon"]
        json_BotLeftLat = self.global_wp_data["BotLeftLat"]
        json_BotLeftLon = self.global_wp_data["BotLeftLon"]

        self.TopRightLatLon = (json_TopRightLat,json_TopRightLon)
        self.BotRightLatLon = (json_BotLeftLat,json_TopRightLon)
        self.BotLeftLatLon = (json_BotLeftLat,json_BotLeftLon)

        self.distance_of_y= distance.distance(self.TopRightLatLon, self.BotRightLatLon ).km
        self.distance_of_x = distance.distance(self.BotLeftLatLon, self.BotRightLatLon ).km
        #You might have a question about how did we know to assign these values.
        #Here is the explanation:
        """
        So, JSON file provides us TopRightLatLon and BotLeftLatLon points.
        So we are supposed to measure the distance in between the TopRight point and BotRight point to find the distance we set for North.
        And in order to find the distance to set for East, we measure the distance in between botLeft and botRight points.
        Since Gazebo uses East, North, Up coordinates; we set the North value to y and East value to x.
        """
        mapMeasures = str(self.distance_of_x)+" "+str(self.distance_of_y)+" "+"1"


        #And now, we set these values to the world file.

        for element in self.root.findall("./world[@name='multiVessel_world']/model[@name='ocean_waves']/link[@name='ocean_waves_link']/visual[@name='ocean_waves_visual']/geometry/mesh/scale"):
            element.text = mapMeasures

        for element in self.root.findall("./world[@name='multiVessel_world']/model[@name='ocean_waves']/link[@name='ocean_waves_link']/visual[@name='ocean_waves_below_visual']/geometry/mesh/scale"):
            element.text = mapMeasures

        self.tree.write(self.xmlFile)

    def User_Defined_Vessel_Details_setParam(self):
        #This function reads self.global_wp_data to set up the vessel details from real AIS data.
        self.AIS_data = self.global_wp_data_userDefinedVessels["AIS_Data"]

        for i in range(len(self.AIS_data)):
            #In each iteration, set the vessel information by reading from AIS data JSON
            print("self.AIS_data[i][0]", self.AIS_data[i][0])
            vessel_AIS_Data_zero = self.AIS_data[i][0]
            vessel_details_dict = {
                "vessel_id":self.vessel_id,
                "MMSI": vessel_AIS_Data_zero["MMSI"],
                "VesselName": vessel_AIS_Data_zero["VesselName"],
                "IMO": vessel_AIS_Data_zero["IMO"],
                "CallSign": vessel_AIS_Data_zero["CallSign"],
                "VesselType": vessel_AIS_Data_zero["VesselType"],
                "Length": vessel_AIS_Data_zero["Length"],
                "Width": vessel_AIS_Data_zero["Width"],
                "Width": vessel_AIS_Data_zero["Width"],
                "Draft": vessel_AIS_Data_zero["Draft"],
                "Cargo": vessel_AIS_Data_zero["Cargo"],
                "TransceiverClass": vessel_AIS_Data_zero["TransceiverClass"],
            }
            rospy.set_param('vessel'+str(self.vessel_id)+"_details", vessel_details_dict)
            self.vessel_id +=1

    
    def Vessel_Details_setParam(self):
        #This function reads self.global_wp_data to set up the vessel details from real AIS data.
        self.AIS_data = self.global_wp_data["AIS_Data"]

        self.vessel_id = 1
        for i in range(len(self.AIS_data)):
            #In each iteration, set the vessel information by reading from AIS data JSON
            print("self.AIS_data[i][0]", self.AIS_data[i][0])
            vessel_AIS_Data_zero = self.AIS_data[i][0]
            vessel_details_dict = {
                "vessel_id":self.vessel_id,
                "MMSI": vessel_AIS_Data_zero["MMSI"],
                "VesselName": vessel_AIS_Data_zero["VesselName"],
                "IMO": vessel_AIS_Data_zero["IMO"],
                "CallSign": vessel_AIS_Data_zero["CallSign"],
                "VesselType": vessel_AIS_Data_zero["VesselType"],
                "Length": vessel_AIS_Data_zero["Length"],
                "Width": vessel_AIS_Data_zero["Width"],
                "Width": vessel_AIS_Data_zero["Width"],
                "Draft": vessel_AIS_Data_zero["Draft"],
                "Cargo": vessel_AIS_Data_zero["Cargo"],
                "TransceiverClass": vessel_AIS_Data_zero["TransceiverClass"],
            }
            rospy.set_param('vessel'+str(self.vessel_id)+"_details", vessel_details_dict)
            self.vessel_id +=1



    def Vessel_Information_setParam(self):
        #This function reads from self.config_data about the user specified vessel information.
        #From self.config_data, this function gets all that information related to vessel information to a list of dictionaries one by one for each vessel.
        #Since user can specify the same information for more than one vessel, this function should consider that as well.
        #After getting all the vessel information in a list of dictionaries, this function sets a ROS parameter for all of them.
        for i in range(len(self.vessel_details_json)):
            print(self.vessel_details_json[i])
            vessel_id_json = self.vessel_details_json[i]["vessel_id"] 
            
            if vessel_id_json.find("-") != -1 : 
                #If the character - is used in the vessel_id name, it indicates that these details should apply for more than one vessel.
                #The related vessels are specified in the vessel_id such as 3-8
                vessel_IDs_in_str = vessel_id_json.split("-")
                vessel_IDs_in_int = [int(i) for i in vessel_IDs_in_str]

                naming_index = 1


                for j in range(vessel_IDs_in_int[0],vessel_IDs_in_int[1] + 1):
                    #Let's say that "vessel_id":"3-8" in one element of the JSON data just like the example data below:
                    """
                    "vessel_id":"3-8",
                    "vessel_name":"BALTACI",
                    "flag":"CYPRUS",
                    "length":100,
                    "width":20,
                    "gross_tonnage":100,
                    "draught":50
                    """
                    #What we want to do is:
                        # Put the names of the vessels as BALTACI 1, BALTACI 2, BALTACI 3, BALTACI 4,BALTACI 5,BALTACI 6,BALTACI 7,BALTACI 8. 
                        # Simply add a number at the end of the name and keep adding +1 to that number.
                                # Similar naming is used for real life vessels.
                                # Users can use this feature to name a bunch of ships after the ships' specialties. 
                                # For example, if user wants to compare different local path planning algorithms such as RRT and APF
                                # vessels can be named as RRT 1, RRT 2, RRT 3, APF 1, APF 2 etc.
                        # After specifying the name, other properties can be specified as it's indicated on JSON file.
                    vessel_id   = str(j) 
                    name_index_str = str(naming_index)
                    vessel_name = self.vessel_details_json[i]["vessel_name"]
                    
                    vessel_dict = self.vessel_details_json[i]
                    vessel_dict["vessel_name"] = vessel_name + " " + name_index_str
                    vessel_dict["vessel_id"]   = int(vessel_id)

                    naming_index = int( int(naming_index) + 1 )

                    rospy.set_param('vessel'+vessel_id+"_details", vessel_dict)




            
            elif vessel_id_json.find("-") == -1 : 
                #If the character - is not used in the vessel_id name, it indicates that these details should apply for only one vessel.
                vessel_dict = self.vessel_details_json[i]
                vessel_id = str( self.vessel_details_json[i]["vessel_id"] )
                vessel_dict["vessel_id"]   = int(vessel_id)
                rospy.set_param('vessel'+vessel_id+"_details", vessel_dict)

                    





        pass

    
    def Local_Path_Planner_setParam(self):
        #This function reads from self.config_data about the user specified vessel information related to local path planner algorithm.
        #And sets a ROS parameter about the local path planner algorithm.
        self.local_path_planning_dict_json = self.config_data["local_path_planning"][0]["RRT"][0]
        rospy.set_param("local_path_planning_parameters", self.local_path_planning_dict_json)

    def UDV_Local_Path_Planner_setParam(self):
        #This function reads from self.config_data about the user specified vessel information related to local path planner algorithm.
        #And sets a ROS parameter about the local path planner algorithm.
        self.local_path_planning_dict_json = self.config_data_udv["local_path_planning"][0]["RRT"][0]
        rospy.set_param("udv_local_path_planning_parameters", self.local_path_planning_dict_json)
    
    def Global_Path_Planner_setParam(self):
        #This function reads from self.config_data about the user specified vessel information related to global path planner algorithm.
        #And sets a ROS parameter about the global path planner algorithm.
        pass

    def Vessel_Controller_setParam(self):
        #This function reads from self.config_data about the user specified vessel information related to vessel controller algorithm.
        #And sets a ROS parameter about the vessel controller algorithm.
        self.controller_dict_json = self.config_data["controller"][0]["PID_Controller"][0]
        rospy.set_param("vessel_controller_parameters", self.controller_dict_json)
        

    def UsvSpawner(self):


        os.system("python3 "+self.full_path+"/scripts/vessel_spawner.py")
            #subprocess.run(["python3",self.full_path+"usvSpawner.py"])

        time.sleep(2)
        #subprocess.run(["python3",self.full_path+"/PIDBasicMultiRobotControllerUsv.py"])
        #os.system("python3 "+self.full_path+"/PIDBasicMultiRobotControllerUsv.py")
        #subprocess.call("python3 "+self.full_path+"/PIDBasicMultiRobotControllerUsv.py", shell=True)
        #subprocess.call(['gnome-terminal', '-x', "python PIDBasicMultiRobotControllerUsv.py"])
        #subprocess.call(["lxterminal -e"+"python3 "+self.full_path+"/PIDBasicMultiRobotControllerUsv.py"], cwd=self.full_path, shell=True),
        #subprocess.run(["python", "PIDBasicMultiRobotControllerUsv.py"])
        
        #All these stuff above didn't work for me.
        #I kept them in order to not try these commands again. 

        """
        os.system("gnome-terminal -- bash -c \"source ~/vrx_ws/devel/setup.bash;python perception_pose_aggregator_node.py; bash\" ")
        
        os.system("gnome-terminal -- bash -c \"source ~/vrx_ws/devel/setup.bash;python multiple_vessel_motion_planner_node.py; bash\" ")
        """

        #os.system("gnome-terminal -- bash -c \"python PerceptionNode.py; bash\" ")

        #TODO: In the JSON file, make the options to allow starting the perception node or other features upon starting the sim






if __name__ == '__main__':
    my_sim_object = Simulation()


