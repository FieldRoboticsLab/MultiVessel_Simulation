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
import time

class Vessel_Node_Starter():
    def __init__(self,vessel_count_):
        self.path = "~/vrx_ws/src/vrx/multiple_vessels"
        self.full_path = os.path.expanduser(self.path)
        #One object of this class will be called for each vessel.

        with open(self.full_path+'/json_files/config.json','r') as f:    
            self.config_data = json.load(f)
        self.vessel_count = vessel_count_
        self.Start_Nodes()

    def Start_Nodes(self):
        #This function sets the ROS parameters for every vessel:
        #https://linuxcommandlibrary.com/man/gnome-terminal
        #https://www.baeldung.com/linux/gnome-terminal-command-line


        #If the global path planner is chosen, start the ROS node in a new tab.
        self.global_path_planner_chosen = self.config_data["simulation"][0]["available_nodes"][0]["Global_Path_Planner"]

        if self.global_path_planner_chosen == True:
            os.system("gnome-terminal --tab -- bash -c \"source ~/vrx_ws/devel/setup.bash;python multiple_vessel_global_path_planner_node.py; bash\" ")

        time.sleep(0.5)
        #If the perception is chosen, start the ROS node in a new tab.
        self.perception_chosen = self.config_data["simulation"][0]["available_nodes"][0]["Perception"]

        if self.perception_chosen == True:
            os.system("gnome-terminal --tab -- bash -c \"source ~/vrx_ws/devel/setup.bash;python perception_pose_aggregator_node.py; bash\" ")

        time.sleep(0.5)
        #If the ship domain is chosen, start the ROS node in a new tab.
        self.ship_domain_chosen = self.config_data["simulation"][0]["available_nodes"][0]["Ship_Domain"]

        if self.ship_domain_chosen == True:
            os.system("gnome-terminal --tab -- bash -c \"source ~/vrx_ws/devel/setup.bash;python multiple_vessel_ship_domain_node.py; bash\" ")

        time.sleep(0.5)
        #If the switch mechanism is chosen, start the ROS node in a new tab.
        self.switch_mechanism_chosen = self.config_data["simulation"][0]["available_nodes"][0]["Switch_Mechanism"]

        if self.switch_mechanism_chosen == True:
            os.system("gnome-terminal --tab -- bash -c \"source ~/vrx_ws/devel/setup.bash;python multiple_vessel_switch_mechanism.py; bash\" ")

        time.sleep(0.5)
        #If the local path planner is chosen, start the ROS node in a new tab.
        self.local_path_planner_chosen = self.config_data["simulation"][0]["available_nodes"][0]["Local_Path_Planner"]

        if self.local_path_planner_chosen == True:
            os.system("gnome-terminal --tab -- bash -c \"source ~/vrx_ws/devel/setup.bash;python multiple_vessel_local_path_planner_RRT.py; bash\" ")

        time.sleep(0.5)
        #If the trajectory tracker is chosen, start the ROS node in a new tab.
        self.trajectory_tracker_chosen = self.config_data["simulation"][0]["available_nodes"][0]["Trajectory_Tracker"]

        if self.trajectory_tracker_chosen == True:
            os.system("gnome-terminal --tab -- bash -c \"source ~/vrx_ws/devel/setup.bash;python multiple_vessel_trajectory_tracker.py; bash\" ")

        time.sleep(0.5)
        #If the pure pursuit is chosen, start the ROS node in a new tab.
        self.pure_pursuit_chosen = self.config_data["simulation"][0]["available_nodes"][0]["Pure_Pursuit_Controller"]

        if self.pure_pursuit_chosen == True:
            os.system("gnome-terminal --tab -- bash -c \"source ~/vrx_ws/devel/setup.bash;python multiple_vessel_controller_purePursuit.py; bash\" ")

        time.sleep(0.5)
        #If the logger node is chosen, start the ROS node in a new tab.
        self.logger_chosen = self.config_data["simulation"][0]["available_nodes"][0]["Logger"]

        if self.logger_chosen == True:
            os.system("gnome-terminal --tab -- bash -c \"source ~/vrx_ws/devel/setup.bash;python simulation_logger_node.py; bash\" ")


        

        pass

if __name__ == '__main__':
    rospy.init_node('multi_vessel_global_planner', anonymous=True)
    hz = 10
    rate = rospy.Rate(hz)

    vessel_count = int(rospy.get_param("/vessel_count"))#get the number of how many vessels are spawned in this run
    print("Number of Vessels: ", str(vessel_count))

    Vessel_Node_Starter(vessel_count)