#!/usr/bin/env python
"""
Switch mechanism ROS node for switching in between local path planner and global path planner.

More details, flowcharts and block diagrams about this node and the whole package in generai is available in the gitHub repository.

This ROS node generates one thread per vessel in the simulation environment. 
And in one thread these happen:
    Thread subscribes to /VesselX/VesselsInShipDomain ROS topic and /VesselX/Is_On_Global_Path to check if any vessels enter to the ship domain of own vessel.
    If a vessel enters to ship domain AND  /VesselX/Is_On_Global_Path == 1  then it means the thread needs to start generating a local path. 
        At that point, Switch_Mechanism ROS node will also be notified since it's subscribed to /VesselX/Is_On_Global_Path and /VesselX/VesselsInShipDomain. And it will set the /VesselX/Is_On_Global_Path to 2
    
    When the local path is generated, it will be published to /VesselX/Local_Trajectory and Switch_Mechanism ROS node will set the /VesselX/Is_On_Global_Path topic to 0 when it takes the  /VesselX/Local_Trajectory.

    After local path is generated and is being tracked by Trajectory_Tracker ROS node, this node will keep track of the local path and check if it gets interrupted.
    This will be done this way: 
        While While local path is not completed: #(while /VesselX/Is_On_Global_Path != 1) (last waypoint of local path is not reached) (checked by switch mechanism)
            Load current vessels inside ship domain and check if any vessel interferes with local path
            If an interference to local path occurs:
                Re-Generate local path 
"""

from os import kill
import rospy
import math
from rospy.core import rospydebug
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, Int16
import threading
import json
from geopy import distance
import os
from tf.transformations import euler_from_quaternion
from multivessel_msgs.msg import VesselPose,Perception

class Switch_Mechanism(threading.Thread):
    def __init__(self, VesselID_):
        threading.Thread.__init__(self)
        self.path = "~/vrx_ws/src/vrx/MultiVessel_Simulation/multiple_vessels"
        self.full_path = os.path.expanduser(self.path)
        
        with open(self.full_path+'/json_files/config.json','r') as f:    
            self.config_data = json.load(f)

        self.vesselID = VesselID_
        self.RRT_parameters = rospy.get_param("/local_path_planning_parameters")
        self.map_area_coefficient = self.RRT_parameters["map_area_coefficient"]
        self.ship_domain_radius = rospy.get_param("/vessel"+str(self.vesselID)+"_Ship_Domain_Radius")
        self.local_map_area = self.map_area_coefficient * self.ship_domain_radius

        self.rrt_chosen = self.config_data["local_path_planning"][0]["RRT"][0]["chosen"]
        self.reaching_to_local_goal_treshold = self.config_data["local_path_planning"][0]["RRT"][0]["reaching_to_local_goal_treshold"]
        #Load our own vessel's details. Because it's necessary to calculate the ship domain's radius.
        vessel_details_param_txt = "/vessel" + str(self.vesselID)  + "_details"
        self.vessel_details = rospy.get_param(vessel_details_param_txt)
        
        self.global_trajectory_txt = '/vessel'+str(self.vesselID)+"_Global_Trajectory"
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

        self.vessels_in_ship_domain = Perception() 

        #subscribe to GPS and IMU sensors
        sub_topic_gps_txt = "vessel"+str(self.vesselID) +"/vessel"+str(self.vesselID)+"/sensors"+"/gps/gps/fix"
        rospy.Subscriber(sub_topic_gps_txt,NavSatFix, self.CallbackGps)

        sub_topic_imu_txt = "vessel"+str(self.vesselID) +"/vessel"+str(self.vesselID)+"/sensors"+"/imu/imu/data"
        rospy.Subscriber(sub_topic_imu_txt,Imu, self.CallbackImu)

        #Subscribe to ShipDomain topic
        sim_Perception_topic_txt =  "vessel"+str(self.vesselID) + "/vessels_in_ship_domain"
        rospy.Subscriber(sim_Perception_topic_txt,Perception, self.CallbackShipDomain)

        #Subscribe to local path topic.
        sim_local_path_topic_txt =  "vessel"+str(self.vesselID) + "/local_path"
        rospy.Subscriber(sim_local_path_topic_txt,Float64MultiArray, self.CallbackLocalPath)

        #Init the calculating_local_path topic
        pub_generate_local_path_txt = "vessel"+str(self.vesselID) + "/local_path_args"
        self.pub_generate_local_path= rospy.Publisher(pub_generate_local_path_txt,Float64MultiArray,queue_size=10)

        #Init the is_on_global_path topic
        pub_is_on_global_path_txt = "vessel"+str(self.vesselID) + "/is_on_global_path"
        self.pub_is_on_global_path= rospy.Publisher(pub_is_on_global_path_txt,Int16,queue_size=10)
        #0:local
        #1:global
        #2:generating local path now

        self.there_is_a_new_encounter = False
        self.Vessels_In_Ship_Domain_Interfere_With_Local_Path = False

        self.cnt = 0
        print(str(self.vesselID)," vessel switch mechanism started")

        self.vessel_is_on_global_path = 1

        self.localPath = []
        self.current_local_path = []
        self.local_path_is_generated = False
        self.vessel_poses_xy = []

        local_path_planning_parameters = rospy.get_param("/local_path_planning_parameters")
        self.vessel_forbidden_zone_coefficient_length = local_path_planning_parameters["vessel_forbidden_zone_coefficient_length"]
        self.vessel_forbidden_zone_coefficient_width = local_path_planning_parameters["vessel_forbidden_zone_coefficient_width"]
        self.vessel_forbidden_zone_coefficient_draft = local_path_planning_parameters["vessel_forbidden_zone_coefficient_draft"]
        self.vessel_forbidden_zone_radius_equation = local_path_planning_parameters["vessel_forbidden_zone_radius_equation"]

    def run(self):
        while not rospy.is_shutdown():
            #try:
                rate.sleep()
                self.Check_New_Encounter()
                if self.there_is_a_new_encounter == True:
                    print("New encounter occured for vessel"+str(self.vesselID)+". Let's start to generate local path.")
                    self.Publish_to_Generate_Local_Path_Topic()
                    msg = Int16(2)
                    self.pub_is_on_global_path.publish(msg)
                    while not rospy.is_shutdown():
                        try:
                            rate.sleep()
                            self.Publish_to_Generate_Local_Path_Topic()
                            self.Check_Local_Path_Topic()
                            if self.local_path_is_generated == True:
                                break
                        except Exception as e:
                            rospy.logerr(e)

                    print("got the local path for vessel"+str(self.vesselID))
                    self.pub_is_on_global_path.publish(0)
                    
                    self.local_path_is_generated = False#let's set the flag right after we got out of the loop.
                    
                    rospy.sleep(2)
                    self.vessel_is_on_global_path = 0

                    while self.vessel_is_on_global_path != 1:
                        #While local path is not completed
                        self.pub_is_on_global_path.publish(0)#publish continuously that we are on local path
                            
                        self.Check_if_The_Local_Path_Is_Completed()#Checks if the local path is completed

                        #Load current vessels inside the ship domain and check if any of them interferes with local path
                        self.Check_if_Vessels_In_Ship_Domain_Interfere_With_Local_Path() # this will change the variable self.Vessels_In_Ship_Domain_Interfere_With_Local_Path

                        if self.Vessels_In_Ship_Domain_Interfere_With_Local_Path == True:
                            #If any vessel interferes with the current local path, break this loop to restart the local path planning process

                            self.Vessels_In_Ship_Domain_Interfere_With_Local_Path = False #Switch the flag right back off for the next iteration

                            break
                        rate.sleep()

                elif self.there_is_a_new_encounter == False:
                    msg = Int16(1)
                    self.pub_is_on_global_path.publish(msg)

                    msg_to_be_published = Float64MultiArray()
                    self.pub_generate_local_path.publish(msg_to_be_published)

            #except Exception as e:
                #rospy.logerr(e)
                
                rate.sleep()

    def Check_Local_Path_Topic(self):
        #This function checks if local path topic is published
        check_local_path = self.localPath 
        if len(check_local_path)>0:
            self.local_path_is_generated =True
    
    def Check_if_The_Local_Path_Is_Completed(self):
        len_localPath = len(self.localPath)
        print("self.localPat",self.localPath)
        self.localGoalLat = self.localPath[-2]
        self.localGoalLon = self.localPath[-1]

        self.localGoal = (self.localGoalLat,self.localGoalLon)
        self.current_position = (self.current_lat,self.current_lon)

        print(self.localGoal,"self.localGoal")
        print(self.current_position,"self.current_position")

        distance_in_between = distance.distance(self.localGoal, self.current_position).meters

        print(distance_in_between,"distance_in_between", "for vessel",self.vesselID,"self.localGoal",self.localGoal)
        
        if distance_in_between  < self.reaching_to_local_goal_treshold:
            self.vessel_is_on_global_path = 1
            
            Last_Reached_Global_WpRosParam = rospy.get_param("/vessel"+str(self.vesselID)+"_Last_Reached_Global_Wp")
            Last_Reached_Global_WpRosParam +=1#jump to the next waypoint.   
            rospy.set_param('vessel'+str(self.vesselID)+"_Last_Reached_Global_Wp", Last_Reached_Global_WpRosParam)

    def Check_if_Vessels_In_Ship_Domain_Interfere_With_Local_Path(self):
        #This function loads the current positions of the vessels inside ship domain and calculates the forbidden zones they allocate.
        #After finding every forbidden zone, this function checks whether or not our current local path gets interrupted by any of those forbidden zones.
        print("Let's check any vessel interrupts the local path for vessel"+str(self.vesselID))

        self.vessel_poses = self.vessels_in_ship_domain.vessel_poses
        self.current_local_path = self.localPath

        self.origin_lat = self.current_lat
        self.origin_lon = self.current_lon

        #First, check if any local path points are on top of an obstacle with isFree function.
        #After, check if local path line's any point are on top of an obstacle with crossObstacle function.

        self.Convert_Vessel_Positions_to_XY()
        self.Generate_COLREGS_Virtual_Obstacles()
        
        if self.virtual_obstacles_xy != []:
            self.vessel_poses_xy=self.vessel_poses_xy + self.virtual_obstacles_xy

        self.Remove_Obstacles_That_Collide_With_Starting_Point()
        local_path_is_free_bool = self.Local_Path_isFree()

        if local_path_is_free_bool == False:
            self.Vessels_In_Ship_Domain_Interfere_With_Local_Path = True
            return True
        else:
            local_path_crosses_an_obstacle = self.Local_Path_crossObstacle()
            if local_path_crosses_an_obstacle == True:
                self.Vessels_In_Ship_Domain_Interfere_With_Local_Path = True
                return True
            elif local_path_crosses_an_obstacle == False:#If the code can come up here, no vessels in ship domain interferes with our local path.
                self.Vessels_In_Ship_Domain_Interfere_With_Local_Path = False
                return False
    
        pass

    def Local_Path_isFree(self):
        #This function checks whether or not all the lat-lon points on the local path are not on top of any obstacle.
        #If any lat-lon couple in the local path is on top of an obstacle, return False
        #If all is safe, return True

        current_local_path_tpl=self.current_local_path
        current_local_path = list(current_local_path_tpl)

        while (len(current_local_path)>0):
            lat = current_local_path.pop(0)
            lon = current_local_path.pop(0)
            x,y = self.get_local_coord(lat,lon)

            vessel_poses = self.vessel_poses_xy.copy()
            while(len(vessel_poses)>0 ):
                vessel_pose_=vessel_poses.pop(0)

                px = (float(vessel_pose_.position.x) - float(x))**2
                py = (float(vessel_pose_.position.y) - float(y))**2

                distance = (px+py)**(0.5)

                forbidden_zone_radius = self.calc_forbidden_zone_radius(vessel_pose_.vessel_details.Length, vessel_pose_.vessel_details.Width)

                if distance < forbidden_zone_radius:
                    print("One point on the local path is on top of a forbidden zone!")
                    
                    return False
            
            return True
        pass

    def Local_Path_crossObstacle(self):
        #This function checks whether or not all the lat-lon points on the local path are not crossing any obstacles.
        #If local path crosses any obstacles, return True
        #If not, return False

        current_local_path_tpl=self.current_local_path
        current_local_path = list(current_local_path_tpl)

        while (len(current_local_path)>0):
            lat = current_local_path.pop(0)
            lon = current_local_path.pop(0)
            x1,y1 = self.get_local_coord(lat,lon)

            lat = current_local_path.pop(0)
            lon = current_local_path.pop(0)
            x2,y2 = self.get_local_coord(lat,lon)

            vessel_poses = self.vessel_poses_xy.copy()
            while(len(vessel_poses)>0 ):
                vessel_pose_=vessel_poses.pop(0)
                forbidden_zone_radius = self.calc_forbidden_zone_radius(vessel_pose_.vessel_details.Length, vessel_pose_.vessel_details.Width)
                for i in range(0,201):
                    u = i/200
                    x = x1*u + x2*(1-u)
                    y = y1*u + y2*(1-u)

                    px = (float(vessel_pose_.position.x) - float(x))**2
                    py = (float(vessel_pose_.position.y) - float(y))**2
                    distance = (px+py)**(0.5)


                    if distance < forbidden_zone_radius:
                        return True
            return False

        pass

    def Generate_COLREGS_Virtual_Obstacles(self):
        #This function reads the map size from 'vessel'+str(self.vesselIDl)+"_Local_Map_Dimensions ROS parameter 
        #And with using the self.vessel_poses_xy list, this function generates the COLREGS virtual obstacles and appends to the self.vessel_poses_xy list.
        #Therefore, local path will be checked if it got interrupted by both from vessel poses and virtual obstacles.
        vessel_local_map_param_txt = "/vessel" + str(self.vesselID)  + "_Local_Map_Dimensions"
        self.local_map_details = []
        self.local_map_details = rospy.get_param(vessel_local_map_param_txt)

        self.map_X_min = self.local_map_details[0]
        self.map_X_max = self.local_map_details[1]
        self.map_Y_min = self.local_map_details[2]
        self.map_Y_max = self.local_map_details[3]

        self.virtual_obstacles_xy = []

        iteration_count = 0
        vessel_poses = self.vessel_poses_xy.copy()
        while(len(vessel_poses)>0 ):
            vessel_pose_=vessel_poses.pop(0)

            roll,pitch,self.own_vessel_yaw = euler_from_quaternion([self.quaternion_x, self.quaternion_y, self.quaternion_z, self.quaternion_w])

            relative_angle = math.atan((vessel_pose_.position.y)/(vessel_pose_.position.x))
            colregs_angle = self.own_vessel_yaw-relative_angle 

            roll,pitch, ts_angle_yaw =  euler_from_quaternion([vessel_pose_.orientation.x, vessel_pose_.orientation.y, vessel_pose_.orientation.z, vessel_pose_.orientation.w])

            if colregs_angle < 0:
                colregs_angle += 2*math.pi
            elif colregs_angle > 2*math.pi:
                colregs_angle -= 2*math.pi

            colregs_angle_degrees = math.degrees(colregs_angle)
            print("colregs_angle_degrees",colregs_angle_degrees)

            if (colregs_angle_degrees > 5) and (colregs_angle_degrees <= 112.5):
                print("OS is rule 15-give_way")
                forbidden_zone_radius = self.calc_forbidden_zone_radius(vessel_pose_.vessel_details.Length, vessel_pose_.vessel_details.Width)
                self.virtual_obs_distance_x = forbidden_zone_radius
                self.virtual_obs_distance_y = forbidden_zone_radius
                self.virtual_obs_x_minus_one = vessel_pose_.position.x
                self.virtual_obs_y_minus_one = vessel_pose_.position.y

                while (abs(self.virtual_obs_x_minus_one)<self.map_X_max ) and (abs(self.virtual_obs_y_minus_one)<self.map_Y_max ):
                    print("ts_angle_yaw",ts_angle_yaw)  
                    virtual_obs_x = self.virtual_obs_x_minus_one + math.sin(ts_angle_yaw)*self.virtual_obs_distance_x
                    virtual_obs_y = self.virtual_obs_y_minus_one + math.cos(ts_angle_yaw)*self.virtual_obs_distance_y

                    virtual_obs_xy = VesselPose()
                    virtual_obs_xy.position.x = virtual_obs_x
                    virtual_obs_xy.position.y = virtual_obs_y

                    virtual_obs_xy.vessel_details.Length = vessel_pose_.vessel_details.Length
                    virtual_obs_xy.vessel_details.Width = vessel_pose_.vessel_details.Width
                    self.virtual_obstacles_xy.append(virtual_obs_xy)

                    self.virtual_obs_x_minus_one = virtual_obs_x  
                    self.virtual_obs_y_minus_one = virtual_obs_y

            elif (colregs_angle_degrees > 112.5) and (colregs_angle_degrees <= 247.5):
                print("OS is rule 13-give_way")
            elif (colregs_angle_degrees > 247.5) and (colregs_angle_degrees <= 355):
                print("OS is rule 15-stand_on")
            elif ( (colregs_angle_degrees > 355) and (colregs_angle_degrees <= 360) ) or ((colregs_angle_degrees > 0) and (colregs_angle_degrees <= 5)):
                print("OS is rule 13-stand_on or rule 14")
                forbidden_zone_radius = self.calc_forbidden_zone_radius(vessel_pose_.vessel_details.Length, vessel_pose_.vessel_details.Width)
                self.virtual_obs_distance_x = forbidden_zone_radius
                self.virtual_obs_distance_y = forbidden_zone_radius
                self.virtual_obs_x_minus_one = vessel_pose_.position.x
                self.virtual_obs_y_minus_one = vessel_pose_.position.y
                
                while (abs(self.virtual_obs_x_minus_one)<self.map_X_max ) and (abs(self.virtual_obs_y_minus_one)<self.map_Y_max ):
                    virtual_obs_x = self.virtual_obs_x_minus_one - math.sin(ts_angle_yaw)*self.virtual_obs_distance_x
                    virtual_obs_y = self.virtual_obs_y_minus_one + math.cos(ts_angle_yaw)*self.virtual_obs_distance_y

                    virtual_obs_xy = VesselPose()
                    virtual_obs_xy.position.x = virtual_obs_x
                    virtual_obs_xy.position.y = virtual_obs_y

                    virtual_obs_xy.vessel_details.Length = vessel_pose_.vessel_details.Length
                    virtual_obs_xy.vessel_details.Width = vessel_pose_.vessel_details.Width
                    self.virtual_obstacles_xy.append(virtual_obs_xy)

                    self.virtual_obs_x_minus_one = virtual_obs_x  
                    self.virtual_obs_y_minus_one = virtual_obs_y

            iteration_count +=1


    def Remove_Obstacles_That_Collide_With_Starting_Point(self):
        #This function checks all the vessel positions and removes if any vessel forbidden zone collides with the starting point.
        #This function should check not only vessel_poses, but virtual_obstacles as well.

        (x,y) = (0, 0)

        iteration_count = 0
        vessel_poses = self.vessel_poses_xy.copy()
        while(len(vessel_poses)>0 ):
            vessel_pose_=vessel_poses.pop(0)

            px = (float(vessel_pose_.position.x) - float(x))**2
            py = (float(vessel_pose_.position.y) - float(y))**2
            distance = (px+py)**(0.5)

            forbidden_zone_radius = self.calc_forbidden_zone_radius(vessel_pose_.vessel_details.Length, vessel_pose_.vessel_details.Width)

            if distance < forbidden_zone_radius*1.5:
                self.vessel_poses_xy.pop(iteration_count)
                print("can't place an obstacle on top of the starting point!")
                pass
            else:
                iteration_count +=1

    def Convert_Vessel_Positions_to_XY(self):
        #This function converts every position variable to XY 
        #Takes the Lat-Lon values from the self.vessel_poses list
        #and puts the XY conversions in the self.vessel_poses_xy list

        self.vessel_poses_xy = []
        #Make sure to reset this list in order to not add the same vessel positions over and over again.s
        
        for i in range(len(self.vessel_poses)):
            pos_x,pos_y = self.get_local_coord(self.vessel_poses[i].position.x,self.vessel_poses[i].position.y)
            #Convert the Lat-Lon values to XY type
            
            vessel_pose_xy = VesselPose()
            vessel_pose_xy.vessel_id = self.vessel_poses[i].vessel_id
            
            vessel_pose_xy.vessel_details.Length = self.vessel_poses[i].vessel_details.Length
            vessel_pose_xy.vessel_details.Width = self.vessel_poses[i].vessel_details.Width

            vessel_pose_xy.position.x = pos_x
            vessel_pose_xy.position.y = pos_y

            #Make sure to put the orientation values as well.
            vessel_pose_xy.orientation.x = self.vessel_poses[i].orientation.x
            vessel_pose_xy.orientation.y = self.vessel_poses[i].orientation.y
            vessel_pose_xy.orientation.z = self.vessel_poses[i].orientation.z
            vessel_pose_xy.orientation.w = self.vessel_poses[i].orientation.w
            self.vessel_poses_xy.append(vessel_pose_xy)

    def calc_forbidden_zone_radius(self,length_,width_):
        #This function takes a ship's lenghth and width measurements.
        #And calculates the forbidden_zone radius for the ship
        length = length_
        width = width_
        
        self.vessel_forbidden_zone_coefficient_length

        if self.vessel_forbidden_zone_radius_equation == "W*Wc+L*Lc":
            forbidden_zone_radius = (length*self.vessel_forbidden_zone_coefficient_length) + (width*self.vessel_forbidden_zone_coefficient_width)

        #This radius can be calculated by different equations.
        #This function returns the forbidden_zone_radius. 

        return forbidden_zone_radius

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
    
    def Check_New_Encounter(self):
        """
        If
 /VesselX/Is_On_Global_Path == 1
AND
/VesselX/Vessels_In_Ship_Domain is changed"""

        vesselsInShipDomain = self.vessels_in_ship_domain

        if (len(vesselsInShipDomain.vessel_poses) > 0) :
            #if there is a vessel inside ship domain and vessel is on global path, start the local path planning procedure by 
            self.there_is_a_new_encounter = True
        else:
            self.there_is_a_new_encounter = False

        return self.there_is_a_new_encounter

    def Publish_to_Generate_Local_Path_Topic(self):
        start =(self.current_lat ,self.current_lon)
        goalRosParam = []
        goalRosParam = rospy.get_param("/vessel"+str(self.vesselID)+"_Global_Trajectory") 
        Last_Reached_Global_WpRosParam = rospy.get_param("/vessel"+str(self.vesselID)+"_Last_Reached_Global_Wp")
        if Last_Reached_Global_WpRosParam == -1:
            Last_Reached_Global_WpRosParam = 0

        total_waypoints = len(goalRosParam) // 5

        print("goalRosParam",len(goalRosParam))
        print("Last_Reached_Global_WpRosParam",Last_Reached_Global_WpRosParam)
        print("(5*(Last_Reached_Global_WpRosParam+1))+0",(5*(Last_Reached_Global_WpRosParam+1))+0)
        #In here, this function should skip to the next global waypoint if the current global waypoint is inside the local path bounding box. 
        while ( len(goalRosParam) / 5 )> Last_Reached_Global_WpRosParam :#We check for this here as well. Because we don't want to exceed the index point. 
            goalLat = goalRosParam[(5*(Last_Reached_Global_WpRosParam))+0]
            goalLon = goalRosParam[(5*(Last_Reached_Global_WpRosParam))+1]
            goal = (goalLat,goalLon)
            current_pos = (self.current_lat,self.current_lon)
            distance_in_between = distance.distance(goal, current_pos).meters
            print(distance_in_between,"distance_in_between")
            #How do we calculate the local path bounding box?
            if self.local_map_area > distance_in_between:
                Last_Reached_Global_WpRosParam +=1#jump to the next waypoint.
                print("global waypoint is inside the local map. Assign the next global waypoint as the goal point.")
                rospy.set_param('vessel'+str(self.vesselID)+"_Last_Reached_Global_Wp", Last_Reached_Global_WpRosParam)
            else:
                
                #If the global waypoint is outside the local map, break this while loop.
                #This while loop doesn't go out of bounds because the while condition is depending on the length of the goalRosParam which has all the global waypoints inside.
                #The exceptional condition we fixed here is this: What if the last global waypoint happens to be inside the local path bounding box?
                #With this loop structure, this loop will set the goal point as it is and won't push it further.
                break
            
        if ( len(goalRosParam) / 5 )> Last_Reached_Global_WpRosParam:
            goalLat = goalRosParam[(5*(Last_Reached_Global_WpRosParam+1))+0]
            goalLon = goalRosParam[(5*(Last_Reached_Global_WpRosParam+1))+1]
            goal = (goalLat,goalLon)

            vessel_poses = []
            vessel_poses = self.vessels_in_ship_domain.vessel_poses

            self.vessel_is_on_global_path = 0
            msg_to_be_published = Float64MultiArray()
            msg_to_be_published.data = [self.current_lat,self.current_lon,goalLat,goalLon]
            self.pub_generate_local_path.publish(msg_to_be_published)
        else:
            print(len(goalRosParam), "len(goalRosParam)")
            print("Last_Reached_Global_WpRosParam",Last_Reached_Global_WpRosParam)
            
            hz = 0.1
            rate = rospy.Rate(hz)
            while True:
                        print("vessel"+str(self.vesselID)+ "has already reached to it's final goal point and is stationary. Not planning any local goals for it.")
                        
                        rate.sleep()

    def CallbackLocalPath(self,local_path_msg_):
        self.localPath  = local_path_msg_.data

    def CallbackShipDomain(self,perception_msg_):
        self.vessels_in_ship_domain = perception_msg_

    def CallbackGps(self,GPS_msg):

        self.current_lat = GPS_msg.latitude
        self.current_lon = GPS_msg.longitude
    
    def CallbackImu(self,imu_msg):    
        self.quaternion_x  = imu_msg.orientation.x
        self.quaternion_y = imu_msg.orientation.y
        self.quaternion_z = imu_msg.orientation.z
        self.quaternion_w = imu_msg.orientation.w

if __name__ == '__main__':
    rospy.init_node('multi_vessel_local_path_planner', anonymous=True)
    hz = 10
    rate = rospy.Rate(hz)

    vessel_objects = []
    vessel_count = int(rospy.get_param("/vessel_count"))#get the number of how many Vessels do we spawn in this run
    print("Number of Vessels: ", str(vessel_count))

    i=1
    for i in range(vessel_count):
        vessel_objects.append(Switch_Mechanism(i+1))

    for obj in vessel_objects:
        obj.start()