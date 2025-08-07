#!/usr/bin/env python3
#this python script writes down the vessels.launch file according to the rosparam /vessel_count


from concurrent.futures import thread
from os import kill
from tracemalloc import start
#import rospy
import time
import math
import random
#import pyproj as proj
import rospy
from std_msgs.msg import Float64MultiArray
import json
import os

#This is an example of vessels.launch file below

class USV_SPAWN():
  def __init__(self):
    
    self.path = "~/vrx_ws/src/vrx/MultiVessel_Simulation/multiple_vessels"
    self.full_path = os.path.expanduser(self.path)
    
    self.vessel_count = int(rospy.get_param("/vessel_count"))#get the number of how many USV's do we spawn in this run
    self.f = open(self.full_path+"/launch/vessels.launch", "w")#we use W in order to overwrite the existing content in every start. I made it vessels1 in order to not lose it while testing
    self.f.write('<?xml version="1.0"?>'+"\n"+ '<launch>' + "\n")

    
    
    with open(self.full_path+'/json_files/config.json','r') as f:    
      self.config_data_udv = json.load(f)

    with open(self.full_path+'/json_files/config_user_defined.json','r') as f:    
      self.config_data = json.load(f)

    with open(self.full_path+'/json_files/Json_Global_Waypoints.json','r') as f:    
      self.global_wp_data = json.load(f)
    with open(self.full_path+'/json_files/Global_Waypoints_userDefinedVessels.json','r') as f:    
      self.user_defined_vessels_global_wp_data = json.load(f)


    self.origin_lat = self.global_wp_data["OriginLat"]
    self.origin_lon = self.global_wp_data["OriginLon"]
    
    self.gps_enabled = self.config_data["simulation"][0]["available_sensors"][0]["GPS"]
    self.imu_enabled = self.config_data["simulation"][0]["available_sensors"][0]["IMU"]
    self.lidar_enabled = self.config_data["simulation"][0]["available_sensors"][0]["LIDAR"]
    self.camera_enabled = self.config_data["simulation"][0]["available_sensors"][0]["Camera"]#sensor configuration as set from config.json file

    #User defined vessels config
    self.gps_enabled_udv = self.config_data_udv["simulation"][0]["available_sensors"][0]["GPS"]
    self.imu_enabled_udv = self.config_data_udv["simulation"][0]["available_sensors"][0]["IMU"]
    self.lidar_enabled_udv = self.config_data_udv["simulation"][0]["available_sensors"][0]["LIDAR"]
    self.camera_enabled_udv = self.config_data_udv["simulation"][0]["available_sensors"][0]["Camera"]#sensor configuration as set from config.json file

    

    self.usv_locations_x = [] #we store the usv locations in order to not spawn multiple vessels at the same spot.
    self.usv_locations_y = []
    self.map_X_max = 0
    self.map_X_min = -700
    self.map_Y_max = 0
    self.map_Y_min = 300

    self.obstacle_radius = 30#minimum distance in between each USV

  def isFree(self,x_arg,y_arg):#chechks whether if the spot for usv creation is empty
    #checks for obstacle collision
        x = x_arg
        y = y_arg

        self.obs_x = self.usv_locations_x.copy()
        #print(obs_x)
        self.obs_y = self.usv_locations_y.copy()
        #print(self.obstacles_x)

        """ We can check for not spawning an USV on the 
        #first, check if it collides with start or end point.
        #check for that first since it doesn't make sense to compare with all the obstacles while 
        #it collides with start or end in the first place

        pxToStart = (float(self.start[0]) - float(x))**2
            #print(px)
        pyToStart = (float(self.start[1]) - float(y))**2
        distStart = (pxToStart + pyToStart)**(0.5)

        if distStart <= self.obstacle_radius*(1.5) : 
            return False

        pxToStart = (float(self.goal[0]) - float(x))**2
        pyToStart = (float(self.goal[1]) - float(y))**2
        distGoal = (pxToStart + pyToStart)**(0.5)

        if distGoal <= self.obstacle_radius*(1.5) : 
            return False
            """

        while len(self.obs_x) > 0:
            #print("hey")
            #en son eklenen node free space'de mi bunu kontrol ediyoruz
            obs_x_one = self.obs_x.pop(0)
            #print( obs_x_one)
            obs_y_one = self.obs_y.pop(0)
            #print(obs_y_one)
            #obstacle merkezinin xy noktalarini aldik. yaricapini da biliyoruz. O halde kontrolu yapalim

            px = (float(obs_x_one) - float(x))**2
            #print(px)
            py = (float(obs_y_one) - float(y))**2
            #print(py)

            distance = (px+py)**(0.5)
            #print(distance)

            if distance < self.obstacle_radius:
                #eger son eklenen node'un merkezi ile engel merkezi arasindaki mesafe, engel yaricapindan kucukse cakisma vardir.
                #self.remove_node(n)
                print("itCollides")
                
                return False
        
        return True

  def spawn_usv(self):
    self.i = 1
    while self.i<self.vessel_count+1:#I make it 1 and vessel_count+1 so vessels will be usv1,usv2 and so on.
      #generate the xy points first
      
      i = self.i      
      """
      ###OLD CENTERX CENTERY FOR SPAWNING VESSELS AT RANDOM PLACES
      centerX = float(random.uniform(self.map_X_min, self.map_X_max))      
      #generate random x and y for vessel spawn and make sure 
      #it's not out of bounds and not at the same place with another vessel 
      centerY = float(random.uniform(self.map_Y_min, self.map_Y_max)) 
      """
      
      #read the initial vessel location from json

      vessel_lat = self.global_wp_data["AIS_Data"][i-1][0]["LAT"]
      vessel_lon = self.global_wp_data["AIS_Data"][i-1][0]["LON"]

      centerX,centerY = self.get_local_coord(vessel_lat,vessel_lon)



      R = float(random.uniform(0,math.pi*2))
      Y = float(random.uniform(0,math.pi*2))
      
      """
      #if random point for spawn collide with another usv, 
      #don't add to i and dont append
      #if random point doesn't collide, add i and append()
      #for j in range(1,len(self.obstacles_x)):
      if (self.isFree(centerX,centerY)):
      """
      #if it's obstacle free, add a new USV 
      print(centerX,centerY)
      self.usv_locations_x.append(centerX)
      self.usv_locations_y.append(centerY)

      self.f.write("\n" + "\t" + '<!-- BEGIN VESSEL '+str(i)+"-->")
      self.f.write("\n" + "\t" + '<group ns="vessel' +str(i)+'">')#vessels name will be vesseli. Whatever the value of i is in this iteration of the loop
      self.f.write("\n" + "\t"+ "\t"  + '<include file="$(find multiple_vessels)/launch/one_vessel.launch" >')
      #self.f.write("\n" + "\t"+ "\t"  + '<include file="$(find vrx_gazebo)/launch/one_vessel.launch" >')
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="gps_enabled" value="'+str(self.gps_enabled)+'" />')
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="imu_enabled" value="'+str(self.imu_enabled)+'" />')
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="lidar_enabled" value="'+str(self.lidar_enabled)+'" />')
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="camera_enabled" value="'+str(self.camera_enabled)+'" />')
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="x" value="'+str(centerX)+'" />')
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="y" value="'+str(centerY)+'" />')
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="z" value="'+"0.1"+'" />')
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="P" value="'+"0"+'" />')#p = 0 because I don't want to get any USV capsized
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="R" value="'+"0"+'" />') #str(R)
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="Y" value="'+str(Y)+'" />') #
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="namespace"  value="vessel'+str(i)+'" />')

      self.f.write("\n" + "\t"+ "\t"  +"</include>")

      self.f.write("\n" + "\t"+"</group>")
      
      self.i = self.i+1

    #If the number of user defined vessels is greater than 0, start another loop for spawning user defined vessels.
    self.number_of_user_defined_vessels = self.config_data["simulation"][0]["number_of_user_defined_vessels"]
    self.j = 1
    self.total_number_of_vessels = self.number_of_user_defined_vessels + self.vessel_count
    self.vesselID_of_udv =  self.vessel_count+1

    
    while self.j< self.number_of_user_defined_vessels+1:
      #generate the xy points first 
      j = self.j      
      #read the initial vessel location from json
      vessel_lat = self.user_defined_vessels_global_wp_data["AIS_Data"][j-1][0]["LAT"]
      vessel_lon = self.user_defined_vessels_global_wp_data["AIS_Data"][j-1][0]["LON"]
      centerX,centerY = self.get_local_coord(vessel_lat,vessel_lon)
      R = float(random.uniform(0,math.pi*2))
      Y = float(random.uniform(0,math.pi*2)) 
      print(centerX,centerY)
      self.usv_locations_x.append(centerX)
      self.usv_locations_y.append(centerY)
      

      self.f.write("\n" + "\t" + '<!-- BEGIN VESSEL '+str(self.vesselID_of_udv)+"-->")
      self.f.write("\n" + "\t" + '<group ns="vessel' +str(self.vesselID_of_udv)+'">')#vessels name will be vesseli. Whatever the value of i is in this iteration of the loop
      self.f.write("\n" + "\t"+ "\t"  + '<include file="$(find multiple_vessels)/launch/one_vessel.launch" >')
      #self.f.write("\n" + "\t"+ "\t"  + '<include file="$(find vrx_gazebo)/launch/one_vessel.launch" >')
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="gps_enabled" value="'+str(self.gps_enabled_udv)+'" />')
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="imu_enabled" value="'+str(self.imu_enabled_udv)+'" />')
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="lidar_enabled" value="'+str(self.lidar_enabled_udv)+'" />')
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="camera_enabled" value="'+str(self.camera_enabled_udv)+'" />')
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="x" value="'+str(centerX)+'" />')
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="y" value="'+str(centerY)+'" />')
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="z" value="'+"0.1"+'" />')
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="P" value="'+"0"+'" />')#p = 0 because I don't want to get any USV capsized
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="R" value="'+"0"+'" />') #str(R)
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="Y" value="'+str(Y)+'" />') #
      self.f.write("\n" + "\t"+ "\t"  +"\t"  + '<arg name="namespace"  value="vessel'+str(self.vesselID_of_udv)+'" />')

      self.f.write("\n" + "\t"+ "\t"  +"</include>")

      self.f.write("\n" + "\t"+"</group>")
      
      self.j = self.j+1
      self.vesselID_of_udv +=1
      

    #After the while function is done, it's time to close the file  
    self.f.write("\n"+"</launch>")
    self.f.close()

  """
<xacro:arg name="camera_enabled" default="false" />
  <xacro:arg name="gps_enabled" default="false" />
  <xacro:arg name="imu_enabled" default="false" />
  <xacro:arg name="lidar_enabled" default="false" />
  """

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

if __name__ == '__main__':
  my_spawner_object = USV_SPAWN()
  my_spawner_object.spawn_usv()
  

  file_path =  "~/vrx_ws/src/vrx/MultiVessel_Simulation/multiple_vessels"
  file_full_path= os.path.expanduser(file_path)
  
  #open and read the file after the appending:
  my_spawner_object.f = open(file_full_path+"/launch/vessels.launch", "r")
  print(my_spawner_object.f.read())
