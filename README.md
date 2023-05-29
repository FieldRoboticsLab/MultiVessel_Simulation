# MultiVessel_Simulation

Welcome to GitHub repository of the Multiple Vessels Simulation environment. This repository consists of the source code and documentation of this project. This project is designed as a ROS package for the VRX-classic simulation environment.

This project is developed in order to test and verify
motion planning algorithms for unmanned surface vehicles under
realistic scenarios where multiple vessels travel around as they
do in the real world. Once a planning algorithm is developed for
USVs, it is crucial to test a planning algorithm under scenarios that closely resemble real world conditions, which include environmental disturbances and real marine traffic. 
To this end, we present an open-source Virtual RobotX-based simulation
environment that enables researchers to add an arbitrary number
of vessels and manually defined their routes, or automatically
generate them from AIS data. Each vessel is equipped with global
and local motion controllers complying with COLREGs.

# How to Install and Run the Project

In this section, we describe how to install and run this project. 

## Recommended Software Setup

The recommended software setup for using this project is below:
* Ubuntu Desktop 20.04 Focal (64-bit)
* Gazebo 11.6.0+
* ROS Noetic

## Depencendies

Because of the accurately modeled vessel and sea surface dynamics including
environmental disturbances such as waves, water current and
winds, we built this project as a ROS package for the VRX Simulation environment. <br/>
For the tutorials about installing the VRX Classic, please refer to [here](https://github.com/osrf/vrx/wiki/vrx_classic_system_setup_tutorials).<br/>
After downloading the VRX classic, it is recommended to edit the `.bashrc` file to include source command to the vrx_ws workspace. 
You can open a terminal and enter the command below to open the .bashrc file in  your Ubuntu:
```
gedit ~/.bashrc
```
At the end of the .bashrc file, please open a new line by pressing enter and enter the command below and save the file by CTRL+S or save button at the top right of the gedit window:
```
source ~/vrx_ws/devel/setup.bash
```
By adding this command, you don't need to source the workspace for every new terminal window.
 <br/>

To install all the dependencies of this project, please enter the commands below:
```
pip install matplotlib
pip install basemap
pip install geopy
pip install transforms3d
```

## Installation of Multiple Vessels package

After the VRX Classic and all the necessary depencencies are installed, you can install the Multiple Vessels package in this repository by following the steps below:
* Download the `multiple_vessels` file from this repository in the workspace that VRX Classic is installed. After you open a terminal, enter these commands below:
```
cd ~/vrx_ws/src/vrx
git clone https://github.com/FieldRoboticsLab/MultiVessel_Simulation.git
```

Upon downloading the package, use `catkin_make` to build the software
```
cd ~/vrx_ws
catkin_make
```


## Launching the Multiple Vessels Simulation Environment
In order to run the simulation with multiple vessels, you can go to the `scripts` directory of the package and run the `multiple_vessel_simulation_starter.py` python script.
```
cd ~/vrx_ws/src/vrx/multiple_vessels/scripts
python multiple_vessel_simulation_starter.py
```
Upon entering this command, the script should initiate the process of starting the simulation environment with default AIS data. The default AIS data consist of 4 vessels. 

## Running the Multiple Vessels ROS Nodes
From the `simulation_config.json` file in the `json_files` directory of the package, you can set which ROS nodes to start automatically upon launching the simulation environment. 
In default settings, ROS nodes doesn't start running upon launching the simulation environment.
In order to start the ROS nodes with one command, you can use `multiple_vessel_node_starter.py` python script. Every Multiple Vessels ROS node is started in a new tab and in the correct order by this python script. You can change which ROS nodes to start with the `multiple_vessel_node_starter.py` python script from `simulation_config.json` file in the `json_files` directory. <br/>

If you want to manually start the ROS nodes, you can open new terminal windows and enter these commands in the given order below:
```
python multiple_vessel_global_path_planner_node.py
python perception_pose_aggregator_node.py
python multiple_vessel_ship_domain_node.py
python multiple_vessel_switch_mechanism.py
python multiple_vessel_local_path_planner_RRT.py
python multiple_vessel_trajectory_tracker.py
python multiple_vessel_controller_purePursuit.py
python simulation_logger_node.py
```

# Detailed Information about the Project

In this section, we describe how the ROS nodes, launch files, scripts etc. are designed and works. 

## Multiple Vessels ROS Nodes
In this section, we describe every Multiple Vessels ROS Node. These ROS Nodes are designed in order to 

### Global Path Planner Node
Global paths for the vessels are set to ROS parameters under the name of `/VesselX/Global_Trajectory` by this ROS node. Also, indexes of the last reached global waypoints are stored by the `/VesselX/Last_Reached_Global_Wp` ROS parameters. This ROS node doesn't update the `/VesselX/Last_Reached_Global_Wp` ROS parameters. Instead, Trajectory Tracker Node is used to track both local and global paths.

### Perception Module
Perception module is designed as two ROS nodes. First ROS node of the Perception Module is `Perception Pose Aggregator Node` subscribes to every vessels GPS and AHRS sensor and combines the pose information into two ROS topics named `/Simulation/GPS_List` and `/Simulation/IMU_List`. `Perception Detect Vessels in Sensing Range Node` subcscribes to these two ROS topics and generates the `/VesselX/Perception` ROS topics. These topics 

#### Perception Pose Aggregator Node
#### Perception Detect Vessels in Sensing Range Node

### Ship Domain Node
### Switch Mechanism Node
### Local Path Planner Node
### Trajectory Tracker Node
### Controller Node
### Logger Node

## JSON Files

JSON files that are responsible for configurations of the simulation and global waypoints for vessels and user defined vessels are apparent. <br/>
* `config.json` file consists the configurations for the simulation, Multiple Vessel ROS nodes and logger node. 
* `config_user_defined_vessels.json` consists the configurations for User Defined Vessels and their ROS nodes. 
* `Json_Global_Waypoints.json` conists the global waypoints for each vessel that are gathered from real world AIS data. Also, this file consists information about the simulation world such as coordinates of the origin point ,top right point and bottom left point of the map. The time interval of the gathered AIS data is also given with `time_start` and `time_end` values. 
* `Global_Waypoints_userDefinedVessels.json` conists the waypoints for user defined vessels. If user defined vessels don't utilize the global waypoints in their approach, this file only consists the spawning positions for every vessel.

## Launch Files and Scripts

We designed the simulation environment to launch by `multiple_vessel_simulation_starter.py` Python script. This script :
* Sets the ROS parameters about vessel details. These details are based on AIS data provided by `Json_Global_Waypoints.json`.
* Sets the simulation world details such as the top right and bottom left points' coordinates and size of the water surface.
* Sets the ROS parameters about the ROS nodes. These parameters are provided to this script from the `config.json` file.
* Calls the `vessel_spawner.py` Python script to spawn the vessels.
<br/>
The `vessel_spawner.py` Python Script:
* Re-writes the `vessels.launch` file based on the initial positions of the vessels provided by `Json_Global_Waypoints.json` file. Also, enabled sensors are written to `vessels.launch` file for each vessel. 
<br/>
After the `vessels.launch` file is generated based on `Json_Global_Waypoints.json` file, `vrx_multivessel_parametric.launch` is launched by `multiple_vessel_simulation_starter.py` Python script.
<br/>
`vrx_multivessel_parametric.launch` Launch file launches the simulation world and calls `vessels.launch` Launch file to spawn vessels. `vessels.launch` spawns vessels by calling `one_vessel.launch` with arguments. 
<br/>


## World Files
As described in the Launch Files and Scripts section, simulation world named `multiple_vessels.world` is generated by `multiple_vessel_simulation_starter.py` based on the map information provided by `Json_Global_Waypoints.json` file. In this world file, coordinates of the origin of the map and size of the map are stored. 

## multivessel_msgs

We defined custom message types such as `Perception.msg`, `VesselDetails.msg` and `VesselPose.msg` under the name of `multivessel_msgs`. These messages are used by the Multiple Vessels ROS nodes to communicate the vessel pose and details. ROS topics under the name of /VesselX/Perception and /VesselX/Vessels_In_Ship_Domain are being published to with `Perception.msg` message type.

## vessel_gazebo 
In order to spawn vessels inside the simulation world, `vessel_gazebo.urdf.xacro` file is used. We used the unmanned surface vehicles called WAM-V for modelling the vessels in the simulation. T
## User Defined Vessels


## Reference

For the detailed descriptions of the algorithms used in this peoject, please refer to our publication. 
Also, if you use the Multiple Vessels simulation in your work, please cite our publication, [COLREG-Compliant Simulation Environment for Verifying USV Motion Planning Algorithms](publication_link_to_here): 

```
@InProceedings{bayrak23COLREG-Compliant,
  Title                    = {COLREG-Compliant Simulation Environment for Verifying USV Motion Planning Algorithms},
  Author                   = {Mustafa Bayrak and Haluk Bayram},
  Booktitle                = {Proceedings of MTS/IEEE OCEANS Conference},
  Year                     = {2023},
  Address                  = {Limerick, IE},
  Month                    = {June}
}
```

