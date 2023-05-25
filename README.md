# MultiVessel_Simulation

Welcome to GitHub repository of the Multiple Vessels Simulation environment. This repository consists the source code and documentation of this project. This project is designed as a ROS package for the VRX-classic simulation environment.

This project is developed in order to test and verify
motion planning algorithms for unmanned surface vehicles under
realistic scenarios where multiple vessels travel around as they
do in the real world. Once a planning algorithm is developed for
USVs, it is crucial to test a planning algorithm under scenarios hat closely resemble real world conditions, which include environmental disturbances and real marine traffic. 
To this end, we present an open-source Virtual RobotX-based simulation
environment that enables researchers to add an arbitrary number
of vessels and manually defined their routes, or automatically
generate them from AIS data. Each vessel is equipped with global
and local motion controllers complying with COLREGs.

## Recommended Software Setup

The recommended software setup for using this project is below:
* Ubuntu Desktop 20.04 Focal (64-bit)
* Gazebo 11.6.0+
* ROS Noetic

## Depencendies

Because of the accurately modeled vessel and sea surface dynamics including
environmental disturbances such as waves, water current and
winds, we built this project as a ROS package for the VRX Simulation environment. <br/>
For the tutorials about installing the VRX Classic, please refer to [here](https://github.com/osrf/vrx/wiki/vrx_classic_system_setup_tutorials)
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



## Reference

For detailed descriptions of the algorithms that are used in this peoject, please refer to our publication. 
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

