# JSON Files

In this directory, JSON files that are responsible for configurations of the simulation and global waypoints for vessels and user defined vessels are apparent. <br/>
* `config.json` file consists the configurations for the simulation, Multiple Vessel ROS nodes and logger node. 
* `config_user_defined_vessels.json` consists the configurations for User Defined Vessels and their ROS nodes. 
* `Json_Global_Waypoints.json` conists the global waypoints for each vessel that are gathered from real world AIS data. Also, this file consists information about the simulation world such as coordinates of the origin point ,top right point and bottom left point of the map. The time interval of the gathered AIS data is also given with `time_start` and `time_end` values. 
* `Global_Waypoints_userDefinedVessels.json` conists the waypoints for user defined vessels. If user defined vessels don't utilize the global waypoints in their approach, this file only consists the spawning positions for every vessel.
