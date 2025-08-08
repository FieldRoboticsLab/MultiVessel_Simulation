#!/usr/bin/env python
from __future__ import print_function
import rospy
import json, os, time

class Vessel_Node_Starter(object):
    def __init__(self, vessel_count_):
        self.path = "~/vrx_ws/src/vrx/MultiVessel_Simulation/multiple_vessels"
        self.full_path = os.path.expanduser(self.path)

        with open(os.path.join(self.full_path, 'json_files', 'config.json'), 'r') as f:
            self.cfg = json.load(f)

        self.vessel_count = vessel_count_
        self._compute_choices()
        self._start_nodes()

    # -------- helpers --------
    def _g(self, *keys, default=False):
        """
        Safe getter for nested dict/list with default.
        Usage: self._g("controller", 0, "Pure_Pursuit", 0, "chosen", default=False)
        """
        cur = self.cfg
        try:
            for k in keys:
                cur = cur[k]
            return cur
        except (KeyError, IndexError, TypeError):
            return default

    def _term(self, cmd):
        os.system('gnome-terminal --tab -- bash -c "source ~/vrx_ws/devel/setup.bash; %s; bash"' % cmd)

    def _compute_choices(self):
        # --- Local path planning
        self.rrt_chosen = self._g("local_path_planning", 0, "RRT", 0, "chosen", default=False)

        # --- Controller choices
        self.pp_chosen      = self._g("controller", 0, "Pure_Pursuit",   0, "chosen", default=False)
        self.pid_chosen     = self._g("controller", 0, "PID_Controller", 0, "chosen", default=False)
        self.stanley_chosen = self._g("controller", 0, "Stanley",        0, "chosen", default=False)

        # --- Ship domain
        self.ship_domain_chosen = self._g("ship_domain", 0, "Circular_Ship_Domain", 0, "chosen", default=False)

        # --- Perception / localization (tie to Subsribe_to_Topics -> choosen)
        self.perception_chosen = self._g("Localization", 0, "Subsribe_to_Topics", 0, "choosen", default=False)

        # --- Loggers
        self.sim_logger_on = (
            self._g("simulation_logger", 0, "logging_available",  default=False) or
            self._g("simulation_logger", 0, "plotting_available", default=False)
        )
        self.local_path_plotter_logger_on = (
            self._g("local_path_plotter_logger", 0, "logging_available",  default=False) or
            self._g("local_path_plotter_logger", 0, "plotting_available", default=False)
        )

        # --- Items not represented directly in JSON: allow ROS params
        # Default on for global path planner (common), off for trajectory tracker unless a non-PP controller is chosen,
        # on for switch mechanism if you have both global & local behaviors.
        self.global_path_planner_on = rospy.get_param("~enable_global_path_planner", True)
        self.switch_mechanism_on    = rospy.get_param("~enable_switch_mechanism", True)
        self.trajectory_tracker_on  = rospy.get_param("~enable_trajectory_tracker", True)

    def _start_nodes(self):
        # Global Path Planner
        if self.global_path_planner_on:
            self._term("python3 multiple_vessel_global_path_planner_node.py")
            time.sleep(0.4)

        # Perception (from Localization->Subscribe_to_Topics->choosen)
        if self.perception_chosen:
            self._term("python3 perception_pose_aggregator_node.py")
            time.sleep(0.4)

        # Ship Domain
        if self.ship_domain_chosen:
            self._term("python3 multiple_vessel_ship_domain_node.py")
            time.sleep(0.4)

        # Switch Mechanism (gate between global/local)
        if self.switch_mechanism_on:
            self._term("python3 multiple_vessel_switch_mechanism.py")
            time.sleep(0.4)

        # Local Path Planner (RRT)
        if self.rrt_chosen:
            self._term("python3 multiple_vessel_local_path_planner_RRT.py")
            time.sleep(0.4)

        # Trajectory Tracker (use if PID/Stanley etc.)
        if self.trajectory_tracker_on:
            self._term("python3 multiple_vessel_trajectory_tracker.py")
            time.sleep(0.4)

        # Pure Pursuit Controller (mutually exclusive with traj. tracker in most stacks)
        if self.pp_chosen:
            self._term("python3 multiple_vessel_controller_purePursuit.py")
            time.sleep(0.4)

        # Simulation logger
        if self.sim_logger_on:
            self._term("python3 simulation_logger_node.py")
            time.sleep(0.4)

        # Local-path plotter/logger (optional, if you have a separate node/file for it)
        if self.local_path_plotter_logger_on:
            # Change to your actual script name if different:
            # self._term("python local_path_plotter_logger_node.py")
            pass

if __name__ == '__main__':
    rospy.init_node('multi_vessel_starter', anonymous=True)
    rate = rospy.Rate(10)

    vessel_count = int(rospy.get_param("/vessel_count", 1))
    print("Number of Vessels:", vessel_count)

    Vessel_Node_Starter(vessel_count)
    rospy.spin()
