#!/usr/bin/env python

PACKAGE = 'depth_controller'
import roslib;roslib.load_manifest(PACKAGE)
import rospy

from dynamic_reconfigure.server import Server
from depth_controller.cfg import DepthControlConfig
import threading
import math
from mavros_msgs.srv import CommandBool
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from depth_controller.msg import ParametersList


class ParametersNode():
    def __init__(self):
        self.data_lock = threading.RLock()

        self.tune_parameters = True
        
        self.experiment_time = 60.0

        # 0 = PD-Controller, 1 = SMC
        self.controller_types = [0, 1]
        # 0 = sine wave, 1 = step wave, 2 = ramp wave, else = self.mean
        self.setpoint_frequencies = [[0.1, 0.15], [0.1], [0.15]]  # <= 0.15

        self.controller_names = ["PD-Controller", "Sliding Mode Controller"]
        self.trajectory_names = ["Sine Wave", "Step Wave", "Ramp Wave"]

        rospy.init_node("experiment_parameters")
        
        self.parameters_pub = rospy.Publisher("parameters",
                                                ParametersList,
                                                queue_size=1)
        
        self.controller_ready_sub = rospy.Subscriber("controller_ready",
                                          Bool,
                                          self.get_controller_status,
                                          queue_size=1)
        self.controller_ready = False

        self.server = Server(DepthControlConfig, self.server_callback)

    def server_callback(self, config, level):
        with self.data_lock:
            if self.tune_parameters and self.controller_ready:
                self.tune_parameters = config.tune_parameters
                rospy.loginfo("New Parameters received by Manager")
                
                msg = ParametersList()
                msg.header.stamp = rospy.Time.now()
                msg.controller_type = config.controller_type
                msg.setpoint_trajectory = config.trajectory
                msg.setpoint_frequency = config.frequency
                msg.setpoint_mean = config.mean
                msg.setpoint_amplitude = config.amplitude
                msg.shutdown = False
                self.parameters_pub.publish(msg)
        return config

    def get_controller_status(self, msg):
        with self.data_lock:
            self.controller_ready = msg.data

    def run(self):
        while not self.controller_ready:
            rospy.sleep(1.0)
        
        while (self.tune_parameters) and (not rospy.is_shutdown()):
            rospy.loginfo_throttle(30.0, "\nTuning Parameters...")
            rospy.sleep(1.0)

        experiment_count = 1
        num_experiments = len(self.controller_types)*sum([len(freq_list) for freq_list in self.setpoint_frequencies])
        for controller_type in self.controller_types:
            for setpoint_trajectory in range(len(self.setpoint_frequencies)):
                for setpoint_frequency in self.setpoint_frequencies[setpoint_trajectory]:
                    if not rospy.is_shutdown():
                        self.go_to_start_position(controller_type)
                        msg = ParametersList()
                        msg.header.stamp = rospy.Time.now()
                        msg.controller_type = controller_type
                        msg.setpoint_trajectory = setpoint_trajectory
                        msg.setpoint_frequency = setpoint_frequency
                        msg.setpoint_mean = -0.5
                        msg.setpoint_amplitude = 0.2
                        msg.shutdown = False
                        self.parameters_pub.publish(msg)

                        rospy.loginfo("\nStarting new experiment %i/%i\nwith controller:\t%s\nand trajectory:\t\t%s at %1.2f Hz" 
                                        %(experiment_count, num_experiments, self.controller_names[controller_type], self.trajectory_names[setpoint_trajectory], setpoint_frequency))
                        
                        experiment_count += 1
                        rospy.sleep(self.experiment_time + 1.0)
        
        if not rospy.is_shutdown():
            rospy.loginfo("\nExperimens concluded. Shutting down...")
            msg = ParametersList()
            msg.header.stamp = rospy.Time.now()
            msg.controller_type = 1
            msg.setpoint_trajectory = 1
            msg.setpoint_frequency = 0.1
            msg.shutdown = True
            self.parameters_pub.publish(msg)

    def go_to_start_position(self, controller_type):
        msg = ParametersList()
        msg.header.stamp = rospy.Time.now()
        msg.controller_type = controller_type
        msg.setpoint_trajectory = 7
        msg.setpoint_frequency = 1.0
        msg.shutdown = False
        self.parameters_pub.publish(msg)
        rospy.sleep(10.0)


def main():
   node = ParametersNode()
   node.run()


if __name__ == "__main__":
   main()
