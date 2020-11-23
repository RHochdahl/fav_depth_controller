#!/usr/bin/env python
import rospy
import math
from mavros_msgs.srv import CommandBool
from std_msgs.msg import Float64
from depth_controller.msg import ParametersList


class ParametersNode():
    def __init__(self):

        self.experiment_time = 30.0

        # 0 = PD-Controller, 1 = SMC
        self.controller_types = [0, 1]
        self.setpoint_trajectories = [0] # [0, 1]
        self.setpoint_frequencies = [0.15] # [0.05, 0.1, 0.15]  # <= 0.15

        self.controller_names = ["PD-Controller", "Sliding Mode Controller"]
        self.trajectory_names = ["Sine Wave", "Step Wave", "Ramp Wave"]

        rospy.init_node("experiment_parameters")
        
        self.parameters_pub = rospy.Publisher("parameters",
                                                    ParametersList,
                                                    queue_size=1)

    def run(self):
        experiment_count = 1
        num_experiments = len(self.controller_types)*len(self.setpoint_trajectories)*len(self.setpoint_frequencies)
        for controller_type in self.controller_types:
            for setpoint_trajectory in self.setpoint_trajectories:
                for setpoint_frequency in self.setpoint_frequencies:
                    if not rospy.is_shutdown():
                        self.go_to_start_position()
                        msg = ParametersList()
                        msg.header.stamp = rospy.Time.now()
                        msg.controller_type = controller_type
                        msg.setpoint_trajectory = setpoint_trajectory
                        msg.setpoint_frequency = setpoint_frequency
                        msg.shutdown = False
                        self.parameters_pub.publish(msg)

                        rospy.loginfo("\nStarting new experiment %i/%i\nwith controller:\t%s\nand trajectory:\t\t%s at %1.2f Hz" 
                                        %(experiment_count, num_experiments, self.controller_names[controller_type], self.trajectory_names[setpoint_trajectory], setpoint_frequency))
                        
                        experiment_count += 1
                        rospy.sleep(self.experiment_time + 1.0)
        
        if not rospy.is_shutdown():
            rospy.loginfo("\nExperimens concluded. Shutting down...")
            for i in range(10):
                msg = ParametersList()
                msg.header.stamp = rospy.Time.now()
                msg.controller_type = 1
                msg.setpoint_trajectory = 1
                msg.setpoint_frequency = 0.1
                msg.shutdown = True
                self.parameters_pub.publish(msg)

    def go_to_start_position(self):
        for i in range(10):
            msg = ParametersList()
            msg.header.stamp = rospy.Time.now()
            msg.controller_type = 1
            msg.setpoint_trajectory = 7
            msg.setpoint_frequency = 0.01
            msg.shutdown = False
            self.parameters_pub.publish(msg)
            rospy.sleep(1.0)

def main():
   node = ParametersNode()
   node.run()


if __name__ == "__main__":
   main()
