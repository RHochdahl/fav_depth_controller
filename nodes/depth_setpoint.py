#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64


class DepthSetpointNode():
    def __init__(self):
        rospy.init_node("depth_setpoints")

        self.setpoint_pub = rospy.Publisher("depth_setpoint",
                                            Float64,
                                            queue_size=1)

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            msg = Float64()
            # fill msg with example setpoint
            msg.data = -0.5
            self.setpoint_pub.publish(msg)
            rate.sleep()


def main():
    node = DepthSetpointNode()
    node.run()


if __name__ == "__main__":
    main()
