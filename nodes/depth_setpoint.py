#!/usr/bin/env python
import rospy
import math
from depth_controller.msg import StateVector3D


class DepthSetpointNode():
    def __init__(self):
        # 0 = sine wave, 1 = step wave, else = self.mean
        self.setpoint_trajectory = 1

        # setpoint frequency
        self.frequency = 0.2
        if self.setpoint_trajectory == 0:
            self.omega = 2*math.pi*self.frequency

        # wave amplitude and mean
        self.amplitude = 0.2
        self.mean = -0.5

        rospy.init_node("depth_setpoints")

        self.setpoint_pub = rospy.Publisher("depth_setpoint",
                                            StateVector3D,
                                            queue_size=1)

        self.init_time = rospy.get_time()

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            msg = StateVector3D()
            msg.header.stamp = rospy.Time.now()
            # fill msg with example setpoint
            t = rospy.get_time()
            if self.setpoint_trajectory == 0:
                pos, vel, acc = self.sine_wave(t)
                msg.position = pos
                msg.velocity = vel
                msg.acceleration = acc
            elif self.setpoint_trajectory == 1:
                pos = self.step_wave(t)
                msg.position = pos
                msg.velocity = 0.0
                msg.acceleration = 0.0
            else:
                msg.position = self.mean
                msg.velocity = 0.0
                msg.acceleration = 0.0
            self.setpoint_pub.publish(msg)
            rate.sleep()
    
    def sine_wave(self, t):
        pos = self.mean + self.amplitude * math.sin(self.omega*t)
        vel = self.omega*self.amplitude * math.cos(self.omega*t)
        acc = -pow(self.omega, 2)*self.amplitude * math.sin(self.omega*t)
        return pos, vel, acc
    
    def step_wave(self, t):
        period = ((int) (2*self.frequency * (t - self.init_time))) % 2
        if period == 0:
            pos = self.mean + self.amplitude
        else:
            pos = self.mean - self.amplitude
        return pos

def main():
    node = DepthSetpointNode()
    node.run()


if __name__ == "__main__":
    main()
