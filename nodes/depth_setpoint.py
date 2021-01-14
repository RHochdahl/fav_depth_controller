#!/usr/bin/env python
import rospy
import threading
import math
from fav_depth_controller.msg import StateVector3D
from fav_depth_controller.msg import ParametersList


class DepthSetpointNode():
    def __init__(self):
        self.data_lock = threading.RLock()

        # 0 = sine wave, 1 = step wave, 2 = ramp wave, else = self.mean
        self.setpoint_trajectory = None

        # setpoint frequency
        self.frequency = 0.15  # <= 0.15
        self.period_time = 1.0/self.frequency
        if self.setpoint_trajectory == 0:
            self.omega = 2*math.pi*self.frequency

        # wave amplitude and mean
        self.amplitude = 0.2
        self.mean = -0.5

        self.shutdown = False
        self.final_position = -0.3

        rospy.init_node("depth_setpoints")

        self.setpoint_pub = rospy.Publisher("depth_setpoint",
                                            StateVector3D,
                                            queue_size=1)

        self.parameters_sub = rospy.Subscriber("parameters",
                                                ParametersList,
                                                self.reset_parameters,
                                                queue_size=1)

        self.init_time = rospy.get_time()

    def reset_parameters(self, msg):
        with self.data_lock:
            self.setpoint_trajectory = msg.setpoint_trajectory
            self.shutdown = msg.shutdown
            if not self.shutdown:
                self.frequency = round(msg.setpoint_frequency, 4)
                self.period_time = 1.0/self.frequency
                if self.setpoint_trajectory == 0:
                    self.omega = 2*math.pi*self.frequency
                self.amplitude = msg.setpoint_amplitude
                self.mean = msg.setpoint_mean
                self.init_time = rospy.get_time()

    def run(self):
        rate = rospy.Rate(50.0)
        while self.setpoint_trajectory is None:
            pass
        while not (rospy.is_shutdown() or self.shutdown):
            msg = StateVector3D()
            msg.header.stamp = rospy.Time.now()
            # fill msg with example setpoint
            t = rospy.get_time() - self.init_time
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
            elif self.setpoint_trajectory == 2:
                pos, vel = self.ramp_wave(t)
                msg.position = pos
                msg.velocity = vel
                msg.acceleration = 0.0
            else:
                msg.position = self.mean
                msg.velocity = 0.0
                msg.acceleration = 0.0
            self.setpoint_pub.publish(msg)
            rate.sleep()
        if (self.shutdown) and (not rospy.is_shutdown()):
            msg = StateVector3D()
            msg.header.stamp = rospy.Time.now()
            msg.position = self.final_position
            msg.velocity = 0.0
            msg.acceleration = 0.0
            self.setpoint_pub.publish(msg)
    
    def sine_wave(self, t):
        pos = self.mean + self.amplitude * math.sin(self.omega*t)
        vel = self.omega*self.amplitude * math.cos(self.omega*t)
        acc = -pow(self.omega, 2)*self.amplitude * math.sin(self.omega*t)
        return pos, vel, acc
    
    def step_wave(self, t):
        t_star = t % self.period_time
        if t_star <= (self.period_time/2):
            pos = self.mean + self.amplitude
        else:
            pos = self.mean - self.amplitude
        return pos

    def ramp_wave(self, t):
        t_star = t % self.period_time
        if t_star <= (self.period_time/2):
            pos = self.mean + self.amplitude * (1 - 4*self.frequency*t_star)
            vel = -4*self.frequency*self.amplitude
        else:
            pos = self.mean + self.amplitude * (-3 + 4*self.frequency*t_star)
            vel = 4*self.frequency*self.amplitude
        return pos, vel


def main():
    node = DepthSetpointNode()
    node.run()


if __name__ == "__main__":
    main()
