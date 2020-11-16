#!/usr/bin/env python
import rospy

class DepthControlNode():
    def __init__(self):
        rospy.init_node("depth_control")
        self.data_lock = threading.RLock()

        self.depth_setpoint = -0.5
        self.depth_velocity_setpoint = 0.0
        self.controller = Controller()
        self.pid_gains = [1.0, 0.0, 0.0]

        self.t_last = rospy.Time.now()

        self.thrust_pub = rospy.Publisher("vertical_thrust",
                                          Float64,
                                          queue_size=1)

        self.dyn_server = Server(PidConfig, self.on_reconfigure)

        self.depth_sub = rospy.Subscriber("depth",
                                          DepthEKFStamped,
                                          self.on_depth,
                                          queue_size=1)

        self.setpoint_sub = rospy.Subscriber("depth_setpoint",
                                             Float64,
                                             self.on_depth_setpoint,
                                             queue_size=1)

    def on_depth(self, msg):
        now = rospy.Time.now()
        depth = msg.depth
        velocity = msg.z_vel
        with self.data_lock:
            dt = now - self.t_last
            self.t_last = now
            u, error, derror = self.compute_control_output(
                depth, velocity, dt.to_sec())

        msg = Float64(u)
        self.thrust_pub.publish(msg)

    def on_depth_setpoint(self, msg):
        with self.data_lock:
            self.depth_setpoint = msg.data

    def compute_control_output(self, depth, depth_velocity, dt):
        error = self.depth_setpoint - depth
        derror = self.depth_velocity_setpoint - depth_velocity
        u = self.controller.update(error, derror, dt)
        return u, error, derror

    def on_reconfigure(self, config, level):
        with self.data_lock:
            self.pid_gains[0] = config["p_gain"]
            self.pid_gains[1] = config["i_gain"]
            self.pid_gains[2] = config["d_gain"]
        self.update_pid_gains()
        return config

    def update_pid_gains(self):
        with self.data_lock:
            self.controller.p_gain = self.pid_gains[0]
            self.controller.i_gain = self.pid_gains[1]
            self.controller.d_gain = self.pid_gains[2]


def main():
    node = DepthControlNode()
    rospy.spin()


if __name__ == "__main__":
    main()
