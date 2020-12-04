#!/usr/bin/env python

PACKAGE = 'depth_controller'
import roslib;roslib.load_manifest(PACKAGE)
import rospy

from dynamic_reconfigure.server import Server
from depth_controller.cfg import DepthControlConfig

import threading
from sensor_msgs.msg import FluidPressure
from depth_controller.msg import StateVector2D
from geometry_msgs.msg import AccelWithCovarianceStamped
from geometry_msgs.msg import TwistStamped


class StateEstimatorNode():
   def __init__(self):
      self.data_lock = threading.RLock()

      self.pascal_per_meter = 9.78057e3  # g*rho
      self.surface_pressure = 1.01325e5  # according to gazebo

      self.rho = 2.5
      self.phi = 0.3
      self.tau = 0.1

      self.z_prev = -0.5
      self.z1hat_prev = -0.5
      self.z2hat_prev = 0.0
      self.time_prev = None

      rospy.init_node("state_estimator")
      self.state_pub = rospy.Publisher("state", StateVector2D, queue_size=1)

      #self.vel_sub = rospy.Subscriber("mavros/local_position/velocity_body",
      #                                 TwistStamped,
      #                                 self.on_velocity,
      #                                 queue_size=1)
      self.acc_sub = rospy.Subscriber("mavros/local_position/accel",
                                       AccelWithCovarianceStamped,
                                       self.on_acceleration,
                                       queue_size=1)
      
      self.tune_parameters = True
      self.server = Server(DepthControlConfig, self.server_callback)

   def server_callback(self, config, level):
      with self.data_lock:
         if self.tune_parameters:
            self.tune_parameters = config.tune_parameters
            rospy.loginfo("New Parameters received by State Estimator")
               
            self.surface_pressure = config.surface_pressure

            self.rho = config.rho
            self.phi = config.phi
            self.tau = config.tau
      return config

   def on_velocity(self, msg):
      with self.data_lock:
         meas_vel = msg.twist.linear.z
         rospy.loginfo_throttle(5.0, "vel = " + str(meas_vel))
   
   def on_acceleration(self, msg):
      with self.data_lock:
         meas_acc = msg.accel.accel.linear.z
         rospy.loginfo_throttle(5.0, "acc = " + str(meas_acc))
      
   def state_estimation_callback(self, pressure_msg):
      with self.data_lock:
         time = pressure_msg.header.stamp.to_sec()
         depth = - (pressure_msg.fluid_pressure - self.surface_pressure) / self.pascal_per_meter
         if self.time_prev is None:
            velocity = 0.0
         else:
            del_time = time - self.time_prev
            velocity = self.calculate_z2hat(depth, del_time)
         msg = StateVector2D()
         msg.header.stamp = rospy.Time.now()
         msg.position = depth
         msg.velocity = velocity
         self.state_pub.publish(msg)
         self.time_prev = time

   def calculate_z2hat(self, z, del_t):
      z1hat = self.z1hat_prev + del_t*self.z2hat_prev
      z2hat = self.z2hat_prev + (del_t/self.tau) * (-self.z2hat_prev-self.rho*self.sat((self.z1hat_prev-self.z_prev)/self.phi))
      self.z_prev = z
      self.z1hat_prev = z1hat
      self.z2hat_prev = z2hat
      return z2hat

   def sat(self, x):
      return min(1.0, max(-1.0, x))


def main():
   node = StateEstimatorNode()
   pressure_sub = rospy.Subscriber("pressure", FluidPressure,
                                    node.state_estimation_callback)
   rospy.spin()


if __name__ == "__main__":
   main()
