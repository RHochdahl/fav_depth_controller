#!/usr/bin/env python
import rospy
from sensor_msgs.msg import FluidPressure
from depth_controller.msg import StateVector2D


class StateEstimatorNode():
   def __init__(self):
      self.pascal_per_meter = 9.78057e3  # g*rho
      self.surface_pressure = 1.01325e5  # according to gazebo

      self.rho = 2.5
      self.phi = 0.3
      self.tau = 0.1

      self.z1hat_prev = 0.5
      self.z2hat_prev = 0.0
      self.time_prev = None

      rospy.init_node("state_estimator")
      self.state_pub = rospy.Publisher("state", StateVector2D, queue_size=1)

   def state_estimation_callback(self, pressure_msg):
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
      z1hat = self.z1hat_prev - del_t*self.rho*self.sat((self.z1hat_prev-z)/self.phi)
      z2hat = self.z2hat_prev + (del_t/self.tau) * (-self.z2hat_prev-self.rho*self.sat((z1hat-z)/self.phi))
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
