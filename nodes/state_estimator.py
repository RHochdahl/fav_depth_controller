#!/usr/bin/env python

PACKAGE = 'fav_depth_controller'
import roslib;roslib.load_manifest(PACKAGE)
import rospy
import numpy as np

from dynamic_reconfigure.server import Server
from fav_depth_controller.cfg import DepthControlConfig

import tf_conversions as tf

import threading
from sensor_msgs.msg import FluidPressure
from fav_depth_controller.msg import StateVector2D
from geometry_msgs.msg import AccelWithCovarianceStamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from range_sensor.msg import RangeMeasurementArray


class StateEstimatorNode():
   def __init__(self):
      self.data_lock = threading.RLock()

      self.simulate = rospy.get_param("simulate")

      self.pascal_per_meter = 9.78057e3  # g*rho
      if self.simulate:
         self.surface_pressure = 1.01325e5  # according to gazebo
      else:
         self.surface_pressure = None

      self.rho = 2.5
      self.phi = 0.3
      self.tau = 0.1

      self.velocity = np.array([0.0, 0.0, 0.0])
      self.x_prev = np.array([0.0, 0.0, -0.5])
      self.x1hat_prev = np.array([0.0, 0.0, -0.5])
      self.x2hat_prev = np.array([0.0, 0.0, 0.0])
      self.prev_smo_time = None

      self.time_motion = None
      self.mu = np.matrix([[0.0],
                           [0.0],
                           [0.0],
                           [0.0],
                           [0.0],
                           [0.0]])
      self.mu_prior = self.mu
      self.sigma = np.diag([0.1, 0.1, 0.1, 0.5, 0.5, 0.5])
      self.sigma_prior = self.sigma
      self.R = np.array([[0.0, 0.0],
                           [0.0, 0.1]])
      self.Q_press = np.array([[0.01]])
      self.Q_range = np.array([[0.05]])
      
      self.tag_coordinates = [np.array([0.5, 3.35, -0.5]),
                              np.array([1.1, 3.35, -0.9]),
                              np.array([0.5, 3.35, -0.5]),
                              np.array([1.1, 3.35, -0.9])]
      
      rospy.init_node("state_estimator")
      self.state_pub = rospy.Publisher("state", StateVector2D, queue_size=1)
      
      self.pressure_sub = rospy.Subscriber("pressure", FluidPressure, self.on_pressure, queue_size=1)
      # self.imu_sub = rospy.Subscriber("mavros/imu/data", Imu, self.on_imu, queue_size=1)
      # self.range_sub = rospy.Subscriber("ranges", RangeMeasurementArray, self.range, queue_size=1)

      self.tune_parameters = True
      self.server = Server(DepthControlConfig, self.server_callback)

   def run(self):
      rate = rospy.Rate(50.0)

      while not rospy.is_shutdown():
         if self.prev_smo_time is None:
            self.prev_smo_time = rospy.get_time()
         else:
            self.velocity = smo(self.mu[:2])
         rate.sleep()
   
   def publish_state(self):
      with self.data_lock:
         msg = StateVector2D()
         msg.header.stamp = rospy.Time.now()
         msg.position = self.mu[2]
         msg.velocity = self.velocity[2]
         self.state_pub.publish(msg)

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

   def on_range(self, msg):
      with self.data_lock:
         for meas in msg.measurements:
            self.sigma_prior = self.sigma
            self.mu_prior = self.mu
            h = np.sqrt(np.sqare(self.mu_prior[0]-self.tag_coordinates[meas.id][0])+
                        np.sqare(self.mu_prior[1]-self.self.tag_coordinates[meas.id][1])+
                        np.sqare(self.mu_prior[2]-self.tag_coordinates[meas.id][2]))
            H = (1/h) * np.matrix([[self.mu_prior[0]-self.tag_coordinates[meas.id][0],
                                    self.mu_prior[1]-self.tag_coordinates[meas.id][1],
                                    self.mu_prior[2]-self.tag_coordinates[meas.id][2],
                                    0,
                                    0,
                                    0]])
            K = H*self.sigma_prior*np.linalg.inv(H*self.sigma_prior*H.T + self.Q)
            self.mu = self.mu_prior + K*(h-H*self.mu_prior)
            self.sigma = (np.eye(6) - K) * self.sigma_prior

   def on_imu(self, msg):
      with self.data_lock:
         euler = tf.transformations.euler_from_quaternion(msg.orientation)
         self.roll = euler[0]
         self.pitch = euler[1]
         self.yaw = euler[2]
         if self.time_motion is None:
            self.time_motion = msg.header.stamp.to_sec()
         else:
            del_t = msg.header.stamp.to_sec() - self.time_motion
            self.time_motion = msg.header.stamp.to_sec()
            self.mu = self.mu_prior + del_t*np.matrix([[self.mu_prior[0, 4]],
                                                      [self.mu_prior[0, 5]],
                                                      [self.mu_prior[0, 6]],
                                                      [msg.linear_acceleration.x],
                                                      [msg.linear_acceleration.y],
                                                      [msg.linear_acceleration.z]])
            G = np.matrix([[1, 0, 0, del_t, 0, 0],
                           [0, 1, 0, 0, del_t, 0],
                           [0, 0, 1, 0, 0, del_t],
                           [0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1]])
            self.sigma = G*self.sigma*G.T+self.R

   def on_pressure(self, pressure_msg):
      with self.data_lock:
         if self.surface_pressure is None:
            self.surface_pressure = pressure_msg.fluid_pressure
         time = pressure_msg.header.stamp.to_sec()
         depth = - (pressure_msg.fluid_pressure - self.surface_pressure) / self.pascal_per_meter
         self.mu[2] = depth
         self.velocity[2] = self.smo(depth)
         self.publish_state()
         '''
         H = np.matrix([[0, 0, 1, 0, 0, 0]])
         self.sigma_prior = self.sigma
         self.mu_prior = self.mu
         K = H*self.sigma_prior*np.linalg.inv(H*self.sigma_prior*H.T + self.Q)
         self.mu = self.mu_prior + K*(np.matrix([[depth]])-H*self.mu_prior)
         self.sigma = (np.eye(6) - K) * self.sigma_prior
         '''

   def smo(self, x):
      time = rospy.get_time()
      if self.prev_smo_time is None:
         self.prev_smo_time = time
         return 0
      del_t = time - self.prev_smo_time
      self.prev_smo_time = time
      i=2
      x1hat = self.x1hat_prev[i] + del_t*self.x2hat_prev[i]
      x2hat = self.x2hat_prev[i] + (del_t/self.tau) * (-self.x2hat_prev[i]-self.rho*self.sat((self.x1hat_prev[i]-self.x_prev[i])/self.phi))
      self.x_prev[i] = x # [i]
      self.x1hat_prev[i] = x1hat
      self.x2hat_prev[i] = x2hat
      return x2hat

   def sat(self, x):
      return min(1.0, max(-1.0, x))


def main():
   node = StateEstimatorNode()
   # node.run()
   rospy.spin()


if __name__ == "__main__":
   main()
