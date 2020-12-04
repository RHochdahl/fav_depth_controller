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
from depth_controller.msg import StateVector2D
from depth_controller.msg import StateVector3D
from depth_controller.msg import ParametersList


class ControllerNode():
    def __init__(self):
        self.e1 = None
        self.e2 = 0.0
        
        self.data_lock = threading.RLock()

        # 0 = PD-Controller, 1 = SMC
        self.controller_type = 0

        # PD-Controller, k_d / k_p ~= 0.6
        self.k_p = 9.0
        self.k_d = 5.8

        # SMC
        self.alpha = 1.5
        self.Lambda = 1.5
        self.kappa = 2.5
        self.epsilon = 0.4

        self.desired_depth = -0.5
        self.desired_velocity = 0.0
        self.desired_acceleration = 0.0
        self.current_depth = None
        self.current_velocity = None

        self.shutdown = False

        self.state_msg_time = 0.0

        self.max_msg_timeout = 0.1

        self.deep_depth_limit = -0.8
        self.shallow_depth_limit = -0.1

        # parameters to determine control offset to negate net bouyancy
        # gazebo net bouyancy =~ 0.04
        self.min_setup_time = 30.0
        self.max_setup_time = 60.0
        self.max_ss_error = 0.01
        self.k_i = 0.2 # 1.0
        self.simulated_offset = 0.0 # abs(...) < 0.67
        self.controller_offset = 0.0
        self.integrator_buffer = 0.0
        self.depth_error_list = []
        self.offset_list = []

        self.arm_vehicle()

        rospy.init_node("controller")
        
        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                    Float64,
                                                    queue_size=1)
        self.error_pub = rospy.Publisher("control_error",
                                          StateVector2D,
                                          queue_size=1)
        self.controller_ready_pub = rospy.Publisher("controller_ready",
                                          Bool,
                                          queue_size=1)
        self.state_sub = rospy.Subscriber("state",
                                          StateVector2D,
                                          self.get_current_state,
                                          queue_size=1)

        self.report_readiness(False)
        self.determine_offset()
        self.report_readiness(True)

        self.tune_parameters = True
        self.server = Server(DepthControlConfig, self.server_callback)

        self.parameters_sub = rospy.Subscriber("parameters",
                                                ParametersList,
                                                self.reset_parameters,
                                                queue_size=1)
        self.setpoint_sub = rospy.Subscriber("depth_setpoint",
                                            StateVector3D,
                                            self.get_setpoint,
                                            queue_size=1)

    def arm_vehicle(self): 
        # wait until the arming serivce becomes available
        rospy.wait_for_service("mavros/cmd/arming")
        # connect to the service
        arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        # call the service to arm the vehicle until service call was successfull
        while not arm(True).success:
            rospy.logwarn("Could not arm vehicle. Keep trying.")
            rospy.sleep(1.0)
        rospy.loginfo("Armed successfully.")

    def send_control_message(self, u):
        msg = Float64()
        msg.data =u + self.simulated_offset
        self.vertical_thrust_pub.publish(msg)

    def publish_error(self):
        err_msg = StateVector2D()
        err_msg.header.stamp = rospy.Time.now()
        err_msg.position = self.e1
        err_msg.velocity = self.e2
        self.error_pub.publish(err_msg)

    def report_readiness(self, ready_bool):
        msg = Bool()
        msg.data = ready_bool
        self.controller_ready_pub.publish(msg)

    def server_callback(self, config, level):
        with self.data_lock:
            if self.tune_parameters:
                self.tune_parameters = config.tune_parameters
                rospy.loginfo("New Parameters received by Controller")

                self.k_p = config.k_p
                self.k_d = config.k_d

                self.alpha = config.alpha
                self.Lambda = config.Lambda
                self.kappa = config.kappa
                self.epsilon = config.epsilon
        return config

    def run(self):
        rate = rospy.Rate(50.0)

        while not (rospy.is_shutdown() or self.shutdown):
            u = self.controller()
            self.send_control_message(u)

            self.publish_error()

            rate.sleep()
        
        if self.shutdown:
            self.report_readiness(False)
            self.controller_type = 1
            rospy.loginfo("\nStabilizing Vehicle at final position...")
            while not (rospy.is_shutdown()) and ((abs(self.e1) > 0.1) or (abs(self.e2) > 0.2)):
                msg = Float64()
                msg.data = self.controller()
                self.vertical_thrust_pub.publish(msg)

            if not rospy.is_shutdown():
                for i in range(10):
                    msg = Float64()
                    msg.data = 0.0
                    self.vertical_thrust_pub.publish(msg)
                rospy.loginfo("\nShutdown complete.")

    def get_setpoint(self, msg):
        with self.data_lock:
            self.desired_depth = msg.position
            self.desired_velocity = msg.velocity
            self.desired_acceleration = msg.acceleration
    
    def get_current_state(self, msg):
        with self.data_lock:
            self.current_depth = msg.position
            self.current_velocity = msg.velocity
            self.state_msg_time = rospy.get_time()

    def reset_parameters(self, msg):
        with self.data_lock:
            self.controller_type = msg.controller_type
            self.shutdown = msg.shutdown

    def controller(self):
        if (rospy.get_time() - self.state_msg_time > self.max_msg_timeout):
            rospy.logwarn_throttle(1.0, "No state information received!")
            return 0.0

        if self.controller_type is None:
            rospy.logwarn_throttle(1.0, "No controller chosen!")
            return 0.0

        # return 0.0 if depth or setpoint is 'unsafe'
        if ((self.desired_depth < self.deep_depth_limit) or (self.desired_depth > self.shallow_depth_limit)):
            rospy.logwarn_throttle(1.0, "Depth setpoint outside safe region!")
            return 0.0

        # return 0.0 if depth or setpoint is 'unsafe'
        if ((self.current_depth < self.deep_depth_limit) or (self.current_depth > self.shallow_depth_limit)):
            rospy.logwarn_throttle(5.0, "Diving depth outside safe region!")
            return 0.0
        
        delta_t = rospy.get_time() - self.time
        self.time = rospy.get_time()
        
        if self.controller_type == 0:
            # PD-Controller
            self.e1 = self.desired_depth - self.current_depth
            self.e2 = self.desired_velocity - self.current_velocity
            u = self.k_p * self.e1 + self.k_d * self.e2 + self.controller_offset

        elif self.controller_type == 1:
            # SMC
            self.e1 = self.desired_depth - self.current_depth
            self.e2 = self.desired_velocity - self.current_velocity
            s = self.e2 + self.Lambda*self.e1
            u = self.alpha*(self.desired_acceleration+self.Lambda*self.e2+self.kappa*(s/(abs(s)+self.epsilon))) + self.controller_offset
            
        elif self.controller_type == 2:
            # integral-SMC
            self.e1 = self.desired_depth - self.current_depth
            self.e2 = self.desired_velocity - self.current_velocity
            self.integrator_buffer = self.sat(self.integrator_buffer+delta_t*self.sat(self.e1, 0.05))
            s = self.e2 + 2*self.Lambda*self.e1 + pow(self.Lambda, 2) * self.integrator_buffer
            u = self.alpha*(self.desired_acceleration+2*self.Lambda*self.e2+pow(self.Lambda, 2)*self.e1+self.kappa*(s/(abs(s)+self.epsilon)))

        elif self.controller_type == 3:
            # PID-Controller
            self.e1 = self.desired_depth - self.current_depth
            self.e2 = self.desired_velocity - self.current_velocity
            self.integrator_buffer = self.sat(self.integrator_buffer+delta_t*self.sat(self.e1, 0.05))
            u = self.k_p * self.e1 + self.k_d * self.e2 + self.k_i * self.integrator_buffer

        elif self.controller_type == 4:
            # SMC with separate i
            self.e1 = self.desired_depth - self.current_depth
            self.e2 = self.desired_velocity - self.current_velocity
            self.integrator_buffer = self.sat(self.integrator_buffer+delta_t*self.sat(self.e1, 0.05))
            s = self.e2 + self.Lambda*self.e1
            u = self.alpha*(self.desired_acceleration+self.Lambda*self.e2+self.kappa*(s/(abs(s)+self.epsilon))) + self.k_i * self.integrator_buffer
            
        else:
            rospy.logerr_throttle(10.0, "\nError! Undefined Controller chosen.\n")
            return 0.0

        return self.sat(u)

    def determine_offset(self):
        self.integrator_buffer = 0
        del self.depth_error_list[:]
        del self.offset_list[:]
        start_time = rospy.get_time()
        prev_time = start_time
        time = start_time
        rate = rospy.Rate(50.0)
        rospy.loginfo("Setting up Controller...")
        mean_error = 1.0
        while ((mean_error > self.max_ss_error) or ((time-start_time) < self.min_setup_time)) and ((time-start_time) < self.max_setup_time):
            u_controller = self.controller()
            if self.e1 is not None:
                self.integrator_buffer = self.sat(self.integrator_buffer+(time-prev_time)*self.e1)
                self.depth_error_list.append(abs(self.e1))
                if len(self.depth_error_list) > 100:
                    del self.depth_error_list[0]
                sum = 0.0
                for error_entry in self.depth_error_list:
                    sum += error_entry
                mean_error = sum / len(self.depth_error_list)
                # rospy.loginfo("\ndepth = " + str(self.current_depth) + "\nrunning average of error = " + str(mean_error) + "\nintegrator buffer = " + str(self.integrator_buffer))
                u = self.sat(u_controller + self.k_i * self.integrator_buffer)
                self.send_control_message(u)
                self.publish_error()
                self.offset_list.append(self.k_i*self.integrator_buffer)
                if len(self.offset_list) > 20:
                    del self.offset_list[0]
            else:
                start_time = rospy.get_time()
            rate.sleep()
            prev_time = time
            time = rospy.get_time()
        self.controller_offset += self.k_i * self.integrator_buffer
        sum = 0.0
        for offset_entry in self.offset_list:
            sum += offset_entry
        mean_offset = sum / len(self.offset_list)
        rospy.loginfo("\nController setup completed." + 
                      "\noffset:\t" + str(self.controller_offset) +    
                      "\nmean offset:\t" + str(mean_offset) + 
                      "\nmean error:\t" + str(mean_error))
        self.controller_type = None

    def sat(self, x, limit=1.0):
        return min(max(x, -limit), limit)

def main():
   node = ControllerNode()
   node.run()


if __name__ == "__main__":
   main()
