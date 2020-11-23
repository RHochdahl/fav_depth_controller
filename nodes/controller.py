#!/usr/bin/env python
import rospy
import threading
import math
from mavros_msgs.srv import CommandBool
from std_msgs.msg import Float64
from depth_controller.msg import StateVector2D
from depth_controller.msg import StateVector3D
from depth_controller.msg import ParametersList


class ControllerNode():
    def __init__(self):
        self.e1 = 0.0
        self.e2 = 0.0
        
        self.data_lock = threading.RLock()

        # 0 = PD-Controller, 1 = SMC
        self.controller_type = None

        # PD-Controller, k_d / k_p ~= 0.6
        self.k_p = 9.0
        self.k_d = 5.5

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

        self.no_state_warn_time = 0.0
        self.no_controller_warn_time = 0.0
        self.unsafe_setpoint_warn_time = 0.0
        self.unsafe_setpoint_warn_time = 0.0
        self.unsafe_depth_warn_time = 0.0

        self.deep_depth_limit = -0.8
        self.shallow_depth_limit = -0.1

        self.arm_vehicle()

        rospy.init_node("controller")
        
        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                    Float64,
                                                    queue_size=1)
        self.error_pub = rospy.Publisher("control_error",
                                          StateVector2D,
                                          queue_size=1)

        self.setpoint_sub = rospy.Subscriber("depth_setpoint",
                                            StateVector3D,
                                            self.get_setpoint,
                                            queue_size=1)
        self.state_sub = rospy.Subscriber("state",
                                          StateVector2D,
                                          self.get_current_state,
                                          queue_size=1)
        self.parameters_sub = rospy.Subscriber("parameters",
                                                ParametersList,
                                                self.reset_parameters,
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

    def run(self):
        rate = rospy.Rate(50.0)

        while not (rospy.is_shutdown() or self.shutdown):
            msg = Float64()
            msg.data = self.controller()
            self.vertical_thrust_pub.publish(msg)

            err_msg = StateVector2D()
            err_msg.header.stamp = rospy.Time.now()
            err_msg.position = self.e1
            err_msg.velocity = self.e2
            self.error_pub.publish(err_msg)

            rate.sleep()
        
        if self.shutdown:
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

    def reset_parameters(self, msg):
        with self.data_lock:
            self.controller_type = msg.controller_type

    def controller(self):
        if (self.current_depth is None) or (self.current_velocity is None):
            time = rospy.get_time()
            if (time - self.no_state_warn_time) > 1.0:
                rospy.logwarn("No state information received!")
                self.no_state_warn_time = time
            return 0.0

        if self.controller_type is None:
            time = rospy.get_time()
            if (time - self.no_controller_warn_time) > 1.0:
                rospy.logwarn("No controller chosen!")
                self.no_controller_warn_time = time
            return 0.0

        # return 0.0 if depth or setpoint is 'unsafe'
        if ((self.desired_depth < self.deep_depth_limit) or (self.desired_depth > self.shallow_depth_limit)):
            time = rospy.get_time()
            if (time - self.unsafe_setpoint_warn_time) > 1.0:
                rospy.logwarn("Depth setpoint outside safe region!")
                self.unsafe_setpoint_warn_time = time
            return 0.0

        # return 0.0 if depth or setpoint is 'unsafe'
        if ((self.current_depth < self.deep_depth_limit) or (self.current_depth > self.shallow_depth_limit)):
            time = rospy.get_time()
            if (time - self.unsafe_depth_warn_time) > 1.0:
                rospy.logwarn("Diving depth outside safe region!")
                self.unsafe_depth_warn_time = time
            return 0.0


        if self.controller_type == 0:
            # PD-Controller
            self.e1 = self.desired_depth - self.current_depth
            self.e2 = self.desired_velocity - self.current_velocity
            u = self.k_p * self.e1 + self.k_d * self.e2

        elif self.controller_type == 1:
            # SMC
            self.e1 = self.current_depth - self.desired_depth
            self.e2 = self.current_velocity - self.desired_velocity
            s = self.e2 + self.Lambda*self.e1
            u = self.alpha*(self.desired_acceleration-self.Lambda*self.e2-self.kappa*(s/(abs(s)+self.epsilon)))            

        else:
            rospy.logwarn("\nError! Undefined Controller chosen.\n")
            return 0.0

        return self.sat(u)

    def sat(self, x):
        return min(max(x, -1), 1)

def main():
   node = ControllerNode()
   node.run()


if __name__ == "__main__":
   main()
