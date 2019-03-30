#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        # TODO: Figure out if we want them as instance variables
        # Read the parameters
        self.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        self.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        self.brake_deadband = rospy.get_param('~brake_deadband', .1)
        self.decel_limit = rospy.get_param('~decel_limit', -5)
        self.accel_limit = rospy.get_param('~accel_limit', 1.)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        self.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        self.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        self.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        self.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        # Acceleration when the car is idle and no break is applied. This comes
        # from the car having an automatic transmission.
        idle_accel = 1.67  # XXX: From parameter?
        # Hate to do this here, but we don't want the car to creep as we are
        # starting up
        self.idle_break = self.vehicle_mass * idle_accel * self.wheel_radius
        # Set sensible initial values to our control variables
        self.throttle = self.steering = 0
        self.brake = 10000  # We don't want the car to move while we are
                            # starting up!

        # Create the publishers for later on
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Create `Controller` object. It basically uses all the parameters.
        self.controller = Controller(vehicle_mass=self.vehicle_mass,
            fuel_capacity=self.fuel_capacity, brake_deadband=self.brake_deadband,
            decel_limit=self.decel_limit, accel_limit=self.accel_limit,
            wheel_radius=self.wheel_radius, wheel_base=self.wheel_base,
            steer_ratio=self.steer_ratio, max_lat_accel=self.max_lat_accel,
            max_steer_angle=self.max_steer_angle)

        # Flag to know if dbw is enabled
        self.enabled = False
        # The measured speed of the vehicle
        self.current_linear = None
        # The current (last) settings for steering, before calculations
        self.target_linear = None
        self.target_angular = None

        # TODO: Am I missing the waypoints here?
        
        # Subscribe to the topics needed
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.toggle_dbw)
        rospy.Subscriber('/current_velocity', TwistStamped, self.read_velocity)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.read_steer)

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # Don't try to control without observation information
            if not self.current_linear is None \
                    and not self.target_linear is None:
                # TODO: Get predicted throttle, brake, and steering using `twist_controller`
                # You should only publish the control commands if dbw is enabled
                # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
                #                                                     <proposed angular velocity>,
                #                                                     <current linear velocity>,
                #                                                     <dbw status>,
                #                                                     <any other argument you need>)
                self.throttle, self.brake, self.steering = self.controller.control(self.current_linear,
                                                                    self.target_linear,
                                                                    self.target_angular,
                                                                    self.enabled)

            if self.enabled:
                self.publish(self.throttle, self.brake, self.steering)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def toggle_dbw(self, msg):
        self.enabled = msg.data

    def read_velocity(self, msg):
        self.current_linear = msg.twist.linear.x

    def read_steer(self, msg):
        self.target_linear = msg.twist.linear.x
        self.target_angular = msg.twist.angular.z

if __name__ == '__main__':
    try:
      DBWNode()
    except rospy.ROSInterruptException:
      rospy.logerr("Could not start the DBW node.")
