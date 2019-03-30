import rospy

from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    __forbidden = ("control")
    def __init__(self, *args, **kwargs):
        for k, v in kwargs.iteritems():
            assert (k not in self.__class__.__forbidden)
            setattr(self, k, v)

        # Acceleration when the car is idle and no break is applied. This comes
        # from the car having an automatic transmission.
        idle_accel = 1.67  # XXX: From parameter?
        #self.idle_break = self.vehicle_mass * idle_accel * self.wheel_radius
        self.idle_break = self._calculate_brake(idle_accel)

        # Initialize our PID controller for the throttle. The minimum value
        # is clear, the maximum is heuristically set to ensure smooth
        # acceleration. The p, i and d constants have been tuned with a bit
        # of twiddling.
        self.throttle_controller = PID(0.7459, 0.000289, 12.1355, mn=0., mx=0.2)

        # Initialize the controller for the steering. We are using here the
        # provided controller, which is mostly initialized with parameters from
        # the vehicle. The only parameter specifically set is the minimum speed
        # (with a value of 0.1) representing the min speed used to steer. This
        # prevents a division by zero error.
        self.steer_controller = YawController(self.wheel_base, self.steer_ratio,
                0.1, self.max_lat_accel, self.max_steer_angle)

        # A low pass filter for vehicle velocity readings (since they are noisy)
        # Use a sampling time matching the reading frequency and a simple
        # cutoff frequency of 1/PI.
        self.velocity_filter = LowPassFilter(0.5, 0.02)

        self.last_time = rospy.get_time()
        
        # Values used to control excessive steering
        self.reasonable_steering_range = self.max_steer_angle / 2.
        self.last_angular_velocity = 0.


    def _calculate_brake(self, accel):
        # TODO: Account for fuel, passenger and cargo weight! Maybe even slope
        #       and movement direction!
        return self.vehicle_mass * self.wheel_radius * accel

    def control(self, current_linear_velocity, target_linear_velocity,
                target_angular_velocity, is_enabled, **kwargs):

        # Stop if we are not controlling the vehicle. Return 0 to apply no changes
        # to the current throttle, break or steer settings.
        if not is_enabled:
            self.throttle_controller.reset()
            self.last_steering = 0.
            return 0., 0., 0.

        # If we are controlling the vehicle...
        # Apply the filter to the velocity reading
        current_velocity = self.velocity_filter.filt(current_linear_velocity)

        # Get steering, which is unencumbered by any additional logic
        steering = self.steer_controller.get_steering(target_linear_velocity,
                target_angular_velocity, current_velocity)

        # TODO: Use constants
        if current_linear_velocity > 5.55: # Over 20km/h
            steering *= 1.1 - (min(current_linear_velocity, 19.44) / 19.44) # 70km/h

        # TODO: If the angle of attack was "big", adjust

        # That being said... prevent steering from being too sudden. We find
        # the steering delta between two calculations and divide it by a
        # reasonable range to get a sense of how aggressively we are steering. A
        # change in steering over one time the reasonable range will cause the
        # factor to be used to limit the final steering delta. The factor
        # 1.61804 used was chosen for no special reason.
        angular_velocity_delta = abs(target_angular_velocity - self.last_angular_velocity)
        rospy.logwarn("cl {0:f} tl {1:f} ta {2:f}, ad: {3:f}".format(current_velocity, target_linear_velocity, target_angular_velocity, angular_velocity_delta))
        #if angular_velocity_delta > 0.01:
        #  steering /= (1.61804 * current_velocity)
        self.last_angular_velocity = target_angular_velocity

        # To calculate the target throttle, we need the error between the target
        # and the current velocity values, and the time elapsed.
        velocity_error = target_linear_velocity - current_velocity
        timestamp = rospy.get_time()
        time_delta = timestamp - self.last_time
        self.last_time = timestamp

        throttle = 0.1 #self.throttle_controller.step(velocity_error, time_delta)

        brake = 0.
        # Apply the breaks when we are supposed to be still
        if target_linear_velocity == 0. and current_velocity < 0.1:
            throttle = 0.
            brake = self.idle_brake

        # Apply regular break when not accelerating is not getting us closer
        # to our target velocity.
        elif throttle < 0.1 and velocity_error < 0:
            throttle = 0.
            assert(self.decel_limit < 0)
            required_deceleration = max(velocity_error, self.decel_limit)
            brake = self._calculate_brake(-required_deceleration)

        rospy.logwarn("T: {0:f}\tB: {1:f}\t S: {2:f}".format(throttle, brake, steering))
        return throttle, brake, steering
