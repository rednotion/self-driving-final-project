import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


"""
You can use this class to implement vehicle control. For example, the control method can take twist data as input and return throttle, brake, and steering values. Within this class, you can import and use the provided pid.py and lowpass.py if needed for acceleration, and yaw_controller.py for steering. Note that it is not required for you to use these, and you are free to write and import other controllers.
"""


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
                 wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        # yaw
        self.yaw_controller = YawController(wheel_base, steer_ratio, ONE_MPH,
                                            max_lat_accel, max_steer_angle)

        # pid/throttle
        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0  # min throttle value
        mx = 0.2  # max throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # low pass
        # helps to filter out the high frequnecy noise in the velocity readings
        tau = 0.5  # 1/(2pi*tau) = cutoff frequency
        ts = 0.02  # sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        # other properties
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)

        # rospy.logwarn("Angular vel: {0}".format(angular_vel))

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if linear_vel == 0 and current_vel < 0.1:
            # if target lin vel is 0 and current vel is also very slow, we are trying to stop
            throttle = 0
            brake = 700
        elif throttle < 0.1 and vel_error < 0:
            # if vel_error < 0 then we are currently going faster than we want to be
            # thus we need to slow down. shut throttle to 0 and implement brakes
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius  # torque N*m

        return throttle, brake, steering
