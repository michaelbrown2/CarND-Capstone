import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):
    def __init__(self,vehicle_mass,fuel_capacity,brake_deadband,decel_limit,accel_limit,wheel_radius,wheel_base,steer_ratio,max_lat_accel,max_steer_angle,min_speed):
        # TODO: Implement
        self.vehicle_mass=vehicle_mass
        self.fuel_capacity=fuel_capacity
        self.brake_deadband=brake_deadband
        self.decel_limit=decel_limit
        self.accel_limit=accel_limit
        self.wheel_radius=wheel_radius
        self.wheel_base=wheel_base
        self.steer_ratio=steer_ratio
        self.max_lat_accel=max_lat_accel
        self.max_steer_angle=max_steer_angle
        self.min_speed=min_speed
        
        self.yaw_controller = YawController(
            wheel_base=self.wheel_base,
            steer_ratio=self.steer_ratio,
            min_speed=self.min_speed,
            max_lat_accel=self.max_lat_accel,
            max_steer_angle=self.max_steer_angle)

        self.pid = PID(kp=0.6, ki=0.0, kd=0.0, mn=self.decel_limit, mx=self.accel_limit)
        self.steer_lpf = LowPassFilter(tau = 0.5, ts = 0.5)
        self.acc_lpf = LowPassFilter(tau = 0.5, ts = 0.5)

    def reset(self):
        self.pid.reset()

    def control(self, twist_velocity,twist_angualr_velocity, current_velocity, delta_time):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer


        steer = self.yaw_controller.get_steering(abs(twist_velocity), twist_angualr_velocity, current_velocity)
        steer = self.steer_lpf.filt(steer)
        
        velocity_error = abs(twist_velocity) - current_velocity
        acceleration = self.pid.step(velocity_error, delta_time)
        acceleration = self.acc_lpf.filt(acceleration)

        if acceleration > 0.0:
            throttle = acceleration
            brake = 0.0
        else:
            throttle = 0.0
            deceleration = -acceleration
            if deceleration < self.brake_deadband:
                deceleration = 0.0
            brake = deceleration * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius

        return throttle, brake, steer
