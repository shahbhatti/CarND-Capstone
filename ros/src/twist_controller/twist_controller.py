import rospy
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858

class Controller(object):
    def __init__(self, **kwargs):
        self.vehicle_mass = kwargs["vehicle_mass"]
        self.fuel_capacity = kwargs["fuel_capacity"]
        self.brake_deadband = kwargs["brake_deadband"]
        self.decel_limit = kwargs["decel_limit"]
        self.accel_limit = kwargs["accel_limit"]
        self.wheel_radius = kwargs["wheel_radius"]

	# Setting our mass
        self.mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY

	# Our Twist_controller consists of a PID-Controller for the velocity and a Yaw-Controller for the steering
	# We choose the values for the PID-Controller with a lot of testing, trial and error
        self.lin_vel_pid = PID(1.7, 0.001, 0.02, mn = self.decel_limit, mx = self.accel_limit)
	self.yawcontroller = YawController(2.8498, 14.8, 5.0, 3.0, 8.)


    def control(self, **kwargs):
        
	# linear velocity, angular velocity for current & future
        linear_velocity_future = kwargs["linear_velocity_future"]
        angular_velocity_future = kwargs["angular_velocity_future"]
        linear_velocity_current = kwargs["linear_velocity_current"]
        angular_velocity_future = kwargs["angular_velocity_future"]
	time_step = kwargs["time_step"]

	# Calculate the difference between the future velocity and the current velocity (linear) for the PID-Controller
        error_velocity = linear_velocity_future - linear_velocity_current

	# Setting the acceleration with the PID-Controller (using step function with error & time)
        acceleration = self.lin_vel_pid.step(error_velocity,time_step)

	# Setting the steering with the Yaw-Controller
	steer = self.yawcontroller.get_steering(linear_velocity_future, angular_velocity_future, linear_velocity_current)

	# If acceleration is negative setting throttle to zero, else throttle is acceleration
        if(acceleration < 0):
            #self.lin_vel_pid.reset()
            throttle = 0
        else:
            throttle = acceleration

	# Setting the brake
        if(acceleration < -abs(self.brake_deadband)):
            brake = acceleration * self.mass * self.wheel_radius
        else:
            brake = 0
            
	#If linear velocity is 0 we set the brake to 100% and throttle to 0 directly!
	if linear_velocity_future == 0:
	    acceleration = 0
	    brake = self.mass * self.wheel_radius

        
        # TODO: Change the arg, kwarg list to suit your needs
        return throttle, brake, steer


    def reset(self):
        self.lin_vel_pid.reset()
