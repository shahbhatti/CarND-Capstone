#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped

from twist_controller import Controller

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        rospy.loginfo('Start initialization of DBWNode')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)


	# Setting up all needed self.variables
        self.linear_velocity_future = None
        self.angular_velocity_future = None
        self.linear_velocity_current = None
        self.angular_velocity_current = None
        self.dbw_enabled = None

	# Necessary to calculate the time difference between this controller step & the future one
	self.time_last = None

	# Rate in which we publish throttle, brake and steer
        self.rate = 50

	# Publisher for steer, throttle, brake
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Our Twist-Controller
        self.controller = Controller(
            vehicle_mass = vehicle_mass,
            fuel_capacity = fuel_capacity,
            brake_deadband = brake_deadband,
            decel_limit = decel_limit,
            accel_limit = accel_limit,
            wheel_radius = wheel_radius
            )

        # Subscribe to all needed topics (twist, velocity, dbw_enabled)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twistcmd_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.cur_vel_cb, queue_size=1)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)


        rospy.loginfo('DBWNode: Subscribed to relevant topics')

        self.loop()

    def loop(self):
        rospy.loginfo('DBWNode: Started looping')

	# Using the rate to publish with 50 Hertz
        rate = rospy.Rate(self.rate)

	
        while not rospy.is_shutdown():

	    # Get the time
	    time_now = rospy.get_rostime()

	    if self.has_valid_data():
		# Get the time difference between steps
		time_diff = time_now - self.time_last

		# Getting throttle, brake and steer with our Controller
		throttle, brake, steer = self.controller.control(
               	 linear_velocity_future = self.linear_velocity_future, 
              	 angular_velocity_future = self.angular_velocity_future, 
                 linear_velocity_current = self.linear_velocity_current, 
                 angular_velocity_current = self.angular_velocity_current,
		 time_step = time_diff.to_sec())

		# Publish throttle, brake and steer here
            	self.publish(throttle, brake, steer)
	    # Setting time_last to our time_now-value for the next step
	    self.time_last = time_now
		
	    # Sleep with the help of rate
            rate.sleep()

    # The publish-function for throttle, brake & steer
    def publish(self, throttle, brake, steer):
	# Make sure not so brake and throttle at the same time
        if brake > 0.0:
            bcmd = BrakeCmd()
            bcmd.enable = True
            bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
            bcmd.pedal_cmd = brake
            self.brake_pub.publish(bcmd)
        else:
            tcmd = ThrottleCmd()
            tcmd.enable = True
            tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
            tcmd.pedal_cmd = throttle
            self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)


    # Twist callback-function
    def twistcmd_cb(self, msg):
        self.linear_velocity_future = msg.twist.linear.x
        self.angular_velocity_future = msg.twist.angular.z


    # Current velocity callback-function
    def cur_vel_cb(self, msg):
        self.linear_velocity_current = msg.twist.linear.x
        self.angular_velocity_current = msg.twist.angular.z


    # Looking up if dbw is enabled with callback-function
    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data

	# Resetting Controller when dbw is not enabled
        if (not self.dbw_enabled):
            self.controller.reset()

    # Looking up if we have valid data to work with
    def has_valid_data(self):
        return (self.time_last is not None) & (self.linear_velocity_future is not None) & (self.linear_velocity_current is not None)
        
        
if __name__ == '__main__':
    DBWNode()
