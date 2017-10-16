#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import tf
import copy

import math

LOOKAHEAD_WPS = 100 # We choose 100 waypoints to look in front of the car

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

	# Subscribe to necessary topics (Car position and velocity, list of waypoints, traffic light)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size = 1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.twist_cb, queue_size=1);
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

	# Publisher for the final waypoint list
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Container for current pose received via subscribtion to '/current_pose'
        self.pose = None

        # Container for current base waypoints received via subscribtion to '/base_waypoints' & one copy
        self.base_waypoints = None
        self.base_waypoints_copy = None

	# Container for the last waypoint ID
        self.last_waypoint_id = None

	# Container for waypoints ahead of car
        self.wps_ahead = None

	# Help variable, if traffic light is red & in front of the car #FIXME probably don't needed, because we could use tf_state
        self.traffic_light_helper = 0

	# Help variable for the stopping function
        self.counter = 0

	# State of driving, driving = 0, stopping = 1
        self.drive = 0

	# State for the traffic_light 
        self.tf_state = -1

	# State for the previous traffic_light
        self.prev_tf_state = -1

        rospy.spin()

    def publish(self):
        # Define container for waypoints ahead
        wps_ahead = Lane()

        # Determine length of input waypoints
        waypoints_len = len(self.base_waypoints)

        # Determine first waypoint ahead of the car
	# Searching for closest waypoint (Just ID)
        idx_wp_closest = self.get_idx_closest_waypoint()
	# Seeing if closest waypoint is ahead, otherwise choose the next (Just ID)
        idx_wp_ahead = self.get_idx_ahead_waypoint(idx_wp_closest)
	# Setting last_waypoint_id now
        self.last_waypoint_id = idx_wp_ahead

        # Determine waypoints ahead to be published
        idx_cur = idx_wp_ahead

	# Going through the whole list of traffic points we want to publish
        for i in range(LOOKAHEAD_WPS):
	    # Setting the pose and twist data for the waypoints in the list
            wp = self.base_waypoints[idx_cur]
            next_wp = Waypoint()
            next_wp.pose = wp.pose
            next_wp.twist = wp.twist
	    # Resetting our speed for the waypoints, if the car is getting from stopping to driving again
            if (self.traffic_light_helper > 0) & (self.drive > 0):
                next_wp.twist.twist.linear.x = self.get_waypoint_velocity(self.base_waypoints_copy[idx_cur])
	    # Append the waypoint to wps_ahead
            wps_ahead.waypoints.append(next_wp)

	    # For the wrap-around last waypoint to first waypoint
            idx_cur = (idx_cur + 1) % waypoints_len
        
	# Setting drive-parameter when we want to driving again
        if (self.traffic_light_helper > 0) & (self.drive > 0):
            self.drive = 0

	# Stopping if a traffic_light is red in front of us
        if self.tf_state > -1:
            self.stopping(self.tf_state, idx_wp_ahead, wps_ahead)
    
	# Setting wps_ahead to self.wps_ahead
        self.wps_ahead = wps_ahead
	
	# Publish our waypoints!
        self.final_waypoints_pub.publish(wps_ahead)

    # Getting our car position with callback-function
    def pose_cb(self, msg):
        self.pose = msg.pose

    # Getting our car velocity with callback-function
    def twist_cb(self, msg):
        self.twist = msg.twist

    # Getting our whole waypoint list & a copy with callback-function
    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints.waypoints
        self.base_waypoints_copy = copy.deepcopy(self.base_waypoints)

    # Callback-function for the traffic_lights / Here we publish our waypoints!
    def traffic_cb(self, msg):
        self.prev_tf_state = self.tf_state
        self.tf_state = msg.data - 20 #TODO: Fix this in tl_detector
       
        #Check if the signal changed from red to non-red(green, yellow, no traffic light).
        #if ((self.prev_tf_state > -1) and (self.tf_state == -1)):
        if ((self.prev_tf_state > -1) and (self.tf_state == -21)): #TODO: Fix this hack
            self.traffic_light_helper = 1
        else:
            self.traffic_light_helper = 0
        
	# Publish waypoint, if we have a list & a position
        if (self.base_waypoints and self.pose):
            self.publish()

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    # Function to get waypoint_velocity
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    # Function to set waypoint_velocity
    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    # Function to get distance between two waypoints
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


    # Function to get closest waypoint to car position
    def get_idx_closest_waypoint(self):
        # If condition validates whether there are base_waypoints and the position of the car available
        if (self.base_waypoints and self.pose):
            min_wp_dist = 1000000
            idx_wp_closest = None

	    # Go through the whole list and search for the waypoint nearest to car_position with a dist-function (probably redundant, we should use the distance function above)
            if self.last_waypoint_id == None:
                for i in range(len(self.base_waypoints)):
                    dist = self.get_eucl_distance(self.base_waypoints[i].pose.pose.position.x, self.base_waypoints[i].pose.pose.position.y,self.pose.position.x,self.pose.position.y)
                    if dist < min_wp_dist:
                        min_wp_dist = dist
                        idx_wp_closest = i

	    # It is not necessary to go through the whole waypoint-list all the time. Here we use the last waypoint to search only in a range of 50 waypoints.
            else:
                for i in range(self.last_waypoint_id, self.last_waypoint_id+50):
                    j = i-3
                    if j < 0:
                        j = j + len(self.base_waypoints)
                    j = j % len(self.base_waypoints)
                    dist = self.get_eucl_distance(self.base_waypoints[j].pose.pose.position.x, self.base_waypoints[j].pose.pose.position.y,self.pose.position.x,self.pose.position.y)
                    if dist < min_wp_dist:
                        min_wp_dist = dist
                        idx_wp_closest = j

            return idx_wp_closest

        else:
            return None


    # Function to see if the closest waypoint is ahead, else choose the next one!
    def get_idx_ahead_waypoint(self, idx_wp_closest):
        wp_x_local,_,_ = self.transform_wp_to_local(self.base_waypoints[idx_wp_closest])
        if wp_x_local > 0.0:
            return idx_wp_closest
        else:
            return (idx_wp_closest + 1) % len(self.base_waypoints)
        
    # Function to transform our waypoint to local coordinates
    def transform_wp_to_local(self, wp):
        wx = wp.pose.pose.position.x
        wy = wp.pose.pose.position.y
        _,_,yaw = tf.transformations.euler_from_quaternion([self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w])

        dx = wx - self.pose.position.x
        dy = wy - self.pose.position.y
        wx_local = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        wy_local = math.sin(-yaw) * dx - math.cos(-yaw) * dy
        return wx_local, wy_local, math.atan2(wy_local, wx_local)


    # Distance function (like said above, maybe redundant)
    def get_eucl_distance(self,x1,y1,x2,y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    # Stopping the car, if red light ahead
    def stopping(self, idx, idx_wp_ahead, wps_ahead):
	    # When car was normally driving before (drive = 0) setting some parameters. Only necessary in the beginning of stopping!
            if (self.drive == 0):
                self.drive = 1
		# A bit convoluted - Makes sure that we stop right at the point we want to stop & not a bit later...
                k = 0
                self.counter = 0
                while k < 6.5*(self.twist.linear.x/11.112):
                    k = self.distance(self.base_waypoints, idx-self.counter, idx)
                    self.counter = self.counter + 1
                    if self.counter > LOOKAHEAD_WPS/4:
                        break
                      
            # Calculate how many waypoints lay between the wapoint ahead and the waypoint, where we want to stop
            wp_until_tl = idx - idx_wp_ahead

            # This is for the wrap-around (first waypoints & the last waypoints)
            if wp_until_tl < (-1*len(self.base_waypoints)/2):
                wp_until_tl = wp_until_tl + len(self.base_waypoints)

	    # Making sure to stop at the right points, probably a better solution possibly
            wp_until_tl = wp_until_tl - self.counter

	    # If we are already behind our stopping point, making sure to stop now for real!
            if wp_until_tl < 0:
                wp_until_tl = 0

	    # Divide our Lookahead in two parts. The part with the waypoint idx and all following get set to 0!
            if wp_until_tl <= LOOKAHEAD_WPS:
                parting_wp = wps_ahead.waypoints[wp_until_tl]

            for i in range(wp_until_tl, LOOKAHEAD_WPS):
                self.set_waypoint_velocity(wps_ahead.waypoints, i, 0.)

	    # Stopping the vehicle in the waypoints leading up to idx
            for i in range(wp_until_tl-1, -1, -1):
                wp = wps_ahead.waypoints[i]
                distance = self.distance(wps_ahead.waypoints, i, i + 1)
                velocity_previous = self.get_waypoint_velocity(parting_wp)
                velocity_current = math.sqrt(0.2*distance) + velocity_previous
                if velocity_current <= 0.25:
                    velocity_current = 0
                velocity_last = self.get_waypoint_velocity(wp)
                if velocity_current > velocity_last:
                    break
                self.set_waypoint_velocity(wps_ahead.waypoints, i, velocity_current)
                parting_wp = wp

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
