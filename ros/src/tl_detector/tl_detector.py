#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
from traffic_light_config import config
import tl_utils
import threading

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights helps you acquire an accurate ground truth data source for the traffic light
        classifier, providing the location and current color state of all traffic lights in the
        simulator. This state can be used to generate classified images or subbed into your solution to
        help you work on another single component of the node. This topic won't be available when
        testing your solution in real life so don't rely on it in the final submission.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/camera/image_raw', Image, self.image_cb)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
       
        #  Flag to indicate if Tensorflow has loaded.
        self.classifier_ready = False
 
        # Function to call when network has been loaded successfully.  
        def wait_for_loading_network():
            self.classifier_ready = True
            print("TL classifier loaded")
            t.cancel()

        print("Waiting for TL classifier to load (5 second timer)")
        t = threading.Timer(5.0, wait_for_loading_network)
        t.daemon = True
        t.start()

        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        '''
         Load the traffic lights configuration from the config structure
        '''
        self.tl_positions = tl_utils.convert_tl_config_to_lane_msgs()
        self.prev_nearest_tl_idx = None

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose

        '''
        args = [self.pose, self.waypoints, self.camera_image, self.tl_positions]
        all_args_available = all([arg is not None for arg in args])
        
        if all_args_available == True:
            idx, nearest_tl_ahead = tl_utils.find_nearest_tl_ahead(self.waypoints,
                                   self.pose.position, self.tl_positions.lights)
            rospy.loginfo('nearest tl:(%d, %f,%f)',
                idx,
                nearest_tl_ahead.pose.pose.position.x,
                nearest_tl_ahead.pose.pose.position.y)
        else: 
            rospy.loginfo('Some arguments missing!')
        '''

    def waypoints_cb(self, lane):
        self.waypoints = lane.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        
        args = [self.pose, self.waypoints, self.camera_image, self.tl_positions]
        all_args_available = all([arg is not None for arg in args])
        
        if all_args_available == True:
            '''idx, nearest_tl_ahead, _, tl_idx'''
            tl_idx, car_wp_idx, tl_car_wp_idx = tl_utils.find_nearest_tl_ahead(self.waypoints,
                                   self.pose.position, self.tl_positions.lights)
            '''
            rospy.loginfo('nearest tl:(%d, %f,%f)',
                idx,
                nearest_tl_ahead.pose.pose.position.x,
                nearest_tl_ahead.pose.pose.position.y)
            '''

            #rospy.loginfo('tl_car_wp_idx:%d', tl_car_wp_idx)
            if (tl_idx != self.prev_nearest_tl_idx):
                self.prev_nearest_tl_idx = tl_idx
                rospy.loginfo('nearest tl:(%d)', tl_idx)

            if (tl_car_wp_idx < 100):
                _, state = self.process_traffic_lights()
            else:
                state = self.last_state
            light_wp = car_wp_idx + tl_car_wp_idx


            '''
            Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            '''
            if self.state != state:
                self.state_count = 0
                self.state = state
            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1

        else: 
            rospy.loginfo('Some arguments missing!')
        

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        return 0


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = config.camera_info.focal_length_x
        fy = config.camera_info.focal_length_y

        image_width = config.camera_info.image_width
        image_height = config.camera_info.image_height

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image

        x = 0
        y = 0

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if (not self.has_image):
            rospy.logerr("Image not available to classify")
            return False

        state = TrafficLight.UNKNOWN   # Default state
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        if self.classifier_ready is True:
            state = self.light_classifier.get_classification(cv_image)
            #rospy.loginfo('State: %d', int(state)) 

        return -1, state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
