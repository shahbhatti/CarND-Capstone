"""
 Helper functions for traffic light detector.
"""

import numpy as np
import rospy
from styx_msgs.msg import TrafficLightArray, TrafficLight
import traffic_light_config

DEBUG = True

# Convert tl config values to Lane messages.
def convert_tl_config_to_lane_msgs():
    traffic_lights = TrafficLightArray()

    tl_list = []
 
    tl_height = rospy.get_param("/tl_height_sim")

    for traffic_light_index, traffic_light_position in enumerate(traffic_light_config.config['light_positions']):
        traffic_light = TrafficLight()

        traffic_light.pose.pose.position.x = traffic_light_position[0]
        traffic_light.pose.pose.position.y = traffic_light_position[1]
        traffic_light.pose.pose.position.z = tl_height
        traffic_light.state = TrafficLight.UNKNOWN
        tl_list.append(traffic_light)

    traffic_lights.lights = tl_list

    global DEBUG
    if DEBUG == True:
        for traffic_light_index, traffic_light_position in enumerate(tl_list):
            rospy.loginfo('x:%f,y:%f', traffic_light_position.pose.pose.position.x, traffic_light_position.pose.pose.position.y)

    return traffic_lights

# Get the road distance from the car, by calculating piecewise sum of waypoint distances.
def get_road_distance(waypoints):
    total_distance = 0.0

    for index in range(1, len(waypoints)):
        x_distance = waypoints[index].pose.pose.position.x - waypoints[index - 1].pose.pose.position.x
        y_distance = waypoints[index].pose.pose.position.y - waypoints[index - 1].pose.pose.position.y
        distance = np.sqrt((x_distance**2) + (y_distance**2))
        total_distance += distance

    return total_distance

# Get the closest waypoint to position value.
def get_closest_wp_index(position, wps_mat):
    x_dist = wps_mat[:, 0] - position.x
    y_dist = wps_mat[:, 1] - position.y

    squared_dist = x_dist ** 2 + y_dist ** 2
    return np.argmin(squared_dist)

# Convert waypoints to np matrices, to facilitate computation.
def get_wps_matrix(waypoints):
    wps_mat = np.zeros(shape=(len(waypoints), 2), dtype=np.float32)

    for index, waypoint in enumerate(waypoints):
        wps_mat[index, 0] = waypoint.pose.pose.position.x
        wps_mat[index, 1] = waypoint.pose.pose.position.y

    return wps_mat

# Find the nearest traffic light ahead of the car.
def find_nearest_tl_ahead(waypoints, car_pose, traffic_lights):
    wps_mat = get_wps_matrix(waypoints)
    car_index = get_closest_wp_index(car_pose, wps_mat)

    # Arrange track waypoints so they start at car position
    wps_ahead = waypoints[car_index:] + waypoints[:car_index]
    wps_ahead_mat = get_wps_matrix(wps_ahead)

    distances = []
    wp_indices = []

    #rospy.loginfo("----Distance----")
    for traffic_light in traffic_lights:
        waypoint_index = get_closest_wp_index(traffic_light.pose.pose.position, wps_ahead_mat)
        distance = get_road_distance(wps_ahead[:waypoint_index])
        #rospy.loginfo(distance)
        distances.append(distance)
        wp_indices.append(waypoint_index)

    closest_traffic_light_index = np.argmin(distances)

    return closest_traffic_light_index, car_index, wp_indices[closest_traffic_light_index]
  
