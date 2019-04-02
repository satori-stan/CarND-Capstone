#!/usr/bin/env python

import cv2
import tf
import numpy as np
import rospy
import yaml
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from scipy.spatial import KDTree

from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        # The full list of waypoints
        self.waypoints = None
        # A 2D extract of the waypoints (for easier processing)
        self.waypoints_2d = None
        # A KDTree of waypoints to easily find the one closest to us
        self.waypoint_tree = None
        self.camera_image = None
        # The list of traffic lights in the route
        self.lights = []
        # An extract of the 2D position of the trafficlights
        self.lights_2d = None
        # A KDTree to easily find the traffic light closest to us
        self.lights_tree = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        # The number of camera frames to skip, for perfomance
        self.skip_frames = 2
        # A counter to enforce the skips
        self.skip_counter = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        self.waypoints_2d = [[waypoint.pose.pose.position.x,
                waypoint.pose.pose.position.y] for waypoint in self.waypoints.waypoints]
        self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights
        self.lights_2d = [[light.pose.pose.position.x,
                light.pose.pose.position.y] for light in self.lights]
        self.lights_tree = KDTree(self.lights_2d)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
            rospy.logwarn("New state: {0}".format(state))
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def _find_closest(self, x, y, tree, full_array):
        pt = [x, y]
        closest, closest_idx = tree.query(pt, 1)
        previous = full_array[closest_idx - 1]
        c = np.array(closest)
        p = np.array(previous)
        me = np.array(pt)
        distance = np.dot(c - p, me - c)
        if distance > 0: # The closest X is behind us!
            closest_idx = (closest_idx + 1) % len(full_array)

        return closest_idx

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        return self._find_closest(x, y, self.waypoint_tree, self.waypoints_2d)

    def _store_for_model(self, cv_image):
        """Takes the image and its label and stores them in a TensorFlow record

        Args:
            cv_image (cv::Mat): The image
            label (uint8): The label as mapped in TrafficLight

        """
        
        pass

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Return if we don't have an image
        if (not self.has_image):
            self.prev_light_loc = None
            return False

        # Return if we haven't skipped enough frames
        if self.skip_counter < self.skip_frames:
            self.skip_counter += 1
            return self.state

        # If we have an image and have skipped enough frames, reset the counter
        self.skip_counter = 0

        # Read the image
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Get classification
        guessed_state = self.light_classifier.get_classification(cv_image)
        #rospy.logwarn(guessed_state)

        # For training and such, come up with a filename to save this with
        #if guessed_state != light.state:
        #timestamp = rospy.Time.now()
        #filename = "/tmp/data/tl_classification/{2}/{0}_{1:07d}.jpg".format(timestamp.secs, timestamp.nsecs, light.state)
        #cv2.imwrite(filename, cv_image)

        return guessed_state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not (self.waypoint_tree and self.lights_tree):
          return -1, TrafficLight.UNKNOWN

        light = None
        light_idx = None
        stop_line_wp_idx = -1

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if (self.pose):
            x = self.pose.pose.position.x
            y = self.pose.pose.position.y
            car_position = self.get_closest_waypoint(x, y)
            # Find the closest visible traffic light (if one exists)
            # The index of the stop line will be the index of the light
            light_idx = self._find_closest(x, y, self.lights_tree, self.lights_2d)
            light = self.lights[light_idx]

        if light:
            state = self.get_light_state(light)
            if state == TrafficLight.RED:
              stop_line_wp_idx = self.get_closest_waypoint(stop_line_positions[light_idx][0], stop_line_positions[light_idx][1])
            return stop_line_wp_idx, state

        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
