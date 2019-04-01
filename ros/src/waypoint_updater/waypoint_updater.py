#!/usr/bin/env python

import math

import numpy as np
from scipy.spatial import KDTree

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Complete initialization only after the publishers we depend
		# on are available.
		#rospy.wait_for_message('/current_pose', PoseStamped)
		#rospy.wait_for_message('/base_waypoints', Lane)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoints', Lane, self.traffic_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # The last pose value received
        self.pose = None
        # The waypoints from base_waypoints
        self.known_waypoints = None
        # An extract of the base waypoints that has only X and Y coordinates
        self.waypoints_2d = None
        # A KDTree of the 2D waypoints to easily find the ones nearer to us
        self.waypoint_tree = None

        self.stopline_wp_idx = -1

        # Improved control over how often we are running
        self.loop()

    def loop(self):
        # Set our target frequency (Hz).
        rate = rospy.Rate(50)
        # Run only if we are meant to and have all required data
        while not rospy.is_shutdown():
            if self.pose and self.waypoint_tree:
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        pt = [self.pose.pose.position.x, self.pose.pose.position.y]
        # Use our KDTree to get the closest waypoin, but return only the index
        closest, closest_idx = self.waypoint_tree.query(pt, 1)

        # Now we figure out if the closest waypoint is in front of the car
        previous = self.waypoints_2d[closest_idx - 1]
        c = np.array(closest)
        p = np.array(previous)
        me = np.array(pt)

        # Distance to the hyperplane in the direction of movement
        distance = np.dot(c - p, me - c)

        if distance > 0:  # The closest waypoint is behind us!
            # Get the next index
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoints(self):
        out = Lane()
        out.header = self.known_waypoints.header
        out.waypoints = self.calculate_target_waypoints()
        self.final_waypoints_pub.publish(out)

    def calculate_target_waypoints(self):
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS

        waypoints = self.known_waypoints.waypoints[closest_idx : farthest_idx]
        if self.stopline_wp_idx == -1 or self.stopline_wp_idx >= farthest_idx:
            return waypoints
        else:
            return self.calculate_deceleration(waypoints, closest_idx, self.stopline_wp_idx)

    def calculate_deceleration(self, waypoints, current_index, stop_index):
        """
        Create a new list of waypoints to follow that slowly decreases speed to a stop

        Args:
            waypoints -- The original array of waypoints being followed
            current_index -- The index in that list of waypoints where we are
            stop_index -- The index in that list of waypoints where we should stop

        Returns:
            An array of waypoints with the twist commands updated to stop
        """
        decel_waypoints = []
        # Make sure we are stopping *before* the obstacle
        stop_index_ = max(current_index - stop_index - 2, 0)
        starting_velocity = None
        last_velocity = None
        stop_distance = None
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            dist = self.distance(waypoints, i, stop_index_)
            if not stop_distance:
                stop_distance = dist

            if not starting_velocity:
                velocity = starting_velocity = wp.twist.twist.linear.x
            else:
                # Slow down with a sigmoid shape
                # TODO: Improve so that we don't stop too early
                velocity = max(starting_velocity / (1. + math.exp(((stop_distance / 2.) - dist) / 5.)), last_velocity - MAX_DECEL)

            # Make sure we don't keep creeping forward, but don't go over the max decel
            if velocity < (MAX_DECEL - 0.1):
                velocity = 0.
            
            last_velocity = velocity

            # Apply modified velocity, but don't override the existing one if
            # it is less than our target.
            p.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x)
            decel_waypoints.append(p)

    def pose_cb(self, msg):
        # Every time we get a new pose, check the list of known waypoints to
        # find the one closest to us in the direction of movement and then
        # get the next "LOOKAHEAD_WPS" into final_waypoints.
        #dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        #distances = [dl(msg.position, waypoint.position) for waypoint in self.known_waypoints]
        #closest_index = distances.index(min(distances))
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # At this stage, take only the array of waypoint poses
        self.known_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, \
                waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg
        rospy.logwarn(self.stopline_wp_idx)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
