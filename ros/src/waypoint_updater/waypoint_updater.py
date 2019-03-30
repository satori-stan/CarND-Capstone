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


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # The last pose value received
        self.pose = None
        # The waypoints from base_waypoints
        self.known_waypoints = None
        # An extract of the base waypoints that has only X and Y coordinates
        self.waypoints_2d = None
        # A KDTree of the 2D waypoints to easily find the ones nearer to us
        self.waypoint_tree = None

        # Improved control over how often we are running
        self.loop()

    def loop(self):
        # Set our target frequency (Hz).
        rate = rospy.Rate(50)
        # Run only if we are meant to and have all required data
        while not rospy.is_shutdown():
            if self.pose and self.waypoint_tree:
                closest_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_idx)
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

    def publish_waypoints(self, starting_index):
        out = Lane()
        out.header = self.known_waypoints.header
        #out.header.stamp = rospy.Time.now()
        out.waypoints = self.known_waypoints.waypoints[starting_index : starting_index + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(out)

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
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
