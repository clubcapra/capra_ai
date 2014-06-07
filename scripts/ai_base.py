#! /usr/bin/env python

import roslib; roslib.load_manifest('capra_ai')
import rospy
import actionlib
import csv
from sensors.Gps import Gps
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from capra_msgs.msg import AiStatus, EStopStatus
from nav_msgs.msg import Odometry

class AIBase(object):
    running = True
    
    def __init__(self, node_name, gps_goals_file=None):
        rospy.init_node(node_name)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._load_waypoints(gps_goals_file)
        
    def _load_waypoints(self, file):
        self.waypoints = []
        if file:
            f = open(file, 'rt')
            try:
                reader = csv.reader(f)
                for waypoint in reader:
                    self.waypoints.append(waypoint)
            finally:
                f.close()

    def run(self):
        self.client.wait_for_server()
        self._create_status_broadcaster()
        rospy.Subscriber("/capra_smartmotor/estop", EStopStatus, self._estop_subscriber)
        rospy.Subscriber("/enu", Odometry, self._gps_subscriber)

    def _create_status_broadcaster(self):
        self.status_publisher = rospy.Publisher('~status', AiStatus)
        rospy.Timer(rospy.Duration(0.2), self._status_broadcaster)

    def _status_broadcaster(self, event):
        status = AiStatus()
        status.isRunning = self.running
        self.status_publisher.publish(status)

    def _estop_subscriber(self, data):
        self.estop_listener(data.stopped)

    def _gps_subscriber(self, data):
        self.gps_listener(data.pose.pose.position.x, data.pose.pose.position.y)

    def send_goal(self, goal, wait=False):
        self.client.send_goal(goal)
        if wait:
            self.client.wait_for_result()

    def get_state(self):
        return self.client.get_state()

    def estop_listener(self, status):
        self.running = not status

    def gps_listener(self, x, y):
        self.Gps = Gps(x, y)
                