#! /usr/bin/env python

import roslib; roslib.load_manifest('capra_ai')
import rospy
from sensor_msgs.msg import LaserScan
import time
import math
from geometry_msgs.msg import *
import numpy as np
import cv2

max_dist = 3.0
map_resolution = 0.01
robot_radius = 0.7
scan_received = False
scan = LaserScan()
displacement_check = 0.025
cost_threshold = 3

DEFAULT_PRIORITY_ANGLES = [0, -10, -20, 10, 20, -30, -40, 30, 40, -50, 0, -60, 50, -70, 60, 70, -80, 80, -90, 90]

def find_dist(dist, angle):
    x = dist * math.cos(angle)
    y = dist * math.sin(angle)

    return x, y


def rf_callback(msg):
    global scan, scan_received
    scan = msg
    scan_received = True


def create_circle(radius, center):
    circle = []
    for x in range(-radius, 1):
        for y in range(-radius, 1):
            if x * x + y * y < radius * radius:
                circle.append((x + center[0], y + center[1]))
                circle.append((-x + center[0], y + center[1]))
                circle.append((x + center[0], -y + center[1]))
                circle.append((-x + center[0], -y + center[1]))
    return circle

def calculate_sum(mat, circle, center):
    sum = 0
    for p in circle:
        x = p[0] + center[0]
        y = p[1] + center[1]
        sum += mat[x][y]
    return sum

def show_matrix(mat, name):
    m = np.flipud(mat)
    m = np.fliplr(m)
    im = np.asarray(m)
    cv2.imshow(name, im)
    cv2.waitKey(10)

rospy.init_node('ai_demo_cones')
cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=4)
rospy.Subscriber('/scan', LaserScan, rf_callback)

rate = rospy.Rate(10)

size = int(math.ceil(2 * (max_dist / map_resolution)))
robot_area = create_circle(int(robot_radius/map_resolution), (int(size/2.0), int(size/2.0)))
cost = 0

while not rospy.is_shutdown():
    if scan_received:
        m = np.zeros((size, size))

        angle = scan.angle_min
        for p in scan.ranges:
            if p < 3.0:
                px, py = find_dist(p, angle)
                x = int((px + max_dist) / map_resolution)
                y = int((py + max_dist) / map_resolution)
                m[x][y] = 1

            angle += scan.angle_increment

        for angle in DEFAULT_PRIORITY_ANGLES:
            angle = math.radians(angle)
            disp_x = int((displacement_check * math.cos(angle))/map_resolution)
            disp_y = int((displacement_check * math.sin(angle))/map_resolution)
            cost = calculate_sum(m, robot_area, (disp_x, disp_y))
            if cost < cost_threshold:
                break

        m2 = m.copy()
        for p in robot_area:
            m2[p[0] + disp_x][p[1]+disp_y] = 1

        show_matrix(m2, "aaa")

        vel = Twist()
        print angle, cost
        if cost < cost_threshold:
             vel.linear.x = 0.2
             vel.angular.z = 0.1
             if angle < 0:
                 vel.angular.z = -vel.angular.z

             print "Linear: ", vel.linear.x
        else:
            vel.angular.z = 0.2 * abs(angle) / angle
            vel.linear.x = 0
            print "Angular: ", vel.angular.z

        cmd_vel.publish(vel)

    rate.sleep()    