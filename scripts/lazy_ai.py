#! /usr/bin/env python

import roslib; roslib.load_manifest('capra_ai')
import rospy
from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import *
import numpy as np
import cv2

rospy.init_node('ai_demo_cones')


#La taille de la carte generee (en m). Le robot est au milieu.
map_size = rospy.get_param("~map_size", 6.0)

#La resolution de la carte generee
map_resolution = rospy.get_param("~map_resolution", 0.01)

#Le rayon du cercle de verification (pour savoir si le robot passe)
robot_radius = rospy.get_param("~robot_radius", 0.7)

#La distance par rapport au range finder ou le cercle est deplace
displacement_check = rospy.get_param("~displacement_check", 0.6)

#La vitesse du robot
speed = rospy.get_param("~speed", 0.5)

#Les angles ou le robot peut aller
priority_angles = [int(v.strip()) for v in rospy.get_param("~priority_angles", "0, 10, 20, 30, 40, 50, 60").split(",")]

scan = LaserScan()
scan_received = False

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

cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=4)
rospy.Subscriber('/scan', LaserScan, rf_callback)
rate = rospy.Rate(10)

size = int(math.ceil(map_size / map_resolution))
max_dist = map_size / 2.0
robot_area = create_circle(int(robot_radius/map_resolution), (int(size/2.0), int(size/2.0)))
cost = 0

while not rospy.is_shutdown():
    if scan_received:
        m = np.zeros((size, size))

        angle = scan.angle_min
        for p in scan.ranges:
            if p < 3.0:
                if not ( p < 1.0 and abs(angle) > math.pi / 2):
                    if p < 1 and p > 0.01:
                        pass
                    px, py = find_dist(p, angle)
                    x = int((px + max_dist) / map_resolution)
                    y = int((py + max_dist) / map_resolution)
                    m[x][y] = 1

            angle += scan.angle_increment

        for angle in priority_angles:
            angle = math.radians(angle)
            disp_x = int((displacement_check * math.cos(angle))/map_resolution)
            disp_y = int((displacement_check * math.sin(angle))/map_resolution)
            cost = calculate_sum(m, robot_area, (disp_x, disp_y))
            if cost == 0:
                break

        #m2 = m.copy()
        #for p in robot_area:
        #    m2[p[0] + disp_x][p[1]+disp_y] = 1
        #
        #m2[size/2][size/2]=0; m2[size/2 + 1][size/2]=0; m2[size/2 - 1][size/2]=0; m2[size/2][size/2 + 1]=0; m2[size/2][size/2 - 1]=0;
        #show_matrix(m2, "aaa")

        vel = Twist()
        if cost == 0:
             vel.linear.x = speed * math.cos(angle)
             vel.angular.z = speed * math.sin(angle)
        else:
            vel.linear.x = 0
            vel.angular.z = speed

        cmd_vel.publish(vel)

    rate.sleep()    