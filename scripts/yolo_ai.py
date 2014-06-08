#! /usr/bin/env python

import roslib; roslib.load_manifest('capra_ai')
import rospy
from sensor_msgs.msg import LaserScan
import time
import math
from geometry_msgs.msg import *
import numpy as np
from PIL import Image, ImageDraw
from shapely.geometry import Polygon
from shapely import affinity
import cv2
from multiprocessing import Pool, cpu_count, Value

theta = 0
map = None
map_size = 201
square_size = 100
max_dist = map_size / 2
angle_found = None

def rf_callback(data):
    global map
    global map_size
    global max_dist
    ranges = data.ranges
    m = np.zeros((map_size, map_size))
    o = -90.0
    for i in xrange(90, 451):
        if ranges[i] > 0:
            if ranges[i] < 3:
                if o == 0:
                    x = 0
                    y = ranges[i] * 100
                else:
                    s = o/abs(o)
                    angle = (90-abs(o)) * s * math.pi / 180
                    x = round(ranges[i] * math.cos(angle) * s * 100)
                    y = round(ranges[i] * math.sin(angle) * s * 100)

                if abs(x) < map_size/2 and y < map_size and y > 0:
                    m[map_size - y - 1][map_size - (x + map_size / 2) - 1] = 1
        o += 0.5
    map = m

def odom_callback(odom):
    #global x
    #global y
    #global theta
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    theta = odom.pose.pose.orientation.z

def get_mask(polygon):
    global map_size
    img = Image.new('L', (map_size, map_size), 0)
    ImageDraw.Draw(img).polygon(polygon, outline=255, fill=255)
    mask = np.array(img)   
    
    return mask

def apply_mask(map, mask):
    global map_size
    m = np.zeros((map_size, map_size))
    for i in range(map_size):
        for j in range(map_size):
            if mask[i,j]:
                m[i,j] = map[i,j]
    return m

def show_matrix(mat, name):
    im = np.asarray(mat)
    cv2.imshow(name, im)
    cv2.waitKey(10)

def init_proc(args):
    global angle_found
    angle_found = args

def check_angle(args):
    global angle_found
    if angle_found.value == 1:
        return None
    map = args[0]
    rect = args[1]
    angle = args[2]

    s = affinity.rotate(rect, -angle, origin=(int(map_size/2), map_size - 1))
    polygon = [(int(math.ceil(i)), int(math.ceil(j))) for i,j in list(s.exterior.coords)]

    mask = get_mask(polygon)
    m = apply_mask(map, mask)
    count = np.sum(m)
    if count == 0:
        angle_found.value = 1
        return angle
    else:
        return None

def find_safe_angle(map):
    global map_size
    global square_size
    global angle_found
    x1 = int(map_size/2) + 1
    y1 = map_size - square_size / 2 - 1
    x2 = x1 + square_size
    y2 = map_size + square_size/2 -1
    rect = Polygon(((x1, y1), (x1, y2), (x2, y2), (x2, y1)))

    tasks = list()

    for angle in range(0, 181, 10):
        tasks.append((map, rect, angle))

    angles = pool.map(check_angle, tasks)
    angles = sorted([a for a in angles if a])
    angle_found.value = 0

    if len(angles) > 0:
        return angles[0]
    else:
        return 0

rospy.init_node('yolo_ai')
rospy.Subscriber("/scan", LaserScan, rf_callback)
rospy.Subscriber("/robot_pose_ekf/odom", PoseWithCovarianceStamped)

angle_found = Value('i', 0)
pool = Pool(processes = cpu_count()/2, initializer = init_proc, initargs = (angle_found, ))

while not rospy.is_shutdown():
    if map is not None:
        print find_safe_angle(map)
    time.sleep(0.06)

rospy.spin()
