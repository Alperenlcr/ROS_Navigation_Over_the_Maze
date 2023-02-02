#! /usr/bin/python3.8
from math import sqrt
from typing import Tuple
from numpy import average, mat, true_divide, zeros, uint8, pi
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import cv2
from collections import defaultdict
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import threading
from random import randint

class Graph():
    def __init__(self):
        """
        self.edges is a dict of all possible next nodes
        e.g. {'X': ['A', 'B', 'C', 'E'], ...}
        self.weights has all the weights between two nodes,
        with the two nodes as a tuple as the key
        e.g. {('X', 'A'): 7, ('X', 'B'): 2, ...}
        """
        self.edges = defaultdict(list)
        self.weights = {}
        self.nodes = [[]]
    def add_edge(self, from_node, to_node, weight):
        # Note: assumes edges are bi-directional
        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)
        self.weights[(from_node, to_node)] = weight
        self.weights[(to_node, from_node)] = weight


def dijsktra(graph, initial, end):
    # shortest paths is a dict of nodes
    # whose value is a tuple of (previous node, weight)
    shortest_paths = {initial: (None, 0)}
    current_node = initial
    visited = set()
    
    while current_node != end:
        visited.add(current_node)
        destinations = graph.edges[current_node]
        weight_to_current_node = shortest_paths[current_node][1]

        for next_node in destinations:
            weight = graph.weights[(current_node, next_node)] + weight_to_current_node
            if next_node not in shortest_paths:
                shortest_paths[next_node] = (current_node, weight)
            else:
                current_shortest_weight = shortest_paths[next_node][1]
                if current_shortest_weight > weight:
                    shortest_paths[next_node] = (current_node, weight)
        
        next_destinations = {node: shortest_paths[node] for node in shortest_paths if node not in visited}
        if not next_destinations:
            return "Route Not Possible"
        # next node is the destination with the lowest weight
        current_node = min(next_destinations, key=lambda k: next_destinations[k][1])
    
    # Work back through destinations in shortest path
    path = []
    while current_node is not None:
        path.append(current_node)
        next_node = shortest_paths[current_node][0]
        current_node = next_node
    # Reverse pathqq
    path = path[::-1]
    return path


def beyaz(m):
    for i in m:
        if 'b' in i:
            return i


def siyah(m):
    for i in m:
        if 's' in i:
            return i


def to_coordinate(x, y):
    return int(x*100)*100000+int(y*100)


def to_x_y(c):
    return (c-c%1000)/10000000,(c%1000)/100


def image_callback(msgImage):
    global img
    img = bridge.imgmsg_to_cv2(msgImage, "bgr8")


def left_laser_callback(msgScan):
    global left_laser
    if max(msgScan.ranges) < 0.8:
        left_laser = True
    else:
        left_laser = False


def small_laser_callback(msgScan):
    global small_laser
    if max(msgScan.ranges) < 0.8:
        small_laser = True
    else:
        small_laser = False


def front_laser_callback(msgScan):
    global front_laser
    global temp_laser
    temp_laser = msgScan.ranges


def transform_orientation(orientation_q):
    """
    Transform theta to [radians] from [quaternion orientation]
    """
    orientation_list = [ orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    if yaw < 0:
        yaw = 2 * pi + yaw  # 0->360 degrees >> 0->2pi
    return (yaw/(2*pi))*360


def odometry_callback(msgOdom):
    global odom
    odom[0] = msgOdom.pose.pose.position.x
    odom[1] = msgOdom.pose.pose.position.y
    odom[2] = transform_orientation(msgOdom.pose.pose.orientation)


def degree_dif(a, b):
    if a > b:
        if a-b < 180:
            return (a-b)
        else:
            return ((a-b)-360)
    else:
        if b-a < 180:
            return -(b-a)
        else:
            return -((b-a)-360)


def op(temp_laser):
    if temp_laser[50] > 1:
        front_laser["right"] = False
        front_laser["left"] = False
        return
# sifirla
# r = 50 den sonraki ilk 0 in indexi
    try:
        r = next(x[0] for x in enumerate(temp_laser[50:]) if x[1] > 1)
    except StopIteration:
        r = 50
# l = 50 den onceki ilk 0 in indexi
    try:
        l = next(x[0] for x in enumerate(temp_laser[:50][::-1]) if x[1] > 1)
    except StopIteration:
        l = 50
    #print(l,r)
    if r - l > 8:
        front_laser["left"] = True
        front_laser["right"] = False
    elif l - r > 8:
        front_laser["right"] = True
        front_laser["left"] = False
    else:
        front_laser["right"] = False
        front_laser["left"] = False


def get_info():
    while stop_thread[-1]:
        rospy.Subscriber("/camera1/image_raw", Image, image_callback)       # check if its on white or black
        rospy.Subscriber("/pose", Odometry, odometry_callback)              # to save for graph
        rospy.Subscriber("/front_scan", LaserScan, front_laser_callback)    # for path finding
        rospy.Subscriber("/left_scan", LaserScan, left_laser_callback)      # for path finding
        rospy.Subscriber("/small_scan", LaserScan, small_laser_callback)      # for path finding
        op(temp_laser)
        global img
        global hareket_duzeltme
        hareket_duzeltme = [False, False, False] # sol yap , stop, sag yap

        # sol scande yol cikarsa %10 hareket duzeltmeyi stopla
        #print(left_laser)
        #print(small_laser)
        #print(front_laser)

        if not small_laser or (left_laser and randint(1, 40) == 2):
            hareket_duzeltme[1] = True
        if front_laser["left"] == True:
            hareket_duzeltme[0] = True
        elif front_laser["right"] == True:
            hareket_duzeltme[2] = True
        #print(hareket_duzeltme)
        # ortala
    # ona gore hareket duzeltmeyi guncelle
        area = img[350:450, 350:450]
        average_color_row = average(area, axis=0)
        average_color = average(average_color_row, axis=0)
        #print(average_color)
        if temp_laser[0] != 0 and average_color[0] < 30 and average_color[1] < 30 and average_color[2] < 30:
            print("end")
            stop_thread.append(False)
            hareket_duzeltme[1] = True
        #print(average_color)
       # point1 = img[230, 320]
       # point2 = img[560, 320]
       # print(point1, point2)
        #img = cv2.circle(img, (230, 320), radius=0, color=(255, 255, 255), thickness=5)
        #img = cv2.circle(img, (560, 320), radius=0, color=(255, 255, 255), thickness=5)
        cv2.imshow('Cam1', area)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit()
        rospy.sleep(0.1)


def drive(command, x=0.5, z=0.15):
    #print(command,x,z)
    # Create a publisher which can "talk" to Turtlesim and tell it to move
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # Create a Twist message and add linear x and angular z values
    move_cmd = Twist()
    if command == "forward":
        move_cmd.linear.x = x
        move_cmd.angular.z = 0
    elif command == "left":
        move_cmd.linear.x = 0
        move_cmd.angular.z = z
    elif command == "right":
        move_cmd.linear.x = 0
        move_cmd.angular.z = -z
    elif command == "stop":
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
    rospy.sleep(0.1)
    pub.publish(move_cmd)


def search():
    while True:
        if hareket_duzeltme[0]:
            drive("left")
        elif hareket_duzeltme[2]:
            drive("right")
        elif hareket_duzeltme[1]:
            drive("stop")
            drive("stop")
            drive("stop")
            return
        else:
            drive("forward", 0.3)
        rospy.sleep(0.2)


def turn():
    angle = (odom[2]-270)
    if angle<0:
        angle += 360
    #print(odom[0], odom[1], angle)
    #print(odom[2], angle)
    noktaya_git(odom[0], odom[1], angle)
    print("start turning")
    while not (max(temp_laser[25:75]) < 0.8 and small_laser):
        drive("right")
        rospy.sleep(0.2)

    drive("stop")
    drive("stop")
    drive("stop")


def noktaya_git(x, y, angle):
    while True:
        angle_dif = degree_dif(odom[2], angle)
#        dis = sqrt((odom[0]-x)**2+(odom[1]-y)**2)
        #print(angle_dif, dis, 0.2+(abs(angle_dif)/432))
        #print(dis, angle)
#        if dis < 0.3 and abs(angle_dif) < 10:
        if abs(angle_dif) < 10:
            drive("stop")
            drive("stop")
            drive("stop")
            return
        elif abs(angle_dif) > 10:
            if angle_dif > 0:
                drive("right", z=0.2+(abs(angle_dif)/432))
            else:
                drive("left", z=0.2+(abs(angle_dif)/432))
#        else:
#            drive("forward")


hareket_duzeltme = [False, False, False] # sol yap , stop, sag yap
stop_thread = [True]
rospy.init_node("MAIN")
bridge = CvBridge()
img = zeros([800, 800, 4], dtype=uint8)
odom = [0, 0, 0]
temp_laser = [0 for i in range(100)]
front_laser = {'left':False, 'right':False}
left_laser = False
small_laser = False
t1 = threading.Thread(target=get_info, args=())


##########################################################################
##########################################################################
##########################################################################
##########################################################################
# image processing path finding
##########################################################################
##########################################################################
##########################################################################

# def get_info():
#     rospy.Subscriber("/camera1/image_raw", Image, image_callback)
#     rospy.Subscriber("/pose", Odometry, odometry_callback)
#     #print(odom)
#     end = False
#     global img
#     global hareket_duzeltme
#     hareket_duzeltme = [False, False, False] # sol yap , stop, sag yap
#     area = img[0:200, 220:580]
#     mid_area = img[0:10, 340:460]
#     gray = [False, False, False]
#     class BreakIt(Exception): pass
#     try:
#         for row in mid_area:
#             for pixel in row:
#                 if (pixel[0] < 170 and pixel[0] > 140) and (pixel[1] < 170 and pixel[1] > 140) and (pixel[2] < 170 and pixel[2] > 140):
#                     hareket_duzeltme[1] = True
#                     gray[1] = True
#                     cv2.putText(img=img, text='Road ends', org=(240, 100), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(0, 0, 0),thickness=1)
#                     raise BreakIt
#     except BreakIt:
#         pass
#     if not gray[1]:
#         left_area = img[0:10, 220:340]
#         #cv2.imshow('Cam3', left_area)

#         try:
#             for row in left_area:
#                 for pixel in row:
#                     if (pixel[0] < 170 and pixel[0] > 140) and (pixel[1] < 170 and pixel[1] > 140) and (pixel[2] < 170 and pixel[2] > 140):
#                         gray[0] = True
#                         raise BreakIt
#         except BreakIt:
#             pass
#         right_area = img[0:10, 460:580]
#         #cv2.imshow('Cam4', right_area)

#         try:
#             for row in right_area:
#                 for pixel in row:
#                     if (pixel[0] < 170 and pixel[0] > 140) and (pixel[1] < 170 and pixel[1] > 140) and (pixel[2] < 170 and pixel[2] > 140):
#                         gray[2] = True
#                         raise BreakIt
#         except BreakIt:
#             pass
#         if gray[0] and gray[2]:
#             end = True
#         elif gray[0]:
#             hareket_duzeltme[2] = True
#             cv2.putText(img=img, text='Right', org=(240, 100), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(0, 0, 0),thickness=1)
#         elif gray[2]:
#             hareket_duzeltme[0] = True
#             cv2.putText(img=img, text='Left', org=(240, 100), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(0, 0, 0),thickness=1)
#     if end:
#         hareket_duzeltme[1] = True
#         cv2.putText(img=img, text='Road ends', org=(240, 100), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(0, 0, 0),thickness=1)
#     if not hareket_duzeltme[0] and not hareket_duzeltme[1] and not hareket_duzeltme[2]:
#         try:
#             for row in area:
#                 for pixel in row:
#                     if (pixel[0] < 170 and pixel[0] > 140) and (pixel[1] < 170 and pixel[1] > 140) and (pixel[2] < 170 and pixel[2] > 140):
#                         hareket_duzeltme[1] = True
#                         gray[1] = True
#                         cv2.putText(img=img, text='Road ends', org=(240, 100), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(0, 0, 0),thickness=1)
#                         raise BreakIt
#         except BreakIt:
#             pass
#     img = cv2.rectangle(img, (220, 0), (580, 200), (0, 0, 0), 2)
#     #print(gray)
    
#     #print(len(area), len(area[0]))
#     #print(list(area))
#     cv2.imshow('Cam1', img)
#     #cv2.imshow('Cam2', area)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         exit()
#     rospy.sleep(0.2)
    
# #    print(stop_thread[-1])
#     if stop_thread[-1]:
#         get_info()


# def search():
#     global hareket_duzeltme
#     if hareket_duzeltme[0]:
#         drive("left")
#     elif hareket_duzeltme[2]:
#         drive("right")
#     elif hareket_duzeltme[1]:
#         drive("stop")
#         drive("stop")
#         drive("stop")
#         return
#     else:
#         drive("forward", 0.3)
#     rospy.sleep(0.2)
#     search()


# def turn():
#     angle = (odom[2]-240)
#     if angle<0:
#         angle += 360
#     print(odom[0], odom[1], angle)
#     noktaya_git(odom[0], odom[1], angle)
#     global hareket_duzeltme
#     while not (not hareket_duzeltme[1] and not hareket_duzeltme[0] and not hareket_duzeltme[2]):
#         drive("right")
#         rospy.sleep(0.2)

#     drive("stop")
#     drive("stop")
#     drive("stop")