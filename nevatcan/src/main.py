#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan, Image
from LidarSensorSimple import LidarSensorSimple
from LidarSensor import LidarSensor
from cv_bridge import CvBridge
import numpy as np
import cv2
import collections
import time

lidar = LidarSensor()
lidarSimple = LidarSensorSimple()
bridge = CvBridge()

angle = lidar.angle #angle
numpoints = angle*3
lidar_data = [0]*numpoints

numClass = 5 #so lop khu nhieu
arr = collections.deque(maxlen=numClass)
isArr = False

rangeDetect = 3 #pham vi quet

#khu nhieu tin hieu
def denoiseLidar():
    global arr
    arr.append(lidar_data)
    new = [0] * numpoints
    for i in range(numpoints):
        tmp_count = 0
        tmp = None
        for j in range(1,len(arr)):
            if arr[j][i] >= lidar.centimet(rangeDetect):
                tmp_count += 1
            if arr[j][i] < lidar.centimet(rangeDetect):
                if tmp is None:
                    new[i] = int(arr[j][i])
                    tmp = new[i]
                else:
                    new[i] = int(0.5 * new[i] + 0.5 * arr[j][i])
                tmp_count = 0
        if tmp_count == len(arr)-1:
            new[i] = lidar.centimet(rangeDetect)
    return new

def lidar_callback(msg):
    global lidar_data, isArr
    rightArea = list(msg.ranges[359:359-angle-angle:-1])
    leftArea = list(msg.ranges[angle:0:-1])
    concat_data = np.hstack([leftArea,rightArea])
    lidar_data = [lidar.centimet(rangeDetect) if (str(i) == 'inf' or i > rangeDetect) else i * 100 for i in concat_data]
    isArr = True

rospy.init_node('lidar_detect', anonymous=True)
rospy.Subscriber('/scan', LaserScan, lidar_callback)

while not rospy.is_shutdown():
    if isArr:
        ref_lidarData = denoiseLidar()
        # print(ref_lidarData)
        # l, cObs, r, img = lidar.detectObject(ref_lidarData)
        forware, behind, side, imgS = lidarSimple.detectObject(ref_lidarData)
        # img = lidar.drawable(img)
        imgS = lidarSimple.drawable(imgS)
        # distance = lidar.dangerArea(cObs)
        distances = lidarSimple.dangerArea(forware,behind,side)
        # mapping = bridge.cv2_to_imgmsg(img, encoding="mono8")
        # pub_map.publish(mapping)
        # pub_danger.publish(distance)
        # cv2.imshow("img", img)
        cv2.imshow("imgS", imgS)
        cv2.waitKey(1)
    time.sleep(0.01)