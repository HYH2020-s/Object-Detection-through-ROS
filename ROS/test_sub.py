#!/usr/bin/env python
# coding: utf-8

import numpy as np
from numpy import uint8
import sys
import base64
import time
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from beginner_tutorials.msg import BBox

def callback(data):
    # try:
    #     sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
    # except:
    #     pass
    # import cv2
    # starting_time = time.time()
    # img_string = (data.data)
    #
    # b64_code = bytes(img_string, encoding='utf-8')
    # bytes_decode = base64.b64decode(b64_code)
    #
    # #img = np.frombuffer(bytes_decode, np.uint8)
    # #img.shape = 720, 1280, 3
    # file = open('1.jpg', "wb")
    # file.write(bytes_decode)
    # img = cv2.imread('1.jpg')
    # cv2.imshow('img',img)
    # cv2.waitKey(3)
    # elapsed_time = time.time() - starting_time
    rospy.loginfo(rospy.get_caller_id() + 'I heard x:%s, y:%s, h:%s, y:%s', str(data.x), str(data.y), str(data.h), str(data.w))

def img_sub():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("BBox", BBox, callback)
    rospy.spin()

if __name__ == '__main__':
    img_sub()




